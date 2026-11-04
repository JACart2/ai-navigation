#!/usr/bin/env python3
"""V8 post-processing. All inputs remain immutable; artifacts are run-local."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import sys
import time

import numpy as np
import yaml

from pipeline import (audit_reconstruction, closure_result, dense_config, nominate_pairs,
                      read_frames, refinement_configs, validate_georef, validate_metric_map)

HERE = Path(__file__).resolve().parent
SCRIPTS = HERE.parent


def parser():
    p = argparse.ArgumentParser(description='V8: synchronous E08 reconstruction, roof stripping, '
                                'dense/endpoint/narrow correction, leveling and georeferenced export.')
    p.add_argument('test_name')
    p.add_argument('--workflow', choices=['e08', 'v7'], default='e08', help='Default: e08; v7 preserves the old waypoint workflow')
    p.add_argument('--bag', type=Path)
    p.add_argument('--bag-dir', type=Path)
    p.add_argument('--maps-dir', type=Path)
    p.add_argument('--lidar-topic', default='/velodyne_points')
    p.add_argument('--imu-topic', default='/zed_front/zed_node_0/imu/data')
    p.add_argument('--gnss-topic', default='/fix')
    p.add_argument('--base-frame', default='base_link')
    for name in ['imu-roll-pitch', 'imu-deskew']:
        p.add_argument('--use-'+name, dest=name.replace('-', '_'), action='store_true', default=True)
        p.add_argument('--no-'+name, dest=name.replace('-', '_'), action='store_false')
    p.add_argument('--pipeline', type=Path, default=SCRIPTS/'loop_closure_waypoint_retroactive.yaml')
    p.add_argument('--localization-pipeline', type=Path)
    p.add_argument('--loop-start-times')
    p.add_argument('--loop-end-times')
    p.add_argument('--loop-start-keyframes')
    p.add_argument('--loop-end-keyframes')
    p.add_argument('--loop-traversal', choices=['same', 'reverse'], default='reverse')
    p.add_argument('--loop-waypoint-pairs', type=int, default=6)
    p.add_argument('--endpoint-max-distance', type=float, default=1.5, help='Maximum nomination distance in corrected geometry (metres)')
    p.add_argument('--roof-length', type=float, default=1.4732, help='Roof length in metres (58 inches)')
    p.add_argument('--roof-width', type=float, default=1.016, help='Roof width in metres (40 inches)')
    p.add_argument('--roof-max-drop', type=float, default=.1016, help='Maximum roof drop below front-centred LiDAR (metres)')
    p.add_argument('--no-roof-filter', action='store_true', help='Explicitly disable roof stripping for a different mounting')
    p.add_argument('--reconstruction-trace', type=Path, help='Trace paired with a reused offline SimpleMap')
    for name in ['simplemap-output', 'lc-simplemap-output', 'georef-output', 'georef-mm-output']:
        p.add_argument('--'+name, type=Path)
    for name in ['save-seed-loop-simplemap', 'save-unleveled-simplemap', 'save-georef-file',
                 'save-trajectory-files', 'allow-existing-output', 'dry-run', 'skip-offline-map',
                 'skip-loop-closure', 'skip-georef', 'skip-georef-mm', 'legacy-auto-endpoint-loop',
                 'no-auto-endpoint-loop']:
        p.add_argument('--'+name, action='store_true')
    p.add_argument('--endpoint-window-percent', default='5.0', help='Legacy v6 mode only')
    p.add_argument('--endpoint-loop-pairs', default='3', help='Legacy v6 mode only')
    return p


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def legacy(argv):
    args = list(argv)
    if '--workflow' in args:
        i = args.index('--workflow'); del args[i:i+2]
    args = [v for v in args if not v.startswith('--workflow=')]
    env = dict(os.environ, MOLA_POST_PROCESSING_LEGACY='1')
    os.execve(SCRIPTS/'launch_mola_post_processing', [str(SCRIPTS/'launch_mola_post_processing'), *args], env)


def bag_info(bag, topic):
    metadata = bag/'metadata.yaml'
    if not metadata.is_file():
        raise ValueError(f'Bag directory with metadata.yaml required: {bag}')
    data = yaml.safe_load(metadata.read_text())['rosbag2_bagfile_information']
    counts = {q['topic_metadata']['name']: q['message_count'] for q in data['topics_with_message_count']}
    if counts.get(topic, 0) < 2:
        raise ValueError(f'Bag has no usable {topic} scan count')
    return counts, data['duration']['nanoseconds']/1e9


class Workflow:
    def __init__(self, a):
        self.a = a
        self.ws = Path(os.environ.get('DEV_WS', '/root/dev_ws')).resolve()
        self.maps = (a.maps_dir or self.ws/'maps').resolve()
        self.bag = (a.bag or (a.bag_dir or self.ws/'bagfiles')/a.test_name).resolve()
        self.root = self.maps/(a.test_name+'_post_processing')
        self.recon = self.root/'reconstruction'
        self.dense = self.root/'e08'
        self.endpoint = self.root/'endpoint'
        self.narrow = self.root/'narrow'
        self.native_build = self.ws/'build/post_processing_tools'
        self.tools = self.native_build/'simplemap_tools'
        self.roof_tool = self.native_build/'filter_cart_roof'
        self.sm = (a.simplemap_output or self.maps/(a.test_name+'_offline.simplemap')).resolve()
        self.lc = (a.lc_simplemap_output or self.maps/(a.test_name+'_lc.simplemap')).resolve()
        self.mm = (a.georef_mm_output or self.maps/(a.test_name+'_georef_localization.mm')).resolve()
        self.geo = (a.georef_output or (self.maps/(a.test_name+'.georef') if a.save_georef_file else self.root/'georef.yaml')).resolve()
        self.seed = self.maps/(a.test_name+'_lc_seed.simplemap') if a.save_seed_loop_simplemap else self.endpoint/'map.simplemap'
        self.unleveled = (self.lc.with_name(self.lc.stem+'_unleveled.simplemap')
                          if a.save_unleveled_simplemap else self.narrow/'map.simplemap')
        self.loc_pipeline = (a.localization_pipeline or self.ws/'maps/sm2mm_gicp_localmap.yaml').resolve()
        self.seed_log = self.maps/(a.test_name+'_lc_seed.log')
        self.narrow_log = self.maps/(a.test_name+'_lc_retroactive.log')
        self.roof_log = self.maps/(a.test_name+'_roof_filter.log')
        self.dense_log = self.maps/(a.test_name+'_lc_e08.log')
        self.manifest = dict(workflow='e08-v8', started=time.time(), bag=str(self.bag),
                             input_simplemap=str(self.sm), steps=[], status='running')
        self.started = False

    def command(self, args, log, cwd=None):
        args = list(map(str, args))
        print('+ '+shlex.join(args), flush=True)
        if self.a.dry_run:
            return ''
        log.parent.mkdir(parents=True, exist_ok=True)
        item = dict(command=args, cwd=str(cwd or self.root), log=str(log), started=time.time())
        self.manifest['steps'].append(item)
        with log.open('w') as stream:
            proc = subprocess.run(args, cwd=cwd or self.root, stdout=stream, stderr=subprocess.STDOUT)
        item.update(exit_code=proc.returncode, elapsed_s=time.time()-item['started'])
        self.save_manifest()
        if proc.returncode:
            raise RuntimeError(f'Command failed ({proc.returncode}); inspect {log}')
        return log.read_text(errors='replace')

    def save_manifest(self):
        if not self.a.dry_run:
            path = self.root/('reconstruction_manifest.json' if self.a.skip_loop_closure and not self.a.skip_offline_map else 'processing_manifest.json')
            path.write_text(json.dumps(self.manifest, indent=2)+'\n')

    def write_yaml(self, path, cfg):
        print(f'Configuration: {path}', flush=True)
        if not self.a.dry_run:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(yaml.safe_dump(cfg, sort_keys=False))

    def export_frames(self, sm, output):
        self.command(['sm-cli', 'export-keyframes', sm, '--output', output], output.with_suffix('.log'))
        return None if self.a.dry_run else read_frames(output)

    def inspect(self, sm, label):
        log = self.command([self.tools, 'inspect', sm], self.root/(label+'_info.log'))
        if self.a.dry_run:
            return None
        matches = re.findall(r'^POST_PROCESSING_JSON: (.+)$', log, re.M)
        if not matches:
            raise ValueError(f'Native SimpleMap inspection failed: {sm}')
        return json.loads(matches[-1])

    def unchanged(self, before, after, before_stats, label, poses=False):
        frames = self.export_frames(after, self.root/(label+'_keyframes.tum'))
        stats = self.inspect(after, label)
        if not self.a.dry_run:
            if frames.shape != before.shape or not np.array_equal(frames[:, 0], before[:, 0]):
                raise ValueError(f'{label}: keyframe identity changed')
            if poses and not np.array_equal(frames, before):
                raise ValueError(f'{label}: poses changed unexpectedly')
            if (stats['gnss_observations'], stats['gnss_digest']) != (before_stats['gnss_observations'], before_stats['gnss_digest']):
                raise ValueError(f'{label}: GNSS observations changed or were lost')
        return frames, stats

    def preflight(self):
        a = self.a
        if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]*', a.test_name):
            raise ValueError('TEST_NAME must contain only letters, numbers, dots, underscores or hyphens')
        for name, value in [('roof length', a.roof_length), ('roof width', a.roof_width),
                            ('endpoint nomination distance', a.endpoint_max_distance)]:
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f'Invalid {name}')
        if not np.isfinite(a.roof_max_drop) or a.roof_max_drop < 0:
            raise ValueError('Invalid roof drop')
        self.counts, self.bag_duration = bag_info(self.bag, a.lidar_topic)
        if not a.skip_offline_map and a.imu_deskew and not self.counts.get(a.imu_topic):
            raise ValueError('IMU deskew requires the configured IMU topic in the bag')
        if not a.skip_offline_map and a.imu_roll_pitch and not self.counts.get(a.imu_topic):
            raise ValueError('IMU initialization requires the configured IMU topic in the bag')
        if not a.skip_loop_closure:
            timestamps = a.loop_start_times or a.loop_end_times
            if timestamps:
                if not (a.loop_start_times and a.loop_end_times) or a.loop_start_keyframes or a.loop_end_keyframes:
                    raise ValueError('Provide both timestamp ranges without keyframe ranges')
                self.start, self.end, self.mode = a.loop_start_times, a.loop_end_times, 'timestamps'
            else:
                if not (a.loop_start_keyframes and a.loop_end_keyframes):
                    raise ValueError('Provide both overlap keyframe ranges or both timestamp ranges; use the offline review view')
                self.start, self.end, self.mode = a.loop_start_keyframes, a.loop_end_keyframes, 'keyframes'
            if a.loop_waypoint_pairs < 4:
                raise ValueError('Endpoint consensus requires at least four pairs')
        for required in ([a.pipeline] if not a.skip_loop_closure else []) + ([self.loc_pipeline] if not a.skip_georef_mm else []):
            if not required.is_file():
                raise ValueError(f'Missing pipeline: {required}')
        if a.skip_offline_map and not a.skip_loop_closure and not self.sm.is_file():
            raise ValueError(f'Missing input SimpleMap: {self.sm}')
        if a.skip_loop_closure and (not a.skip_georef or not a.skip_georef_mm) and not self.lc.is_file():
            raise ValueError(f'Missing final SimpleMap for skipped loop-closure stage: {self.lc}')
        if a.skip_georef and not a.skip_georef_mm and not self.geo.is_file():
            raise ValueError('Skipped georeferencing requires an existing --georef-output')
        outputs = []
        if not a.skip_offline_map:
            outputs += [self.sm, self.recon/'raw.simplemap', self.recon/'pipeline.yaml', self.recon/'replay.log',
                        self.recon/'trace.csv', self.recon/'overlap_review.html']
        if not a.skip_loop_closure:
            outputs += [self.lc, self.seed, self.unleveled, self.dense/'map.simplemap',
                        self.dense/'pipeline.yaml', self.seed_log, self.narrow_log, self.dense_log]
            if not a.no_roof_filter:
                outputs += [self.root/'roof.simplemap', self.roof_log]
        if not a.skip_georef:
            outputs += [self.geo]
        if not a.skip_georef_mm:
            outputs += [self.mm]
        outputs = [p.resolve() for p in outputs]
        if len(set(outputs)) != len(outputs):
            raise ValueError('Output paths must be distinct')
        protected = {self.sm} if a.skip_offline_map else set()
        if a.skip_loop_closure: protected.add(self.lc)
        if a.skip_georef: protected.add(self.geo)
        protected.update([a.pipeline.resolve(), self.loc_pipeline])
        if set(outputs) & protected or any(self.bag == p or self.bag in p.parents for p in outputs):
            raise ValueError('An output aliases an input or lies inside the original bag')
        for path in outputs:
            if path.exists() and not a.allow_existing_output:
                raise ValueError(f'Output already exists: {path}; choose a new name or explicitly allow replacement')
        executables = ['cmake', 'sm-cli'] + ([] if a.skip_loop_closure else ['mola-sm-lc-cli']) + ([] if a.skip_offline_map else ['mola-lidar-odometry-cli'])
        if not a.skip_georef: executables.append('mola-sm-georeferencing')
        if not a.skip_georef_mm: executables += ['sm2mm', 'mm-info']
        for exe in executables:
            if not shutil.which(exe):
                raise ValueError(f'Required executable missing: {exe}')
        self.outputs = outputs

    def dependencies(self):
        a = self.a
        if not a.skip_loop_closure:
            exe = Path(shutil.which('mola-sm-lc-cli')).resolve()
            ldd = subprocess.check_output(['ldd', str(exe)], text=True)
            match = re.search(r'libmola_sm_loop_closure\.so\S* => (\S+)', ldd)
            if not match:
                raise ValueError('Cannot identify the loaded loop-closure library')
            lib = Path(match[1]).resolve()
            content = lib.read_bytes()
            features = [b'loop_submap_half_length_m', b'registration_remove_ground',
                        b'registration_voxel_size', b'candidate_frame_stride',
                        b'manual_hypothesis_min_inliers', b'KISS-Matcher enabled:']
            if not all(feature in content for feature in features):
                raise ValueError('Loaded loop-closure library lacks required E08 features; see post_processing/README.md')
            self.manifest['loop_closure'] = dict(executable=str(exe), sha256=digest(exe),
                                                  library=str(lib), library_sha256=digest(lib), ldd=ldd)
        self.command(['cmake', '-S', HERE, '-B', self.native_build], self.root/'native_configure.log')
        self.command(['cmake', '--build', self.native_build, '-j2'], self.root/'native_build.log')

    def audit(self, check_map=True):
        a = self.a
        trace = a.reconstruction_trace or self.recon/'trace.csv'
        log = trace.parent/'replay.log'
        if a.skip_offline_map and not trace.is_file():
            trace, log = self.sm.parent/'trace.csv', self.sm.parent/'replay.log'
        if a.dry_run:
            print(f'Validate complete reconstruction and tracking: {trace}')
            return
        if not trace.is_file() or not log.is_file():
            raise ValueError('Reused maps require their reconstruction trace and replay.log; supply --reconstruction-trace')
        result = audit_reconstruction(trace, log.read_text(errors='replace'), self.counts[a.lidar_topic], self.bag_duration)
        # Reject a complete trace from a different interval attached to a partial map.
        if check_map and self.sm.is_file():
            frames = self.export_frames(self.sm, self.root/'audit_keyframes.tum')
            samples = np.atleast_1d(np.genfromtxt(trace, delimiter=',', names=True))
            if frames[0, 0] < samples['timestamp'][0]-1 or frames[-1, 0] > samples['timestamp'][-1]+1 or frames[-1, 0] < samples['timestamp'][-1]-10:
                result['passed'] = False
                result['keyframe_tail_mismatch'] = True
        result['trace'] = str(trace)
        (self.root/'reconstruction_audit.json').write_text(json.dumps(result, indent=2)+'\n')
        if not result['passed']:
            raise ValueError(f'Reconstruction is incomplete or unhealthy; inspect {self.root / "reconstruction_audit.json"}')

    def reconstruct(self):
        a = self.a
        base = Path('/opt/ros/jazzy/share/mola_lidar_odometry')
        cfg = yaml.safe_load((base/'pipelines/lidar3d-gicp.yaml').read_text())
        cfg['localmap_generator'][0]['params']['metric_map_definition']['creationOpts'].update(
            max_search_keyframes=12, num_diverse_keyframes=4)
        cfg['params']['debug_traces'] = dict(save_to_file=True, output_file=str(self.recon/'trace.csv'))
        cfg['params']['estimated_trajectory'] = dict(save_to_file=True, output_file=str(self.recon/'trajectory.tum'))
        self.write_yaml(self.recon/'pipeline.yaml', cfg)
        self.command(['mola-lidar-odometry-cli', '--input-rosbag2', self.bag, '-c', self.recon/'pipeline.yaml',
                      '--lidar-sensor-label', a.lidar_topic, '--imu-sensor-label', a.imu_topic,
                      '--base-link-frame-id', a.base_frame, '--state-estimator-param-file',
                      base/'state-estimator-params/state-estimation-simple.yaml',
                      '--output-simplemap', self.recon/'raw.simplemap',
                      '--output-tum-path', self.recon/'trajectory.tum'], self.recon/'replay.log', self.recon)
        self.audit(check_map=False)
        before = self.export_frames(self.recon/'raw.simplemap', self.recon/'raw_keyframes.tum')
        sensor_cfg = dict(params=dict(rosbag_filename=str(self.bag), base_link_frame_id=a.base_frame,
                                     sensors=[dict(topic=a.gnss_topic, sensorLabel='gps', is_optional=True)]))
        self.write_yaml(self.recon/'gnss_dataset.yaml', sensor_cfg)
        # Save to a new path first. Atomic replace is used only for explicit output replacement.
        staged = self.recon/'gnss_preserved.simplemap'
        if not a.dry_run and staged.exists(): staged.unlink()
        self.command([self.tools, 'attach-bag', self.recon/'gnss_dataset.yaml',
                      self.recon/'raw.simplemap', staged, '1.0'], self.recon/'gnss_preserve.log')
        after = self.export_frames(staged, self.recon/'keyframes.tum')
        if not a.dry_run:
            if not np.array_equal(before, after):
                raise ValueError('GNSS attachment changed keyframe poses')
            self.sm.parent.mkdir(parents=True, exist_ok=True)
            os.replace(staged, self.sm)
            review_path = self.recon/'overlap_review.html'
            if review_path.exists(): review_path.unlink()
        self.command([sys.executable, HERE/'review.py', self.sm, '--output', self.recon/'overlap_review.html'],
                     self.recon/'review.log')
        print(f'Overlap review: {self.recon / "overlap_review.html"}')

    def correct(self):
        a = self.a
        self.audit()
        initial = self.export_frames(self.sm, self.root/'input_keyframes.tum')
        stats = self.inspect(self.sm, 'input')
        source = self.sm
        if not a.no_roof_filter:
            self.command([self.roof_tool, '--self-test'], self.root/'roof_self_test.log')
            roof = self.root/'roof.simplemap'
            if not a.dry_run and roof.exists(): roof.unlink()  # checked explicit replacement in preflight
            self.command([self.roof_tool, self.sm, roof, a.roof_length, a.roof_width, a.roof_max_drop], self.roof_log)
            self.unchanged(initial, roof, stats, 'roof', poses=True)
            source = roof
        cfg = dense_config(yaml.safe_load(a.pipeline.read_text()))
        self.write_yaml(self.dense/'pipeline.yaml', cfg)
        self.close(source, self.dense/'map.simplemap', self.dense/'pipeline.yaml', self.dense_log, self.dense)
        frames, stats = self.unchanged(initial, self.dense/'map.simplemap', stats, 'e08')
        if a.dry_run:
            print('Nominate distinct ordered endpoint correspondences within '+self.start+' / '+self.end)
            pairs = []
        else:
            pairs = nominate_pairs(frames, self.start, self.end, self.mode, a.loop_traversal,
                                   a.loop_waypoint_pairs, a.endpoint_max_distance)
            (self.endpoint/'pairs.json').write_text(json.dumps(pairs, indent=2)+'\n')
        seed, narrow = refinement_configs(cfg, pairs)
        self.write_yaml(self.endpoint/'pipeline.yaml', seed)
        self.write_yaml(self.narrow/'pipeline.yaml', narrow)
        self.close(self.dense/'map.simplemap', self.seed, self.endpoint/'pipeline.yaml', self.seed_log, self.endpoint, seed=True)
        self.unchanged(initial, self.seed, stats, 'seed')
        self.close(self.seed, self.unleveled, self.narrow/'pipeline.yaml', self.narrow_log, self.narrow)
        self.unchanged(initial, self.unleveled, stats, 'narrow')
        self.command(['sm-cli', 'level', self.unleveled, self.lc], self.root/'level.log')
        self.unchanged(initial, self.lc, stats, 'leveled')

    def close(self, source, output, cfg, log, cwd, seed=False):
        text = self.command(['mola-sm-lc-cli', '-a', 'mola::FrameToFrameLoopClosure',
                             '-i', source, '-o', output, '-p', cfg, '-v', 'DEBUG'], log, cwd)
        if not self.a.dry_run:
            self.manifest.setdefault('corrections', {})[cwd.name] = closure_result(text, seed)

    def export(self):
        a = self.a
        if a.skip_georef and a.skip_georef_mm:
            return
        stats = self.inspect(self.lc, 'final')
        if not a.skip_georef:
            text = self.command(['mola-sm-georeferencing', '-i', self.lc, '-o', self.geo], self.root/'georef.log')
            if not a.dry_run:
                validate_georef(yaml.safe_load(self.geo.read_text()), text, stats)
        elif not a.dry_run:
            # A reused sidecar must still agree with actual observed GNSS.
            validate_georef(yaml.safe_load(self.geo.read_text()), f'Found: {stats["usable_gnss"]} GNSS frames', stats)
        if not a.skip_georef_mm:
            # Stage export before publishing the requested final path.
            staged = self.root/'validated_localization.mm'
            self.command(['sm2mm', '-i', self.lc, '-o', staged, '-p', self.loc_pipeline,
                          '-l', 'libmola_metric_maps.so', '-g', self.geo], self.root/'sm2mm.log')
            info = self.command(['mm-info', '-l', 'libmola_metric_maps.so', staged], self.root/'mm_info.log')
            if not a.dry_run:
                validate_metric_map(info, stats['keyframes'], yaml.safe_load(self.geo.read_text()))
                self.mm.parent.mkdir(parents=True, exist_ok=True)
                os.replace(staged, self.mm)

    def execute(self):
        self.preflight()
        if not self.a.dry_run:
            self.root.mkdir(parents=True, exist_ok=True)
            self.lock = (self.root/'.lock').open('a')
            fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.preflight()  # Check again under the per-run lock before any outputs.
            self.started = True
            for path in (self.recon, self.dense, self.endpoint, self.narrow): path.mkdir(exist_ok=True)
            for path in self.outputs: path.parent.mkdir(parents=True, exist_ok=True)
        self.dependencies()
        if not self.a.skip_offline_map: self.reconstruct()
        if not self.a.skip_loop_closure: self.correct()
        self.export()
        self.manifest['status'] = 'complete'
        self.manifest['finished'] = time.time()
        self.save_manifest()
        if not self.a.dry_run:
            # Preserve diagnostics and intermediates on every failure; clean optional
            # seed/unleveled/georef files only after all requested checks pass.
            cleanups = []
            if not self.a.skip_loop_closure:
                cleanups += [(self.a.save_seed_loop_simplemap, self.seed), (self.a.save_unleveled_simplemap, self.unleveled)]
            if not self.a.skip_georef:
                cleanups += [(self.a.save_georef_file or self.a.georef_output is not None, self.geo)]
            for keep, path in cleanups:
                if not keep and path.is_file(): path.unlink()
        print(f'Artifacts and diagnostics: {self.root}')
        print(f'Offline SimpleMap: {self.sm}\nFinal SimpleMap: {self.lc}\nLocalization map: {self.mm}')
        if not self.a.dry_run: self.lock.close()


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    a = parser().parse_args(argv)
    if a.workflow == 'v7' or a.legacy_auto_endpoint_loop:
        legacy(argv)
    if a.no_auto_endpoint_loop:
        raise ValueError('--no-auto-endpoint-loop is legacy-only; select --workflow v7')
    # Explicit run-local values prevent ambient mapping/recovery settings from
    # silently changing the tested E08 reconstruction.
    values = dict(MOLA_MAPPING_ENABLED='True', MOLA_START_ACTIVE='True',
                  MOLA_GENERATE_SIMPLEMAP='True', MOLA_QUIT_ON_DATASET_END='true',
                  MOLA_SIMPLEMAP_MIN_XYZ='1.0', MOLA_SIMPLEMAP_MIN_ROT='20.0',
                  MOLA_MIN_XYZ_BETWEEN_MAP_UPDATES='0.25', MOLA_MIN_ROT_BETWEEN_MAP_UPDATES='5.0',
                  MOLA_PUBLISH_LOCAL_MAP_UPDATES_EVERY_N='40', MOLA_TF_BASE_LINK=a.base_frame,
                  MOLA_NAVSTATE_ENFORCE_PLANAR_MOTION='false',
                  MOLA_NAVSTATE_IMU_SENSOR_NAME='.*' if a.imu_roll_pitch or a.imu_deskew else '__disabled_for_mapping__',
                  MOLA_OBS_VALIDITY_MIN_POINTS='100', MOLA_ENABLE_OBS_VALIDITY_FILTER='True',
                  MOLA_IMU_TOPIC=a.imu_topic, MOLA_GNSS_TOPIC=a.gnss_topic, MOLA_GPS_NAME=a.gnss_topic,
                  MOLA_DESKEW_METHOD='MotionCompensationMethod::IMU' if a.imu_deskew else 'MotionCompensationMethod::Linear',
                  MOLA_LO_INITIAL_LOCALIZATION_METHOD='InitLocalization::PitchAndRollFromIMU' if a.imu_roll_pitch else 'InitLocalization::FixedPose',
                  MOLA_IMU_GRAVITY_CORRECTION='true' if a.imu_roll_pitch else 'false', MOLA_IMU_GRAVITY_SIGMA_DEG='3.0',
                  SAVE_TRAJECTORY_FILES_WITH_COV='true' if a.save_trajectory_files else 'false',
                  OMP_NUM_THREADS='4', OPENBLAS_NUM_THREADS='1')
    os.environ.update(values)
    for key in ('MOLA_LOAD_MM', 'MOLA_LOAD_SM', 'MOLA_SAVE_MM'): os.environ.pop(key, None)
    w = Workflow(a)
    w.manifest['environment'] = values
    try:
        w.execute()
    except (Exception, KeyboardInterrupt) as exc:
        w.manifest.update(status='failed', error=str(exc), finished=time.time())
        if w.started and not a.dry_run: w.save_manifest()
        raise
    finally:
        if hasattr(w, 'lock') and not w.lock.closed: w.lock.close()


if __name__ == '__main__':
    try:
        main()
    except (Exception, KeyboardInterrupt) as exc:
        print(f'Post-processing stopped: {exc}', file=sys.stderr)
        sys.exit(1)
