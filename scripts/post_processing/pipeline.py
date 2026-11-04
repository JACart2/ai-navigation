"""E08 configuration, geometric nominations and promotion checks.

No dependency sources or global settings are modified by these helpers.
"""
import copy
import json
import math
import re
from pathlib import Path

import numpy as np
import yaml


def dense_config(base):
    cfg = copy.deepcopy(base)
    p = cfg['params']
    p.update(use_gnss=False, assume_planar_world=False, use_kiss_matcher=True,
             kiss_matcher_resolution=0.7, kiss_matcher_layer='points_to_register_points',
             min_distance_between_frames=0.0, max_distance_for_lc_candidate=12.0,
             max_lc_candidates=30, min_frames_between_lc=25, candidate_frame_stride=3,
             max_lc_optimization_rounds=3, lc_optimize_every_n=4,
             lc_candidate_strategy='MULTI_OBJECTIVE', lc_verbose_candidate_selection=True,
             min_icp_goodness=0.35, threshold_sigma_initial=1.5, threshold_sigma_final=0.15,
             input_odometry_noise_xyz=0.04, input_odometry_noise_ang=0.10,
             loop_submap_half_window=5, loop_submap_half_length_m=2.5,
             registration_remove_ground=True, registration_ground_band=0.30,
             registration_voxel_size=0.15, manual_lc_identity_fallback=False,
             manual_hypothesis_min_inliers=1, scan_context_endpoint_search=False,
             save_trajectory_files=True, min_icp_goodness_to_save_icplog=2.0)
    p.pop('manual_loop_constraints', None)
    icp = cfg['icp_settings']
    icp['params']['saveIterationDetails'] = False
    icp['params'].pop('quality_checkpoints', None)
    icp['quality'][0]['params'] = {
        'reuse_icp_pairings': False, 'absolute_minimum_pairing_ratio': 0.10,
        'threshold': 0.30, 'thresholdAngularDeg': 0.0, 'pairingsPerPoint': 1,
        'allowMatchAlreadyMatchedGlobalPoints': True,
        'pointLayerMatches': [{'global': 'points_to_register_points',
                              'local': 'points_to_register_points', 'weight': 1.0}]}
    return cfg


def refinement_configs(dense, pairs):
    seed = copy.deepcopy(dense)
    seed['params'].update(
        manual_loop_constraints=[{'timestamp_i': q['time_i'], 'timestamp_j': q['time_j'],
                                  'sigma_xyz': 0.2 + 0.04*k, 'trust_as_inlier': False}
                                 for k, q in enumerate(pairs)],
        max_lc_optimization_rounds=0, loop_submap_half_window=15,
        loop_submap_half_length_m=0.0, registration_voxel_size=0.35,
        kiss_matcher_resolution=1.0, manual_hypothesis_min_inliers=4,
        manual_hypothesis_max_translation_spread=3.0,
        manual_hypothesis_max_rotation_spread_deg=6.0)
    narrow = copy.deepcopy(dense)
    narrow['params'].update(
        loop_submap_half_window=10, loop_submap_half_length_m=0.0,
        registration_voxel_size=0.35, kiss_matcher_resolution=1.0,
        min_frames_between_lc=30, min_distance_between_frames=0.25,
        max_distance_for_lc_candidate=2.5, max_lc_candidates=20,
        max_lc_optimization_rounds=3, lc_optimize_every_n=0)
    return seed, narrow


def read_frames(path):
    a = np.atleast_2d(np.loadtxt(path))
    if a.shape[1] != 8 or len(a) < 2 or not np.isfinite(a).all():
        raise ValueError(f'Invalid or incomplete keyframe trajectory: {path}')
    if np.any(np.diff(a[:, 0]) <= 0):
        raise ValueError('Keyframe timestamps must be strictly increasing')
    return a


def nominate_pairs(frames, start, end, mode, traversal, count=6, max_distance=1.5):
    def interval(value):
        a, b = map(float, value.split(':'))
        if not math.isfinite(a+b) or a >= b:
            raise ValueError(f'Invalid overlap range: {value}')
        if mode == 'timestamps':
            if a < frames[0, 0] or b > frames[-1, 0]:
                raise ValueError('Overlap timestamps are outside the offline trajectory')
            a, b = (int(np.argmin(abs(frames[:, 0]-v))) for v in (a, b))
        elif not a.is_integer() or not b.is_integer():
            raise ValueError('Keyframe ranges must contain integer IDs')
        a, b = int(a), int(b)
        if not 0 <= a < b < len(frames) or count > b-a+1:
            raise ValueError('Overlap range is too short or exceeds offline keyframes')
        return a, b
    sf, sl = interval(start)
    ef, el = interval(end)
    if sl >= ef:
        raise ValueError('First and final overlap traversals must be disjoint and ordered')
    if count < 4:
        raise ValueError('E08 endpoint consensus requires at least four nominated pairs')
    early = np.rint(np.linspace(sl if traversal == 'reverse' else sf,
                                sf if traversal == 'reverse' else sl, count)).astype(int)
    late = np.arange(ef, el+1)
    pairs = []
    for i in early:
        d = np.linalg.norm(frames[late, 1:4]-frames[i, 1:4], axis=1)
        j = int(late[np.argmin(d)])
        if float(d.min()) > max_distance:
            raise ValueError(f'No endpoint correspondence within {max_distance} m for frame {i}')
        pairs.append(dict(i=int(i), j=j, time_i=float(frames[i, 0]),
                          time_j=float(frames[j, 0]), map_distance_m=float(d.min())))
    if len({p['j'] for p in pairs}) != count or any(
            pairs[k]['j'] >= pairs[k+1]['j'] for k in range(count-1)):
        raise ValueError('Endpoint nominees are duplicated or violate traversal order; inspect overlap ranges')
    return pairs


def closure_result(log, seed=False):
    if 'KISS-Matcher enabled:' not in log or 'build lacks KISS' in log:
        raise ValueError('Required KISS-Matcher support was not active')
    gnc = re.findall(r'GNC result: (\d+) LC inlier\(s\), (\d+) LC outlier\(s\)', log)
    if not gnc or int(gnc[-1][0]) < (4 if seed else 1):
        raise ValueError('No sufficient final GNC-retained corrections')
    if seed:
        if not re.search(r'Manual LC hypothesis .* accepted atomically', log):
            raise ValueError('Endpoint hypothesis was not accepted atomically')
    else:
        accepted = re.findall(r'Total accepted loop closures: (\d+)', log)
        if not accepted or int(accepted[-1]) == 0:
            raise ValueError('No automatic loop corrections accepted')
    return dict(gnc_inliers=int(gnc[-1][0]), gnc_outliers=int(gnc[-1][1]))


def audit_reconstruction(trace_path, replay_log, recorded_scans, bag_duration):
    a = np.atleast_1d(np.genfromtxt(trace_path, delimiter=',', names=True))
    t, q = a['timestamp'], a['icp_quality']
    xyz = np.column_stack([a['robot_'+v] for v in 'xyz'])
    if len(t) < 2 or not np.isfinite(t).all() or not np.isfinite(q).all() or not np.isfinite(xyz).all():
        raise ValueError('Empty or nonfinite reconstruction trace')
    dt = np.diff(t)
    if np.any(dt <= 0):
        raise ValueError('Reconstruction trace has non-increasing timestamps')
    # Permit the small startup omissions observed in the original E08 replays.
    coverage = len(t)/recorded_scans
    longest = 0.0
    start = None
    for stamp, quality in zip(t, q):
        if quality < .5:
            start = stamp if start is None else start
            longest = max(longest, stamp-start)
        else:
            start = None
    steps = np.linalg.norm(np.diff(xyz, axis=0), axis=1)
    result = dict(processed_scans=len(t), recorded_scans=recorded_scans,
                  coverage=coverage, trace_duration_s=float(t[-1]-t[0]),
                  median_quality=float(np.median(q)), fraction_quality_ge_0_5=float(np.mean(q >= .5)),
                  longest_low_quality_s=float(longest), maximum_step_m=float(steps.max()))
    result['passed'] = bool(.98 <= coverage <= 1.01 and t[-1]-t[0] >= .98*bag_duration and
                            np.mean(q >= .5) >= .95 and longest <= 5 and steps.max() <= 8 and
                            re.search(r'100\.0+%', replay_log))
    return result


def validate_georef(data, log, stats):
    g = data.get('geo_coord', {})
    values = [g.get(k, float('nan')) for k in ('lat', 'lon', 'altitude')]
    if not data.get('defined') or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values):
        raise ValueError('Undefined or nonfinite geographic anchor')
    lat, lon, _ = values
    if not stats.get('usable_gnss') or not re.search(r'Found: [1-9][0-9]* GNSS frames', log):
        raise ValueError('Georeferencing has no usable GNSS observations')
    if abs(lat) > 90 or abs(lon) > 180 or abs(lat-stats['median_lat']) > .1 or abs(lon-stats['median_lon']) > .1:
        raise ValueError('Geographic anchor disagrees with recorded GNSS')
    transform = data.get('T_enu_to_map', {}).get('mean', {})
    if not all(isinstance(transform.get(k), (int, float)) and math.isfinite(transform[k])
               for k in ('x', 'y', 'z', 'yaw_deg', 'pitch_deg', 'roll_deg')):
        raise ValueError('Invalid georeferencing transform')


def validate_metric_map(log, keyframes, georef=None):
    matches = re.findall(r'KeyframePointCloudMap: ([0-9]+) keyframes', log)
    if re.search(r'exception|unregistered class', log, re.I) or not matches or max(map(int, matches)) != keyframes:
        raise ValueError('Native metric-map loading failed or has incomplete keyframe contents')
    if not re.search(r'geo|georef', log, re.I):
        raise ValueError('Native metric-map inspection did not report georeferencing')
    if georef is not None:
        anchor = re.search(r'\(([-+\d.]+)\s+([-+\d.]+)\)\s+h=', log)
        transform = re.search(r'T_enu_map=\[([^\]]+)\]', log)
        if not anchor or not transform:
            raise ValueError('Native map inspection lacks an anchor or ENU transform')
        if abs(float(anchor[1])-georef['geo_coord']['lat']) > 1e-6 or abs(float(anchor[2])-georef['geo_coord']['lon']) > 1e-6:
            raise ValueError('Exported map anchor disagrees with georeferencing')
        if any(abs(float(v)) > 1e-4 for v in transform[1].split()):
            raise ValueError('Exported map is not in the expected ENU coordinates')
