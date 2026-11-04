import copy
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pipeline import (audit_reconstruction, closure_result, dense_config, nominate_pairs,
                      refinement_configs, validate_georef, validate_metric_map)
from run import Workflow, parser
HERE = Path(__file__).resolve().parents[1]


class RecipeTests(unittest.TestCase):
    def test_matches_retained_e08_recipe(self):
        base = yaml.safe_load((HERE.parent/'loop_closure_waypoint_retroactive.yaml').read_text())
        original = copy.deepcopy(base)
        reference = yaml.safe_load((HERE/'tests/data/e08_dense.yaml').read_text())
        self.assertEqual(dense_config(base), reference)
        self.assertEqual(base, original)

    def frames(self):
        frames = np.zeros((12, 8)); frames[:, 0] = np.arange(12)+1000
        frames[:6, 1] = np.arange(6)
        frames[6:, 1] = np.arange(6)[::-1]+.1
        frames[:, 7] = 1
        return frames

    def test_reverse_geometric_pairs_and_timestamp_resolution(self):
        frames = self.frames()
        pairs = nominate_pairs(frames, '0:5', '6:11', 'keyframes', 'reverse')
        self.assertEqual([(q['i'], q['j']) for q in pairs], [(5-k, 6+k) for k in range(6)])
        self.assertEqual(pairs, nominate_pairs(frames, '1000:1005', '1006:1011', 'timestamps', 'reverse'))

    def test_same_direction(self):
        frames = self.frames(); frames[6:, 1] = np.arange(6)+.1
        self.assertEqual([q['j'] for q in nominate_pairs(frames, '0:5', '6:11', 'keyframes', 'same')], list(range(6, 12)))

    def test_reject_duplicate_far_and_out_of_range_nominees(self):
        frames = self.frames(); frames[6:, 1] = 2
        with self.assertRaises(ValueError): nominate_pairs(frames, '0:5', '6:11', 'keyframes', 'reverse', max_distance=10)
        frames[6:, 1] = 100
        with self.assertRaises(ValueError): nominate_pairs(frames, '0:5', '6:11', 'keyframes', 'reverse')
        with self.assertRaises(ValueError): nominate_pairs(self.frames(), '900:905', '1006:1011', 'timestamps', 'reverse')

    def test_positive_quality_and_nonplanar_refinements(self):
        dense = yaml.safe_load((HERE/'tests/data/e08_dense.yaml').read_text())
        pairs = nominate_pairs(self.frames(), '0:5', '6:11', 'keyframes', 'reverse')
        seed, narrow = refinement_configs(dense, pairs)
        self.assertEqual(seed['params']['manual_hypothesis_min_inliers'], 4)
        for cfg in (dense, seed, narrow):
            self.assertEqual(cfg['params']['min_icp_goodness'], .35)
            self.assertFalse(cfg['icp_settings']['quality'][0]['params']['reuse_icp_pairings'])
            self.assertFalse(cfg['params']['use_gnss'])
            self.assertFalse(cfg['params']['assume_planar_world'])


class PromotionTests(unittest.TestCase):
    def test_reject_zero_final_inliers_despite_accepted_registrations(self):
        log = 'KISS-Matcher enabled:\nTotal accepted loop closures: 10\nGNC result: 0 LC inlier(s), 10 LC outlier(s)'
        with self.assertRaises(ValueError): closure_result(log)
        self.assertEqual(closure_result(log.replace('0 LC inlier(s), 10', '8 LC inlier(s), 2'))['gnc_inliers'], 8)

    def test_reject_seed_without_atomic_consensus(self):
        log = 'KISS-Matcher enabled:\nGNC result: 6 LC inlier(s), 0 LC outlier(s)'
        with self.assertRaises(ValueError): closure_result(log, seed=True)
        self.assertEqual(closure_result(log+'\nManual LC hypothesis 0 accepted atomically', seed=True)['gnc_inliers'], 6)

    def test_reject_defined_georef_without_usable_fixes(self):
        g = dict(defined=True, geo_coord=dict(lat=38.4, lon=-78.8, altitude=400),
                 T_enu_to_map=dict(mean=dict(x=0, y=0, z=0, yaw_deg=0, pitch_deg=0, roll_deg=0)))
        stats = dict(usable_gnss=0, median_lat=38.4, median_lon=-78.8)
        with self.assertRaises(ValueError): validate_georef(g, 'Found: 0 GNSS frames', stats)
        stats['usable_gnss'] = 6
        validate_georef(g, 'Found: 6 GNSS frames', stats)
        bad = copy.deepcopy(g); bad['geo_coord']['lat'] = 0
        with self.assertRaises(ValueError): validate_georef(bad, 'Found: 6 GNSS frames', stats)

    def test_reject_native_loader_exception_or_incomplete_map(self):
        for text in ['Exception: unregistered class', 'georeferenced KeyframePointCloudMap: 0 keyframes',
                     'georeferenced KeyframePointCloudMap: 3 keyframes']:
            with self.assertRaises(ValueError): validate_metric_map(text, 6)
        validate_metric_map('georeferenced KeyframePointCloudMap: 6 keyframes', 6)

    def test_export_anchor_and_enu_direction(self):
        geo = dict(geo_coord=dict(lat=38.4, lon=-78.8))
        info = 'georeferenced (38.400000 -78.800000) h=400 T_enu_map=[0 0 0 0 0 0] KeyframePointCloudMap: 6 keyframes'
        validate_metric_map(info, 6, geo)
        for bad in [info.replace('38.400000', '0.000000'), info.replace('[0 0 0 0 0 0]', '[1 0 0 0 0 0]')]:
            with self.assertRaises(ValueError): validate_metric_map(bad, 6, geo)

    def test_coverage_and_stalled_tracking(self):
        with tempfile.TemporaryDirectory() as folder:
            trace = Path(folder)/'trace.csv'
            def write(q):
                trace.write_text('timestamp,icp_quality,robot_x,robot_y,robot_z\n' +
                                 ''.join(f'{1000+i},{q},0,0,0\n' for i in range(20)))
            write(.9)
            self.assertTrue(audit_reconstruction(trace, '100.00%', 20, 19)['passed'])
            self.assertFalse(audit_reconstruction(trace, '100.00%', 100, 99)['passed'])
            self.assertFalse(audit_reconstruction(trace, '99.00%', 20, 19)['passed'])
            write(.1)
            self.assertFalse(audit_reconstruction(trace, '100.00%', 20, 19)['passed'])


class WorkflowTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.root = Path(self.temp.name)
        self.bag = self.root/'bag'; self.bag.mkdir()
        (self.bag/'metadata.yaml').write_text(yaml.safe_dump(dict(rosbag2_bagfile_information=dict(
            topics_with_message_count=[dict(topic_metadata=dict(name='/velodyne_points'), message_count=20)],
            duration=dict(nanoseconds=19_000_000_000)))))
        self.sm = self.root/'input.simplemap'; self.sm.touch()

    def tearDown(self): self.temp.cleanup()

    def workflow(self, *args):
        return Workflow(parser().parse_args(['test', '--bag', str(self.bag), '--maps-dir', str(self.root/'maps'),
            '--simplemap-output', str(self.sm), '--skip-offline-map', '--skip-georef', '--skip-georef-mm',
            '--loop-start-keyframes', '0:5', '--loop-end-keyframes', '6:11', *args]))

    def test_preflight_refuses_input_alias_even_with_replacement_permission(self):
        w = self.workflow('--lc-simplemap-output', str(self.sm), '--allow-existing-output')
        with patch('run.shutil.which', return_value='/fake/tool'):
            with self.assertRaisesRegex(ValueError, 'aliases'): w.preflight()
        self.assertTrue(self.sm.exists())

    def test_existing_outputs_refused_before_any_commands(self):
        w = self.workflow(); w.lc.parent.mkdir(); w.lc.write_text('preserved')
        with patch('run.shutil.which', return_value='/fake/tool'):
            with self.assertRaisesRegex(ValueError, 'already exists'): w.preflight()
        self.assertEqual(w.lc.read_text(), 'preserved'); self.assertFalse(w.root.exists())

    def test_dry_run_does_not_create_output_directories(self):
        w = self.workflow('--dry-run')
        with patch('run.shutil.which', return_value='/fake/tool'), patch.object(w, 'dependencies'):
            w.execute()
        self.assertFalse(w.maps.exists())

    def test_skipped_stages_do_not_delete_retained_intermediates(self):
        w = self.workflow('--skip-loop-closure')
        w.endpoint.mkdir(parents=True); w.seed.write_text('preserved')
        w.narrow.mkdir(); w.unleveled.write_text('preserved')
        with patch('run.shutil.which', return_value='/fake/tool'), patch.object(w, 'dependencies'):
            w.execute()
        self.assertEqual(w.seed.read_text(), 'preserved'); self.assertEqual(w.unleveled.read_text(), 'preserved')


if __name__ == '__main__': unittest.main()
