"""Use a six-keyframe fixture and a synthetic GNSS/TF bag; never run odometry."""
import json
import os
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

import yaml

BUILD = Path(os.environ.get('DEV_WS', '/root/dev_ws'))/'build/post_processing_tools'


@unittest.skipUnless((BUILD/'post_processing_fixture').is_file(), 'Build native test fixture first')
class NativeTests(unittest.TestCase):
    def command(self, *args):
        p = subprocess.run(list(map(str, args)), capture_output=True, text=True)
        self.assertEqual(p.returncode, 0, p.stdout+p.stderr)
        return p.stdout

    def stats(self, path):
        text = self.command(BUILD/'simplemap_tools', 'inspect', path)
        return json.loads(re.findall(r'^POST_PROCESSING_JSON: (.+)$', text, re.M)[-1])

    def test_bag_fix_covariance_tf_and_roof_preservation(self):
        import rosbag2_py
        from rclpy.serialization import serialize_message
        from sensor_msgs.msg import NavSatFix
        from tf2_msgs.msg import TFMessage
        from geometry_msgs.msg import TransformStamped
        with tempfile.TemporaryDirectory(prefix='e08-native-') as folder:
            root = Path(folder); bag = root/'bag'
            writer = rosbag2_py.SequentialWriter()
            writer.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'),
                        rosbag2_py.ConverterOptions('', ''))
            for i, name, kind in [(1, '/tf_static', 'tf2_msgs/msg/TFMessage'), (2, '/fix', 'sensor_msgs/msg/NavSatFix')]:
                writer.create_topic(rosbag2_py.TopicMetadata(id=i, name=name, type=kind, serialization_format='cdr'))
            tf = TransformStamped(); tf.header.frame_id='base_link'; tf.child_frame_id='gps'
            tf.transform.rotation.w=1.; tf.transform.translation.z=.2
            writer.write('/tf_static', serialize_message(TFMessage(transforms=[tf])), 999_000_000_000)
            for stamp in (1000, 1002, 1004):
                fix = NavSatFix(); fix.header.frame_id='gps'; fix.header.stamp.sec=stamp
                fix.status.status=0; fix.status.service=1
                fix.latitude=38.4; fix.longitude=-78.8; fix.altitude=400.
                fix.position_covariance=[.25,0.,0.,0.,.25,0.,0.,0.,1.]
                fix.position_covariance_type=3
                writer.write('/fix', serialize_message(fix), stamp*1_000_000_000)
            del writer
            cfg = root/'dataset.yaml'
            cfg.write_text(yaml.safe_dump(dict(params=dict(rosbag_filename=str(bag), base_link_frame_id='base_link',
                sensors=[dict(topic='/fix', sensorLabel='gps', is_optional=True)]))))
            raw, attached, roof = root/'raw.simplemap', root/'attached.simplemap', root/'roof.simplemap'
            self.command(BUILD/'post_processing_fixture', 'create', raw)
            self.command(BUILD/'simplemap_tools', 'attach-bag', cfg, raw, attached, '1.0')
            self.command(BUILD/'post_processing_fixture', 'check-gps', attached)
            self.assertEqual(self.stats(raw)['gnss_observations'], 0)
            before = self.stats(attached); self.assertEqual(before['usable_gnss'], 6)
            self.command(BUILD/'filter_cart_roof', '--self-test')
            log = self.command(BUILD/'filter_cart_roof', attached, roof)
            self.assertIn('removed=6', log)
            after = self.stats(roof)
            self.assertEqual(before['gnss_digest'], after['gnss_digest'])
            self.assertEqual(before['keyframes'], after['keyframes'])
            # The one-second timestamp gate must reject fixes outside its range.
            gated = root/'gated.simplemap'
            self.command(BUILD/'simplemap_tools', 'attach-bag', cfg, raw, gated, '.1')
            self.assertEqual(self.stats(gated)['gnss_observations'], 3)


if __name__ == '__main__': unittest.main()
