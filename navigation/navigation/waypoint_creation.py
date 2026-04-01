import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Point, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import networkx as nx
import os
from pathlib import Path


class WaypointCreation(Node):
    def __init__(self):
        super().__init__('waypoint_creation')
        
        # Declare parameters
        self.declare_parameter('new_map_mode', True)
        self.declare_parameter('input_gml_file', './src/ai-navigation/navigation/maps/main_shift3.gml')
        self.declare_parameter('output_gml_file', './navigation/maps/main_shift4.gml')
        
        self.new_map_mode = self.get_parameter('new_map_mode').value
        self.input_gml_file = self.get_parameter('input_gml_file').value
        self.output_gml_file = self.get_parameter('output_gml_file').value

        self.recent_pos = None

        # How many meters between points when auto-build map is toggled on
        self.AUTO_POINT_GAP = 2
        
        # Ensure output directory exists
        if self.output_gml_file:
            output_dir = os.path.dirname(self.output_gml_file)
            if output_dir and not os.path.exists(output_dir):
                os.makedirs(output_dir, exist_ok=True)
                self.get_logger().info(f'Created output directory: {output_dir}')
        
        # Store waypoints separately
        self.existing_waypoints = []
        self.new_waypoints = []
        
        # Load existing waypoints from GML file if specified
        if self.input_gml_file:
            self.load_waypoints_from_gml()
        
        # Subscription to new_point topic
        self.subscription = self.create_subscription(
            PointStamped,
            'new_point',
            self.new_point_callback,
            10
        )

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/pcl_pose", self.pose_callback, 10
        )
        
        # Publisher for RViz markers
        self.marker_pub = self.create_publisher(
            MarkerArray,
            'waypoint_markers',
            10
        )

        self.new_point_pub = self.create_publisher(
            PointStamped,
            'new_point',
            10
        )
        
        self.get_logger().info('Waypoint Creation Node initialized')
        
        # Publish initial markers if any exist
        if self.new_map_mode and (self.existing_waypoints or self.new_waypoints):
            self.publish_markers()

    def pose_callback(self, msg):
            current_pos = PointStamped()
            
            current_pos.point.x = msg.pose.pose.position.x
            current_pos.point.y = msg.pose.pose.position.y
        
            if self.recent_pos is None:
                self.recent_pos = current_pos
        
            # Place a new point down every specified amount of meters
            if self.dis(self.recent_pos.point.x, self.recent_pos.point.y, current_pos.point.x, current_pos.point.y) > self.AUTO_POINT_GAP:
                self.recent_pos = current_pos
                self.new_point_pub.publish(current_pos)

    def dis(self, x1, y1, x2, y2):
        return math.sqrt((x2-x1)**2 + (y2-y1)**2)

    def load_waypoints_from_gml(self):
        if not os.path.exists(self.input_gml_file):
            self.get_logger().warn(f'Input GML file does not exist: {self.input_gml_file}')
            return
        
        try:
            graph = nx.read_gml(self.input_gml_file)
            for node in graph:
                pos = graph.nodes[node]["pos"]
                point = Point()
                point.x = pos[0]
                point.y = pos[1]
                point.z = 0.0
                self.existing_waypoints.append(point)
            
            self.get_logger().info(f'Loaded {len(self.existing_waypoints)} existing waypoints from {self.input_gml_file}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints from GML file: {e}')

    def save_waypoints_to_gml(self):
        if not self.output_gml_file:
            return
        
        if self.new_map_mode is False:
            waypoints_to_save = self.existing_waypoints + self.new_waypoints
        else:
            waypoints_to_save = self.new_waypoints
        
        if not waypoints_to_save:
            return
        
        try:
            with open(self.output_gml_file, 'w') as f:
                f.write('graph [\n')
                f.write('  directed 1\n')
                for i, point in enumerate(waypoints_to_save):
                    f.write('  node [\n')
                    f.write(f'    id {i}\n')
                    f.write(f'    label "Waypoint:{i}"\n')
                    f.write('    active 0\n')
                    f.write(f'      pos {point.x}\n')
                    f.write(f'      pos {point.y}\n')
                    f.write('  ]\n')
                f.write(']\n')
            
            self.get_logger().info(f'Saved {len(waypoints_to_save)} waypoints to {self.output_gml_file}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to save waypoints to GML file: {e}')

    def new_point_callback(self, msg):
        # Store the new point
        self.new_waypoints.append(msg.point)
        self.get_logger().info(f'Added new waypoint: ({msg.point.x}, {msg.point.y}, {msg.point.z})')
        
        # Save waypoints to file
        self.save_waypoints_to_gml()
        
        # If new map mode is enabled, publish markers to RViz
        if self.new_map_mode:
            self.publish_markers()

    def publish_markers(self):
        marker_array = MarkerArray()
        marker_id = 0
        
        # Publish new waypoints in green
        for point in self.new_waypoints:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "new_waypoints"
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = point.x
            marker.pose.position.y = point.y
            marker.pose.position.z = point.z
            marker.pose.orientation.w = 1.0
            marker.scale.x = 1.0  # change
            marker.scale.y = 1.0
            marker.scale.z = 0.2
            marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # Green spheres for new
            marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
            marker_array.markers.append(marker)
            marker_id += 1
        
        self.marker_pub.publish(marker_array)
        total_waypoints = len(self.existing_waypoints) + len(self.new_waypoints)
        self.get_logger().info(f'Published {total_waypoints} waypoints to RViz ({len(self.existing_waypoints)} existing, {len(self.new_waypoints)} new)')




def main(args=None):
    rclpy.init(args=args)
    waypoint_creation_node = WaypointCreation()
    rclpy.spin(waypoint_creation_node)
    waypoint_creation_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()