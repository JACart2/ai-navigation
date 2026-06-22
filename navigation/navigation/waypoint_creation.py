import math
import ctypes
import os
import subprocess
import re
import signal
import sys
import termios
import threading
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PointStamped, Point, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import networkx as nx


PR_SET_PDEATHSIG = 1


class WaypointCreation(Node):
    def __init__(self, selected_mode=None):
        super().__init__('waypoint_creation')
        
        # Declare parameters
        self.declare_parameter('new_map_mode', True)
        self.declare_parameter('input_gml_file', '/root/dev_ws/src/ai-navigation/navigation/maps/main_shift3.gml')
        self.declare_parameter('output_gml_file', '/root/dev_ws/src/ai-navigation/navigation/maps/main_shift6.gml')
        
        self.rviz_config_file = '/root/dev_ws/src/ai-navigation/navigation/test/waypoint_creation.rviz'
        self.new_map_mode = self.get_parameter('new_map_mode').value
        self.input_gml_file = self.get_parameter('input_gml_file').value
        self.output_gml_file = self.get_parameter('output_gml_file').value

        if selected_mode is not None:
            self.new_map_mode = selected_mode

        if not self.new_map_mode:
            self.output_gml_file = self.input_gml_file

        self.recent_pos = None
        self.point_mode = "Add"
        self.auto_connect = self.new_map_mode
        self.first_selection = None
        self.second_selection = None
        self.selected_node = None
        self.global_graph = nx.DiGraph()
        self.node_count = 0
        self.last_node = None
        self.input_thread = None
        self.has_unsaved_changes = False

        # How many meters between points when auto-build map is toggled on
        self.AUTO_POINT_GAP = 2
        self.MOVE_STEP = 1.0
        
        # Ensure output directory exists
        if self.output_gml_file:
            output_dir = os.path.dirname(self.output_gml_file)
            if output_dir and not os.path.exists(output_dir):
                os.makedirs(output_dir, exist_ok=True)
                self.get_logger().info(f'Created output directory: {output_dir}')
        
        # Store waypoints separately for RViz coloring
        self.existing_waypoints = []
        self.new_waypoints = []
        
        # Load existing waypoints from GML file if specified
        if not self.new_map_mode and self.input_gml_file:
            self.load_waypoints_from_gml()

        if not self.new_map_mode and not self.existing_waypoints:
            error_message = f'Cannot edit route because input GML file was not found or contained no waypoints: {self.input_gml_file}'
            self.get_logger().error(error_message)
            raise FileNotFoundError(error_message)
        
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

        self.clicked_point_sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_point_callback,
            10
        )
        
        marker_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Publisher for RViz markers
        self.marker_pub = self.create_publisher(
            MarkerArray,
            'waypoint_markers',
            marker_qos
        )

        self.new_point_pub = self.create_publisher(
            PointStamped,
            'new_point',
            10
        )
        
        self.get_logger().info('Waypoint Creation Node initialized')

        if self.existing_waypoints or self.new_waypoints:
            self.publish_markers()

        if not self.new_map_mode:
            self.start_edit_controls()

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
            self.global_graph = nx.read_gml(self.input_gml_file)
            self.refresh_waypoint_lists()
            self.node_count = self.get_max_node_index()
            self.last_node = self.get_last_node_name()
            self.get_logger().info(f'Loaded {len(self.existing_waypoints)} existing waypoints from {self.input_gml_file}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints from GML file: {e}')

    def save_waypoints_to_gml(self):
        if not self.output_gml_file:
            return
        
        if self.global_graph.number_of_nodes() == 0:
            return
        
        try:
            nx.write_gml(self.global_graph, self.output_gml_file)
            self.has_unsaved_changes = False
            self.get_logger().info(f'Saved {self.global_graph.number_of_nodes()} waypoints to {self.output_gml_file}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to save waypoints to GML file: {e}')

    def new_point_callback(self, msg):
        self.add_point(msg.point.x, msg.point.y)

    def clicked_point_callback(self, msg):
        if self.new_map_mode:
            return

        if self.point_mode == "Add":
            self.add_point(msg.point.x, msg.point.y)
        elif self.point_mode == "Remove":
            self.remove_point(msg.point.x, msg.point.y)
        elif self.point_mode == "Connect":
            self.connect_point(msg.point.x, msg.point.y)
        elif self.point_mode == "Select":
            self.select_point(msg.point.x, msg.point.y)

    def refresh_waypoint_lists(self):
        self.existing_waypoints = []
        self.new_waypoints = []

        for _, data in self.global_graph.nodes(data=True):
            pos = data["pos"]
            point = Point()
            point.x = pos[0]
            point.y = pos[1]
            point.z = 0.0

            if data.get("is_new", False):
                self.new_waypoints.append(point)
            else:
                self.existing_waypoints.append(point)

    def get_max_node_index(self):
        max_index = 0
        for node_name in self.global_graph.nodes:
            match = re.search(r'(\d+)$', str(node_name))
            if match:
                max_index = max(max_index, int(match.group(1)))
        return max_index

    def get_last_node_name(self):
        best_name = None
        best_index = -1
        for node_name in self.global_graph.nodes:
            match = re.search(r'(\d+)$', str(node_name))
            if match and int(match.group(1)) > best_index:
                best_index = int(match.group(1))
                best_name = node_name
        return best_name

    def add_point(self, x, y):
        self.node_count += 1
        node_name = f'Waypoint:{self.node_count}'
        self.global_graph.add_node(node_name, pos=[x, y], active=False, is_new=True)

        if self.auto_connect and self.last_node is not None and self.last_node in self.global_graph:
            self.add_weighted_edge(self.last_node, node_name)

        self.last_node = node_name
        self.refresh_waypoint_lists()
        if self.new_map_mode:
            self.save_waypoints_to_gml()
        else:
            self.has_unsaved_changes = True
        self.publish_markers()
        self.get_logger().info(f'Added waypoint: ({x}, {y})')

    def remove_point(self, x, y):
        closest_node = self.get_closest_node(x, y)
        if closest_node is None:
            return

        if self.selected_node == closest_node:
            self.selected_node = None

        self.global_graph.remove_node(closest_node)
        self.last_node = self.get_last_node_name()
        self.refresh_waypoint_lists()
        self.has_unsaved_changes = True
        self.publish_markers()
        self.get_logger().info(f'Removed waypoint: {closest_node}')

    def connect_point(self, x, y):
        if self.first_selection is None:
            self.first_selection = self.get_closest_node(x, y)
            if self.first_selection is not None:
                self.get_logger().info(f'Selected first waypoint: {self.first_selection}')
            return

        self.second_selection = self.get_closest_node(x, y)
        if self.second_selection is None:
            return

        self.add_weighted_edge(self.first_selection, self.second_selection)
        self.has_unsaved_changes = True
        self.publish_markers()
        self.get_logger().info(f'Connected {self.first_selection} -> {self.second_selection}')
        self.first_selection = None
        self.second_selection = None

    def select_point(self, x, y):
        self.selected_node = self.get_closest_node(x, y)
        if self.selected_node is not None:
            self.get_logger().info(f'Selected waypoint: {self.selected_node}')
            self.publish_markers()

    def add_weighted_edge(self, first_node, second_node):
        x1, y1, x2, y2 = self.get_node_coordinates(first_node, second_node)
        cost = self.dis(x1, y1, x2, y2)
        self.global_graph.add_edge(first_node, second_node, weight=cost)

    def get_node_coordinates(self, first_node, second_node):
        first_pos = self.global_graph.nodes[first_node]['pos']
        second_pos = self.global_graph.nodes[second_node]['pos']
        return first_pos[0], first_pos[1], second_pos[0], second_pos[1]

    def get_closest_node(self, x, y):
        min_dist = None
        min_node = None
        for node_name, data in self.global_graph.nodes(data=True):
            node_x = data['pos'][0]
            node_y = data['pos'][1]
            dist = self.dis(node_x, node_y, x, y)
            if min_dist is None or dist < min_dist:
                min_dist = dist
                min_node = node_name
        return min_node

    def get_all_waypoints(self):
        return self.existing_waypoints + self.new_waypoints

    def move_selected_node(self, dx, dy):
        if self.selected_node is None or self.selected_node not in self.global_graph:
            self.get_logger().info('No waypoint selected. Use select mode and click a waypoint in RViz first.')
            return

        node_pos = self.global_graph.nodes[self.selected_node]['pos']
        node_pos[0] += dx
        node_pos[1] += dy

        for first_node, second_node in list(self.global_graph.edges(self.selected_node)):
            self.add_weighted_edge(first_node, second_node)
        for first_node, second_node in list(self.global_graph.in_edges(self.selected_node)):
            self.add_weighted_edge(first_node, second_node)

        self.refresh_waypoint_lists()
        self.has_unsaved_changes = True
        self.publish_markers()
        self.get_logger().info(
            f'Moved {self.selected_node} to ({node_pos[0]:.2f}, {node_pos[1]:.2f})'
        )

    def publish_markers(self):
        marker_array = MarkerArray()
        marker_id = 0
        
        for first_node, second_node in self.global_graph.edges:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "route_edges"
            marker.id = marker_id
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.scale.x = 0.3
            marker.scale.y = 0.6
            marker.scale.z = 0.0
            marker.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)
            marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()

            x1, y1, x2, y2 = self.get_node_coordinates(first_node, second_node)
            first_point = Point()
            first_point.x = x1
            first_point.y = y1

            second_point = Point()
            second_point.x = x2
            second_point.y = y2

            marker.points = [first_point, second_point]
            marker_array.markers.append(marker)
            marker_id += 1

        for point in self.existing_waypoints:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "existing_waypoints"
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = point.x
            marker.pose.position.y = point.y
            marker.pose.position.z = point.z
            marker.pose.orientation.w = 1.0
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 0.2
            marker.color = ColorRGBA(r=0.0, g=0.4, b=1.0, a=1.0)
            marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
            marker_array.markers.append(marker)
            marker_id += 1

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

        if self.selected_node is not None and self.selected_node in self.global_graph:
            selected_pos = self.global_graph.nodes[self.selected_node]['pos']
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "selected_waypoint"
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = selected_pos[0]
            marker.pose.position.y = selected_pos[1]
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 1.4
            marker.scale.y = 1.4
            marker.scale.z = 0.3
            marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8)
            marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
            marker_array.markers.append(marker)
            marker_id += 1
        
        self.marker_pub.publish(marker_array)
        total_waypoints = len(self.existing_waypoints) + len(self.new_waypoints)
        self.get_logger().info(f'Published {total_waypoints} waypoints to RViz ({len(self.existing_waypoints)} existing, {len(self.new_waypoints)} new)')

    def start_edit_controls(self):
        self.print_edit_controls()
        self.input_thread = threading.Thread(target=self.edit_controls_loop, daemon=True)
        self.input_thread.start()

    def print_edit_controls(self):
        print("\nEdit route controls:")
        print("  a = add waypoint using RViz Publish Point")
        print("  r = remove nearest clicked waypoint")
        print("  c = connect two clicked waypoints")
        print("  v = select waypoint, then use move commands")
        print("  up/down/left/right = move selected waypoint")
        print("  w = toggle auto-connect when adding")
        print("  s = save graph now")
        print("  h = show this help")
        print("  q = quit without saving unsaved changes\n")

    def read_select_command(self):
        prompt = "select-move> "
        print(prompt, end="", flush=True)

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        try:
            tty.setraw(fd)
            first_char = sys.stdin.read(1)
            if first_char == '\x1b':
                second_char = sys.stdin.read(1)
                third_char = sys.stdin.read(1)
                if second_char == '[':
                    if third_char == 'A':
                        print("up")
                        return 'up'
                    if third_char == 'B':
                        print("down")
                        return 'down'
                    if third_char == 'C':
                        print("right")
                        return 'right'
                    if third_char == 'D':
                        print("left")
                        return 'left'
                print()
                return ''

            print(first_char)
            return first_char.lower()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def edit_controls_loop(self):
        while rclpy.ok():
            try:
                if self.point_mode == "Select":
                    command = self.read_select_command()
                else:
                    command = input("edit-route> ").strip().lower()
            except EOFError:
                return

            if command == 'a':
                self.point_mode = "Add"
                print("Point mode set to Add")
            elif command == 'r':
                self.point_mode = "Remove"
                self.first_selection = None
                self.second_selection = None
                print("Point mode set to Remove")
            elif command == 'c':
                self.point_mode = "Connect"
                self.first_selection = None
                self.second_selection = None
                print("Point mode set to Connect")
            elif command == 'v':
                self.point_mode = "Select"
                self.first_selection = None
                self.second_selection = None
                print("Point mode set to Select. Click a waypoint in RViz, then use arrow keys.")
            elif command == 'up':
                self.move_selected_node(0.0, -self.MOVE_STEP)
            elif command == 'down':
                self.move_selected_node(0.0, self.MOVE_STEP)
            elif command == 'right':
                self.move_selected_node(-self.MOVE_STEP, 0.0)
            elif command == 'left':
                self.move_selected_node(self.MOVE_STEP, 0.0)
            elif command == 'w':
                self.auto_connect = not self.auto_connect
                print(f"Auto-connect set to {self.auto_connect}")
            elif command == 's':
                self.save_waypoints_to_gml()
                print(f"Saved {self.output_gml_file}")
            elif command == 'h':
                self.print_edit_controls()
            elif command == 'q':
                if self.has_unsaved_changes:
                    print("Discarded unsaved changes")
                rclpy.shutdown()
                return

    def launch_rviz(self):
        if not os.path.exists(self.rviz_config_file):
            self.get_logger().warn(f'RViz config file does not exist: {self.rviz_config_file}')
            return

        try:
            def set_parent_death_signal():
                libc = ctypes.CDLL("libc.so.6")
                libc.prctl(PR_SET_PDEATHSIG, signal.SIGTERM)

            subprocess.Popen(
                ['rviz2', '-d', self.rviz_config_file],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=set_parent_death_signal,
            )
            self.get_logger().info(f'Launched RViz with config: {self.rviz_config_file}')
        except FileNotFoundError:
            self.get_logger().warn('rviz2 executable not found; unable to launch RViz automatically.')


def prompt_for_route_mode():
    prompt = (
        "\nSelect waypoint mode:\n"
        "1. Create new route\n"
        "2. Edit existing route\n"
        "Enter choice [1/2]: "
    )

    while True:
        choice = input(prompt).strip()
        if choice == '1':
            return True
        if choice == '2':
            return False
        print("Invalid choice. Please enter 1 or 2.")




def main(args=None):
    selected_mode = prompt_for_route_mode()
    rclpy.init(args=args)
    try:
        waypoint_creation_node = WaypointCreation(selected_mode=selected_mode)
    except FileNotFoundError:
        rclpy.shutdown()
        return
    if not selected_mode:
        waypoint_creation_node.launch_rviz()
    rclpy.spin(waypoint_creation_node)
    waypoint_creation_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
