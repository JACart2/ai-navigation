#!/usr/bin/env python
"""
This is the ROS 2 node that handles the global planning for the JACART.

Authors: Zane Metz, Lorenzo Ashurst, Zach Putz
Modified by Joshua Sun in Spring 2026.

"""
import sys
print(sys.path)
# Python based imports
import math
import os
import networkx as nx
import navigation.simple_gps_util as simple_gps_util
from ament_index_python.packages import get_package_share_directory

# ROS based import
import rclpy.node
import tf_transformations
import tf2_geometry_msgs  #  Import is needed, even though not used explicitly


import rclpy
from geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    PointStamped,
    PoseWithCovarianceStamped,
)
from visualization_msgs.msg import Marker
from std_msgs.msg import String, Header, Float32
from navigation_interface.msg import (
    VehicleState,
    LatLongPoint,
    LocalPointsArray,
    LatLongArray,
)


class GlobalPlanner(rclpy.node.Node):
    """ROS2 node that handles controlling the motor."""

    def __init__(self):
        super().__init__("global_planner")

        # Current waypoint of cart
        self.current_pos = None
        self.current_cart_node = None
        self.prev_cart_node = None
        self.destination_node = None
        self.yaw = None
        self.navigating = False
        self.calculating_nav = False
        self.orientation = (0.0, 0.0, 0.0)

        self.cur_speed = 0.0

        self.vel_polls = 0

        self.gps_calibrated = False

        self.minimize_travel = True  # TODO - Consider removing or setting another way


        # GPS calibration state using landmark-based calibration
        self.ref_lat = 0.0
        self.ref_lon = 0.0
        self.cx_local = 0.0
        self.cy_local = 0.0
        self.cx_gps = 0.0
        self.cy_gps = 0.0
        self.calibration_theta = 0.0

        default_config_dir = os.path.join(
            get_package_share_directory("navigation"),
            "maps",
        )
        self.declare_parameter(
            "calibration_config_dir",
            default_config_dir,
        )
        self.declare_parameter(
            "calibration_config_file",
            "main_graph_config.yaml",
        )
        calibration_config_dir = self.get_parameter(
            "calibration_config_dir"
        ).get_parameter_value().string_value
        calibration_config_file = self.get_parameter(
            "calibration_config_file"
        ).get_parameter_value().string_value
        calibration_config_path = simple_gps_util.resolve_config_path(
            calibration_config_dir,
            calibration_config_file,
        )
        self.test_location_local, self.test_location_gps = (
            simple_gps_util.load_landmark_calibration(calibration_config_path)
        )

        # Run GPS calibration at startup using landmark-based method
        self.ref_lat, self.ref_lon, self.cx_local, self.cy_local, self.cx_gps, self.cy_gps, self.calibration_theta = simple_gps_util.calibrate_with_landmarks(
            self.test_location_local,
            self.test_location_gps,
        )
        self.gps_calibrated = True
        self.get_logger().info(
            f"GPS calibrated at startup using {len(self.test_location_gps)} landmark point(s), "
            f"rotation: {self.calibration_theta:.4f} degrees"
        )
        self.get_logger().info(
            f"Loaded calibration landmarks from {calibration_config_path}"
        )

        # Graphing variables
        self.global_graph = nx.DiGraph()
        self.logic_graph = None

        self.declare_parameter(
            "graph_file", "./src/ai-navigation/navigation/maps/main_shift3.gml"
        )
        self.declare_parameter("graph_coordinate_format", "ros")
        file_name = self.get_parameter("graph_file").get_parameter_value().string_value
        graph_coordinate_format = self.get_parameter(
            "graph_coordinate_format"
        ).get_parameter_value().string_value
        self.load_file(file_name, graph_coordinate_format)

        # ROS2 SUBSCRIBERS
        # ------------------------------------------

        # The library we are working with is giving us a PoseWithCovarianceStamped.
        # This is the pose we are getting back from localization.
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/pcl_pose", self.pose_callback, 10
        )

        # Listen for vehicle state changes
        self.vehicle_state_sub = self.create_subscription(
            VehicleState, "/vehicle_state", self.state_change, 10
        )

        # Clicked destination requests, accepted from RViz clicked points, disabled when building maps
        self.click_req_sub = self.create_subscription(
            PointStamped, "/clicked_point", self.point_callback, 10
        )

        # Listen for velocity/speed of cart
        self.vel_sub = self.create_subscription(
            Float32, "/estimated_vel_mps", self.vel_callback, 10
        )

        # Listen for GPS destination requests
        self.lat_long_req = self.create_subscription(
            LatLongPoint, "/gps_request", self.gps_request_cb, 10
        )

        # Listen for standard destination requests
        # self.dest_req_sub = rospy.Subscriber('/destination_request', String, self.request_callback, queue_size=10)

        # ROS 2 PUBLISHERS
        # ------------------------------------------

        # Publish the path for local planner to begin navigating
        self.path_pub = self.create_publisher(LocalPointsArray, "/global_path", 10)

        # GPS publishers and position update timer
        self.gps_path_pub = self.create_publisher(LatLongArray, "/gps_global_path", 10)
        self.display_pub = self.create_publisher(Marker, "/display_gps", 10)
        # Publishes current cart position in GPS coordinates
        self.gps_pose_pub = self.create_publisher(LatLongPoint, "/gps_send", 10)
        # Publish cart GPS position at 10 Hz while navigating
        self.gps_timer = self.create_timer(0.1, self.output_pos_gps)

    # Load the graph file as the global graph
    def load_file(self, file_name, graph_coordinate_format="ros"):
        """Loads a file and sets all nodes to active(allowed to take part in pathfinding).

        Args:
            file_name (string): The .gml file to load as the graph, cart_planning/launch/constants.launch
            is where the file path can be found
        """

        try:
            self.global_graph = nx.read_gml(file_name)
            self.global_graph = simple_gps_util.convert_graph_to_local(
                self.global_graph,
                graph_coordinate_format,
                self.ref_lat,
                self.ref_lon,
                self.cx_local,
                self.cy_local,
                self.cx_gps,
                self.cy_gps,
                self.calibration_theta,
            )

            for node in self.global_graph:
                self.global_graph.nodes[node]["active"] = True
        except Exception as e:
            self.log_header(
                f"Unable to launch graph file pointed to in the constants file in {file_name} because {e}"
            )

    def calc_nav(self, destination):
        """Main navigation calculation function, main job is to calculate the path. As of right now this only
        gets called in point_callback which deals with
        local cordinates. When GPS is operational there are GPS methods that can call calc_nav

        Args:
            point (ROS Point Message): Point representing where the cart should navigate to
        """

        # Allows other functions to not make critical decisions or modify data while calculating navigation
        self.calculating_nav = True
        self.total_distance = 0

        # Make a destination request when we don't know where the cart is at yet
        if self.current_pos is None:
            self.current_pos = PoseStamped()
            self.current_pos.pose.position.x = 0.0
            self.current_pos.pose.position.y = 0.0

        current_cart_pos = self.current_pos.pose.position

        # When the cart_trans param is set to true it attemps to find a node closest to the us based on the path we are on.
        # This is useful for when we are already navigating...
        self.current_cart_node = self.get_closest_node(
            current_cart_pos.x, current_cart_pos.y, cart_trans=True
        )

        # Get the node closest to where we want to go
        # It would be worth investigating whether adding "cart_trans=True" here would be useful.
        destination_point = self.get_closest_node(destination.x, destination.y)
        self.destination_node = destination_point

        self.current_cart_node = self.determine_lane(self.current_cart_node)

        # Maybe the cart was manually moved and the lane is no longer near the original destination
        if self.current_cart_node is None:
            self.log_header(
                "Problem obtaining position of the cart, maybe you moved the cart manually? Refreshing graph status and retrying..."
            )

            # Attempt to find a node that was not a part of the original path that is appropriate to the cart
            self.reset_node_activity()
            self.current_cart_node = self.determine_lane(None)

            # If the cart still can not find a reliable node/lane for the cart to be in give up
            if self.current_cart_node is None:
                self.log_header(
                    "Still an issue finding an appropriate position in the graph"
                )
                self.log_header(
                    "Make sure you are within reliable distance of the road network supported by the graph(~10 meters)"
                )
                return None
            else:
                self.log_header(
                    "A suitable node was found, please check RViz to make sure the pathing is safe"
                )

        # Attempt to find a route to the destination
        try:
            nodelist = None
            if self.minimize_travel:
                nodelist = self.calc_efficient_destination(destination_point)
            else:
                nodelist = nx.dijkstra_path(
                    self.global_graph, self.current_cart_node, destination_point
                )

            # Set all nodes to a state of not being a part of the coming path
            for node in self.global_graph:
                self.global_graph.nodes[node]["active"] = False

            # Convert our list of nodes along the path to the destination to a list of Point messages
            points_arr = LocalPointsArray()

            # Begin conversion and set nodes belonging to the path to active
            for node in nodelist:
                self.global_graph.nodes[node]["active"] = True
                new_point = Pose()
                new_point.position.x = self.global_graph.nodes[node]["pos"][0]
                new_point.position.y = self.global_graph.nodes[node]["pos"][1]
                points_arr.localpoints.append(new_point)

            # Allows for class changes again
            self.calculating_nav = False

            # Convert the path to GPS coordinates and publish for networking
            if self.gps_calibrated:
                self.output_path_gps(points_arr)

            self.path_pub.publish(points_arr)
            self.get_logger().info(
                f"Publishing Path: {str(self.current_cart_node)} to {str(destination_point)}"
            )

        except nx.NetworkXNoPath:
            self.log_header("Unable to find a path to the desired destination")
            self.log_header(
                "Debug info: Starting Node: "
                + str(self.current_cart_node)
                + " End node: "
                + str(destination_point)
            )
            if not nx.has_path(
                self.global_graph, self.current_cart_node, destination_point
            ):
                self.log_header("NetworkX can't find a connection")
            else:
                self.log_header("NetworkX can find a connection")

    def determine_lane(self, cart_node):
        """A function for determining which lane the cart is in, or should be in. (Note lanes being directions in the directed graph)

        Args:
            cart_node (NetworkX Node/Hashable object): The node the cart is at or closest to

        Returns:
            The closest node of the appropriate lane
        """
        if cart_node is not None:
            # Obtain the nodes within a 10 meter radius
            closest_nodes = self.get_nodes_in_radius(cart_node, 10)
        else:
            # A reduced radius, assuming this is used for second attempt, need to be safer here
            closest_nodes = self.get_nodes_in_radius(None, 5)

        if len(closest_nodes) < 1:
            return None

        cart_pos = None

        # Dictionary of nodes and the angle difference between the cart and the node
        node_angles = {}
        for node in closest_nodes:
            # If the node is to far away from the actual cart(10 meters) somehow, disregard it
            node_pos = self.global_graph.nodes[node]["pos"]
            cart_pos = (
                self.current_pos.pose.position.x,
                self.current_pos.pose.position.y,
            )
            distance = self.dis(node_pos[0], node_pos[1], cart_pos[0], cart_pos[1])
            if distance > 10:
                continue
            # A node is active if and only if the cart traverses or is supposed to traverse the node at some point during its trip
            if self.global_graph.nodes[node]["active"]:
                for u, v in self.global_graph.out_edges(node):
                    # Calculate the angle of our node to its outgoing neighbor
                    dy = (
                        self.global_graph.nodes[v]["pos"][1]
                        - self.global_graph.nodes[u]["pos"][1]
                    )
                    dx = (
                        self.global_graph.nodes[v]["pos"][0]
                        - self.global_graph.nodes[u]["pos"][0]
                    )
                    node_angle = math.atan2(dy, dx)

                    # Obtain the yaw of the cart
                    cart_angle = self.orientation[2]

                    # If node qualifies as being correct in the lane(not requiring a turn of more than half pi radians) assign a distance to it
                    if abs(cart_angle - node_angle) < math.pi / 2:
                        node_pos = self.global_graph.nodes[node]["pos"]

                        # If there is a close valid node to the cart, use it for distance calculations
                        if cart_node is not None:
                            cart_pos = self.global_graph.nodes[cart_node]["pos"]
                        node_angles[node] = self.dis(
                            node_pos[0], node_pos[1], cart_pos[0], cart_pos[1]
                        )

        if len(node_angles) < 1:
            return None

        # Finally return the closest node of the proper lane
        return min(node_angles, key=node_angles.get)

    def reset_node_activity(self):
        """Resets all nodes active status to True this makes them all game for the cart to determine which is closest/most appropriate"""
        for node in self.global_graph.nodes:
            self.global_graph.nodes[node]["active"] = True

    def get_nodes_in_radius(self, center_node, radius=3):
        """Obtains nodes in a given radius from a node representing center.

        Args:
            center_node (NetworkX Node/Hashable Object): The center node for determining where to search out from
            radius (int): The radius (in meters to search) to search from the center

        Returns:
            A list of nodes that are in distance
        """
        close_nodes = []
        center_node_pos = None

        # If there is a center node to go by find in its radius neighbors
        if center_node is not None:
            center_node_pos = self.global_graph.nodes[center_node]["pos"]
        else:
            # If there is no known or at least reliable center node use the carts current position
            center_node_pos = (
                self.current_pos.pose.position.x,
                self.current_pos.pose.position.y,
            )

        # Loop through each node determine if in proximity
        for node in self.global_graph.nodes:
            node_pos = self.global_graph.nodes[node]["pos"]

            # Distance between this node in iteration and center node or cart position
            dist = self.dis(
                node_pos[0], node_pos[1], center_node_pos[0], center_node_pos[1]
            )

            if dist < radius:
                close_nodes.append(node)

        return close_nodes

    def calc_efficient_destination(self, destination):
        """Attempts to find more efficient pathfinding to the destination over the default path.
        This is useful for not having the cart overshoot the passenger if the passenger is on the wrong side of the road for pickup.
        Preventing the cart from having to go far and doing a U-Turn to get to the side the passenger is on.

        Args:
            destination (NetworkX Node/Hashable Object): The destination node for pickup/driving to

        Returns:
            Path list of nodes to the destination
        """
        # Find nodes within 3 meters of destination node
        close_nodes = [destination]
        local_logic_graph = self.global_graph
        dest_node_pos = local_logic_graph.nodes[destination]["pos"]

        # Begin determining which nodes are close and eliminate any nodes that are more efficient just because they are next to the destination node
        for node in self.global_graph.nodes:
            inefficient = True
            node_pos = local_logic_graph.nodes[node]["pos"]

            # Is node close enough and also not incident to the destination
            dist = self.dis(
                node_pos[0], node_pos[1], dest_node_pos[0], dest_node_pos[1]
            )
            if dist < 5 and node is not destination:
                for u, v in local_logic_graph.out_edges(node):
                    if u is destination or v is destination:
                        inefficient = True
                        break
                    else:
                        inefficient = False

            # if node is not an inefficient destination add it
            if not inefficient:
                close_nodes.append(node)

        min_path = None
        # Out of the most efficient paths, which one has the least driving distance
        for node in close_nodes:
            node_path = nx.dijkstra_path(
                local_logic_graph, self.current_cart_node, node
            )
            if min_path is None:
                min_path = node_path

            # TODO replace with cost analysis
            if len(node_path) < len(min_path):
                min_path = node_path

        return min_path

    # Closest node to given point
    def get_closest_node(self, x, y, cart_trans=False):
        """Gets node closest to the given x, y

        Args:
            x (int/float): The X position of the center to search from
            y (int/float): The y position of the center to search from
            cart_trans (boolean): If true the cart will look for the closest node that only exists in the current path it is on,
            otherwise it will look for the closest node in the entire graph

        Returns:
            The node closest to the (x,y) tuple
        """
        min_dist = 99999
        min_node = None

        # Just in case if the graph list in the for loop is outdated by a future update
        local_logic_graph = self.global_graph

        for node in local_logic_graph.nodes:
            # Position of each node in the graph
            node_posx = local_logic_graph.nodes[node]["pos"][0]
            node_posy = local_logic_graph.nodes[node]["pos"][1]

            dist = self.dis(node_posx, node_posy, x, y)

            # Buffer, if we are trying to find the node closest to the cart, lets not count nodes the cart isnt actively following
            if cart_trans:
                if dist < min_dist and local_logic_graph.nodes[node]["active"]:
                    min_node = node
                    min_dist = dist
            else:
                if dist < min_dist:
                    min_node = node
                    min_dist = dist

        # if we found a new closer node along our path let global planner know the current node and previous node
        if min_node is not self.current_cart_node and cart_trans:
            self.current_cart_node = min_node
            # rospy.loginfo("Current node: " + str(self.current_cart_node))
            # rospy.loginfo("Previous node: " + str(self.prev_cart_node))

        return min_node

    def dis(self, x1, y1, x2, y2):
        """Distance between two points

        Args:
            x1 (float): The X position of the first point
            y1 (float): The Y position of the first point
            x2 (float): The X position of the second point
            y2 (float): The y position of the second point

        Returns:
            Distance between the two points
        """
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    # Get the closest waypoint to the cart
    def pose_callback(self, msg):
        """On new cart positon, update the class on the new position and orientation information

        Args:
            msg (ROS PoseStamped Message): The PoseWithCovarianceStamped of the cart's position and orientation coming from /pcl_pose topic
        """
        self.current_pos = msg.pose

        # If the cart is mid-navigation calculation, theres no need to update the cart node until it is done
        if not self.calculating_nav:
            self.current_cart_node = self.get_closest_node(
                msg.pose.pose.position.x, msg.pose.pose.position.y, cart_trans=True
            )

        # Obtain euler angles from pose orientation of cart
        cart_quat = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        self.orientation = tf_transformations.euler_from_quaternion(cart_quat)

    # We've received a clicked point from RViz, calculate a path to it
    def point_callback(self, msg):
        """If a point was published in RViz figure out how to get there

        Args:
            msg (PointStamped): The Clicked Point coming from the publish in RViz
        """
        self.calc_nav(msg.point)

    def state_change(self, msg):
        """Keeps the global planner updated on the current state of the cart

        Args:
            msg (VehicleState): A VehicleState message containing navigation status
        """
        self.navigating = msg.is_navigating

    def vel_callback(self, msg):
        """Keeps the global planner updated on current speed of the cart

        Args:
            msg (Float32 message): Contains current cart speed
        """
        # Estimated velocity can fluctuate slightly, so an average is ideal
        if self.vel_polls < 5:
            self.cur_speed += msg.data
            self.vel_polls += 1
        else:
            self.cur_speed = self.cur_speed / self.vel_polls
            self.vel_polls = 0
            self.cur_speed = 0

    def output_path_gps(self, path, single=False):
        """Function for converting the list of points along a path to latitude and longitude

        Args:
            path (LocalPointsArray message): List of X,Y points along the path
            single (Boolean): Whether or not you're sending a path with a single point or not (e.g. see output_pos_gps below)
        """
        gps_path = LatLongArray()

        for pose in path.localpoints:
            # Convert local coordinates to GPS using landmark-based calibration
            lat, lon = simple_gps_util.local_to_gps(
                pose.position.x, pose.position.y,
                self.ref_lat, self.ref_lon,
                self.cx_local, self.cy_local,
                self.cx_gps, self.cy_gps,
                self.calibration_theta
            )

            final_pose = LatLongPoint()
            final_pose.latitude = lat
            final_pose.longitude = lon

            # If we only care about a single point (e.g. cart position) send it off
            if single:
                return final_pose

            gps_path.gpspoints.append(final_pose)

            # Testing log to make sure conversion is working correctly
            # TODO: Remove logger after testing
            self.get_logger().info(f"Converted local point ({pose.position.x}, {pose.position.y}) to GPS coordinates ({final_pose.latitude}, {final_pose.longitude})")

        # Publish here
        self.gps_path_pub.publish(gps_path)

    def output_pos_gps(self):
        """Outputs the cart location in GPS and publishes. This uses the GPS Util for an approximate solution
        rather than GPS which can be relatively inaccurate.

        """
        if self.navigating and self.gps_calibrated and self.current_pos is not None:
            package_point = LocalPointsArray()
            cart_pos = self.current_pos.pose
            package_point.localpoints.append(cart_pos)

            point_in_gps = self.output_path_gps(package_point, single=True)

            self.gps_pose_pub.publish(point_in_gps)

    def gps_request_cb(self, msg):
        """Converts a GPS point from Lat, Long to local coordinates using landmark-based calibration. Also displays the GPS once converted, in RViz

        Args:
            msg (ROS LatLongPoint Message): Message containing the latitude and longitude to convert and navigate to
        """
        local_point = Point()

        # Convert GPS to local using landmark-based calibration
        local_point.x, local_point.y = simple_gps_util.gps_to_local(
            msg.latitude, msg.longitude,
            self.ref_lat, self.ref_lon,
            self.cx_local, self.cy_local,
            self.cx_gps, self.cy_gps,
            self.calibration_theta
        )

        self.calc_nav(local_point)

        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "/map"

        marker.ns = "GPS_NS"
        marker.id = 0
        marker.type = marker.CUBE
        marker.action = 0

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # rclpy uses Duration from the rclpy.duration module, not rclpy directly
        marker.lifetime = rclpy.duration.Duration(seconds=10).to_msg()

        marker.pose.position.x = local_point.x
        marker.pose.position.y = local_point.y
        marker.pose.position.z = 0.0

        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 0.8

        self.display_pub.publish(marker)

    def log_header(self, msg):
        """Helper method to print noticeable log statements."""
        self.get_logger().info("=" * 50)
        self.get_logger().info(f"{msg}")
        self.get_logger().info("=" * 50)


def main():
    """The main method that actually handles spinning up the node."""

    rclpy.init()
    node = GlobalPlanner()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
