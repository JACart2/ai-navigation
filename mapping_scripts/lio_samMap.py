import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d
import numpy as np
from std_msgs.msg import Header
import copy

# Global variables for point clouds
global_map = None
previous_pc = None


class PointCloudProcessor(Node):
    def __init__(self):
        super().__init__("point_cloud_icp_registration")

        # Subscriber to the point cloud topic
        self.create_subscription(
            PointCloud2,
            "/lio_sam/mapping/cloud_registered",  # Replace with your pointcloud topic
            self.point_cloud_callback,
            10,
        )

        self.get_logger().info(
            "ICP Registration Node started. Processing point clouds."
        )

    def point_cloud_callback(self, msg):
        global global_map, previous_pc

        # Convert PointCloud2 message to Open3D point cloud
        pc_data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points = np.array(list(pc_data))
        current_pc = o3d.geometry.PointCloud()
        current_pc.points = o3d.utility.Vector3dVector(points)

        if previous_pc is None:
            # First point cloud, no ICP registration required
            previous_pc = current_pc
            global_map = copy.deepcopy(previous_pc)
            self.get_logger().info(
                "First point cloud received, initializing global map."
            )
            return

        # ICP Registration between the previous and current point cloud
        self.get_logger().info("Performing ICP registration...")
        reg_p2p = o3d.registration.registration_icp(
            current_pc,
            previous_pc,
            max_correspondence_distance=0.05,
            init=np.identity(4),
            estimation_method=o3d.registration.TransformationEstimationPointToPoint(),
        )

        # Apply the transformation to align the current point cloud
        aligned_pc = current_pc.transform(reg_p2p.transformation)

        # Update the global map by combining the aligned point cloud with the existing map
        if global_map is not None:
            global_map += aligned_pc
        else:
            global_map = aligned_pc

        # Update previous_pc for the next iteration
        previous_pc = aligned_pc

        # Optionally, save the global map every so often
        # global_map.save("global_map.pcd")  # Uncomment this to save periodically

        self.get_logger().info("Point cloud registered and added to global map.")

    def save_global_map(self):
        # Save the final global map to a PCD file
        if global_map is not None:
            self.get_logger().info("Saving global map to global_map.pcd.")
            o3d.io.write_point_cloud("global_map.pcd", global_map)


def main(args=None):
    rclpy.init(args=args)

    processor = PointCloudProcessor()

    try:
        # Spin to keep the node alive and processing point clouds
        rclpy.spin(processor)
    except KeyboardInterrupt:
        pass
    finally:
        # Shutdown and save the final global map
        processor.save_global_map()
        processor.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
