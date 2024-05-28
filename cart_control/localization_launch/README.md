# Localization Launch File
This is the custom JACART launch file created to launch the components associated with localization
This includes launching:
1. Velodyne Lidar Packages - Converts data to raw velodyne packets then to velodyne pointcloud data
2. ZED Multi Camera - Runs the ZED packages in order to collect the Data coming off the zed cameras
3. Lidar_localization_ros2 - Takes that Velodyne point cloud data and the odomentry provided by the ZED and actually runs the localization processs
