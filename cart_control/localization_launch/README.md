# Localization_Launcher Launch File Components
1. Velodyne LiDAR Driver
Launches the velodyne_driver node for VLP16 LiDAR with specified parameters.
2. Velodyne LiDAR Transform
Includes the launch configuration for transforming Velodyne LiDAR data.
3. LiDAR Localization
Includes the launch configuration for LiDAR-based localization.
4. ZED Multi-Camera Setup
Launches the ZED Multi-Camera system with specified camera configurations.
5. Static Transform
Sets up a static transform between the reference link and the base link for camera positioning.
6. RViz2 Visualization
Launches RViz2 with a predefined configuration file for visualization.
## Customization
Modify the launch configurations or parameters as needed for your specific setup.
Ensure correct paths to configuration files and packages.

# Lidar_Localization Luanch File
This is our version of the Localization launch file provided in the (insert link here) package
This contains changes for the frame positions of the velodyne in relativity to the cart
# Localization Parameters Configuration

## Description
The localization parameters are configured to fine-tune the behavior and performance of the localization system. These parameters control various aspects such as registration method, point cloud processing, sensor setup, initial pose, and debugging options.
```
    These parameters are fined tuned for our system so only change if absolutely necessary
```

## Parameters
Registration Method
registration_method: Specifies the method used for point cloud registration. Current method: NDT_OMP.
### NDT Parameters
1. ndt_resolution: Resolution parameter for the Normal Distributions Transform (NDT).
2. ndt_step_size: Step size used in the NDT algorithm.
3. ndt_num_threads: Number of threads used for NDT computation.
4. transform_epsilon: Epsilon value for convergence criterion in the NDT algorithm.
### Point Cloud Processing
1. voxel_leaf_size: Leaf size for voxel grid downsampling of point cloud data.
2. scan_max_range: Maximum range of LiDAR scans.
3. scan_min_range: Minimum range of LiDAR scans.
4. scan_period: Time interval between consecutive scans.
### Map Configuration
1. use_pcd_map: Flag to indicate whether to use a pre-built point cloud map.
2. map_path: Path to the point cloud map file.
### Initial Pose
1. set_initial_pose: Flag to indicate whether to set the initial pose.
2. initial_pose_x, initial_pose_y, initial_pose_z: Initial position coordinates.
3. initial_pose_qx, initial_pose_qy, initial_pose_qz, initial_pose_qw: Initial orientation (quaternion).
### Sensor Setup
1. use_odom: Flag to indicate whether to use odometry data.
2. use_imu: Flag to indicate whether to use IMU data.
### Debugging
1. enable_debug: Flag to enable/disable debugging options.
### Frame IDs
1. global_frame_id: ID of the global frame (map).
2. odom_frame_id: ID of the odometry frame.
3. base_frame_id: ID of the base frame (robot's base link).
### Customization
Modify the parameter values as needed to adapt to specific localization requirements.
Adjust parameters based on sensor characteristics, environment conditions, and performance considerations.
Ensure correct paths are provided for map files and other resources.