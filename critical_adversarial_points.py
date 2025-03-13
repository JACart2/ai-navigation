# Ensure you have numpy open3d torch and rosbag installed on your computer

import numpy as np
import torch
import torch.nn as nn
from rosbags.highlevel import AnyReader
from rosbags.typesys import get_typestore
import open3d as o3d

# Load and Extract LiDAR Point Cloud from .bag File
def extract_point_cloud_from_bag(bag_file, topic="/velodyne_points"):
    points = []
    with AnyReader([bag_file]) as reader:
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == topic:
                typestore = get_typestore(reader)
                msg = typestore.deserialize(rawdata, connection.msgtype)
                for p in msg.data:
                    points.append([p.x, p.y, p.z])
    return np.array(points)

# Step 2: Adversarial Attack - Adding Perturbations to the Point Cloud
def generate_adversarial_points(point_cloud, model, epsilon=0.01, num_iterations=10):
    # Convert point cloud to torch tensor
    point_cloud_tensor = torch.tensor(point_cloud, dtype=torch.float32, requires_grad=True)

    # Define loss function
    criterion = nn.CrossEntropyLoss()

    # Target label (set an incorrect label)
    target_label = torch.tensor([1])  # Change this as needed

    # Optimizer
    optimizer = torch.optim.Adam([point_cloud_tensor], lr=0.01)

    for _ in range(num_iterations):
        optimizer.zero_grad()
        
        # Forward pass (fake model prediction for demonstration)
        outputs = torch.randn(1, 2, dtype=torch.float32)  # Replace with `model(point_cloud_tensor)`
        
        # Compute loss
        loss = criterion(outputs, target_label)
        
        # Backward pass
        loss.backward()
        
        # Update the point cloud
        with torch.no_grad():
            perturbation = epsilon * point_cloud_tensor.grad.sign()
            point_cloud_tensor += perturbation
            point_cloud_tensor = torch.clamp(point_cloud_tensor, min=-1, max=1)
            point_cloud_tensor.grad.zero_()

    return point_cloud_tensor.detach().numpy()

# Step 3: Visualizing the Original and Adversarial Point Clouds
def visualize_point_clouds(original, adversarial):
    pcd_orig = o3d.geometry.PointCloud()
    pcd_orig.points = o3d.utility.Vector3dVector(original)
    pcd_orig.paint_uniform_color([0, 0, 1])  # Blue for original

    pcd_adv = o3d.geometry.PointCloud()
    pcd_adv.points = o3d.utility.Vector3dVector(adversarial)
    pcd_adv.paint_uniform_color([1, 0, 0])  # Red for adversarial

    o3d.visualization.draw_geometries([pcd_orig, pcd_adv])

# Main Execution
if __name__ == "__main__":
    bag_file = "C:\Users\Shano\OneDrive\Desktop\480\2021-02-11-15-37-25.bag"
    topic = "/velodyne_points"  # Update based on your dataset

    # Step 1: Extract point cloud
    point_cloud = extract_point_cloud_from_bag(bag_file, topic)

    # Step 2: Generate adversarial point cloud
    adversarial_point_cloud = generate_adversarial_points(point_cloud, model=None)  # Replace model=None with your trained model

    # Step 3: Visualize results
    visualize_point_clouds(point_cloud, adversarial_point_cloud)
