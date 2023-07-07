import math
import numpy as np
import open3d as o3d
from rplidar import RPLidar
import matplotlib.pyplot as plt

def acquire_range_and_bearing_data(num_iterations=1000):
    lidar = RPLidar('COM5')
    range_and_bearing_data = []
    for i, data in enumerate(lidar.iter_measurments()):
        _, angle, distance = data[1], data[2], data[3]
        range_and_bearing_data.append((distance, angle))
        if i >= num_iterations:
            break
    lidar.stop()
    lidar.disconnect()
    return range_and_bearing_data



def range_and_bearing_to_cartesian(range_and_bearing_data):
    points = [(d*math.cos(math.radians(a)), d*math.sin(math.radians(a)), 0) for d, a in range_and_bearing_data]
    return np.asarray(points)

def point_cloud_to_o3d(points):
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(points)
    
    return pc

def point_cloud_distance_to_cylinder(pc, cylinder_center, cylinder_axis, radius):
    pc_centered = pc - cylinder_center
    projected = np.dot(pc_centered, cylinder_axis)
    projection_points = cylinder_center + projected[:, None] * cylinder_axis
    distances = np.linalg.norm(pc_centered - projection_points, axis=1)
    return np.abs(distances - radius)

def ransac_cylinder_segmentation(pc, radius, tolerance, iterations):
    best_inliers = []
    n_points = pc.shape[0]

    if n_points < 2:
        print("Not enough points in the point cloud for RANSAC.")
        return best_inliers

    for _ in range(iterations):
        sample_indices = np.random.choice(n_points, size=2, replace=False)
        p1, p2 = pc[sample_indices]
        cylinder_axis = p2 - p1
        cylinder_axis_norm = np.linalg.norm(cylinder_axis)

        if cylinder_axis_norm == 0:
            continue
        else:
            cylinder_axis /= cylinder_axis_norm

        distances = point_cloud_distance_to_cylinder(pc, p1, cylinder_axis, radius)
        inliers = np.where(np.abs(distances) < tolerance)[0]

        if len(inliers) > len(best_inliers):
            best_inliers = inliers.astype(int)

    return best_inliers


def extract_cylindrical_features(pc, cylinder_radius, tolerance, iterations=1000):
    inlier_indices = ransac_cylinder_segmentation(pc, cylinder_radius, tolerance, iterations)
    inlier_pc = pc[inlier_indices]

    o3d_inlier_pc = o3d.geometry.PointCloud()
    o3d_inlier_pc.points = o3d.utility.Vector3dVector(inlier_pc)

    return np.array(inlier_indices, dtype=int)



# ... (keep the existing functions like acquire_range_and_bearing_data, range_and_bearing_to_cartesian, ransac_cylinder_segmentation, etc.) ...

def plot_point_cloud_and_cylindrical_features(pc, cylindrical_features):
    fig, ax = plt.subplots()
    ax.scatter(pc[:, 0], pc[:, 1], s=10, c='blue', label='Point Cloud')
    ax.scatter(cylindrical_features[:, 0], cylindrical_features[:, 1], s=20, c='red', label='Cylindrical Features')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.legend()
    plt.show()


import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ... rest of your code ...
cylinder_radius = 0.05
tolerance = 0.01
iterations = 1000
import matplotlib.pyplot as plt
import numpy as np

# Initialize the figure and the axes
fig, ax = plt.subplots()

# Initial empty plots
scatter = ax.scatter(np.array([]), np.array([]))

# Function to update the plot
def update(i):
    # Acquire data and extract features
    range_and_bearing_data = acquire_range_and_bearing_data()
    point_cloud_data = range_and_bearing_to_cartesian(range_and_bearing_data)
    cylindrical_feature_indices = extract_cylindrical_features(point_cloud_data, cylinder_radius, tolerance, iterations)
    cylindrical_features = point_cloud_data[cylindrical_feature_indices]

    # Update the scatter plot data
    scatter.set_offsets(point_cloud_data[:, :2])
    scatter.set_array(point_cloud_data[:, 2])

    # Redraw the figure
    fig.canvas.draw()

# Run the update function in a loop
for i in range(100):
    update(i)
    plt.pause(0.1)

plt.show()


# Main code
#range_and_bearing_data = acquire_range_and_bearing_data()
#point_cloud_data = range_and_bearing_to_cartesian(range_and_bearing_data)

# Set cylinder_radius, tolerance, and iterations based on your requirements
#cylindrical_feature_indices = extract_cylindrical_features(point_cloud_data, cylinder_radius, tolerance, iterations)
#cylindrical_features = point_cloud_data[cylindrical_feature_indices]

#plot_point_cloud_and_cylindrical_features(point_cloud_data, cylindrical_features)
