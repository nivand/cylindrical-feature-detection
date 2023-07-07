# cylindrical-feature-detection
 Python script for acquiring and analyzing point cloud data from a lidar sensor using RANSAC-based segmentation.
 # Lidar Point Cloud Analysis

This repository contains a Python script for acquiring and analyzing point cloud data from a lidar sensor. The script uses the RPLidar library to acquire range and bearing data from the sensor, and then converts this data into a point cloud. The point cloud is then analyzed to identify cylindrical features, using a RANSAC-based segmentation algorithm.

## Dependencies

The script requires the following Python libraries:

- `math`
- `numpy`
- `open3d`
- `rplidar`
- `matplotlib`

You can install these dependencies using pip:

```
pip install numpy open3d rplidar matplotlib
```

## Usage

The main functions in the script are:

- `acquire_range_and_bearing_data(num_iterations=1000)`: Acquires range and bearing data from the lidar sensor. The number of iterations can be specified as an argument.

- `range_and_bearing_to_cartesian(range_and_bearing_data)`: Converts range and bearing data to Cartesian coordinates.

- `ransac_cylinder_segmentation(pc, radius, tolerance, iterations)`: Performs RANSAC-based segmentation on the point cloud to identify cylindrical features.

- `extract_cylindrical_features(pc, cylinder_radius, tolerance, iterations=1000)`: Extracts cylindrical features from the point cloud.

- `plot_point_cloud_and_cylindrical_features(pc, cylindrical_features)`: Plots the point cloud and the identified cylindrical features.

The main code at the end of the script demonstrates how to use these functions. It acquires data from the lidar sensor, converts it to a point cloud, extracts cylindrical features, and then plots the results.

## Example

Here is an example of how to use the script:

```python
# Acquire data from the lidar sensor
range_and_bearing_data = acquire_range_and_bearing_data()

# Convert the data to a point cloud
point_cloud_data = range_and_bearing_to_cartesian(range_and_bearing_data)

# Set parameters for cylindrical feature extraction
cylinder_radius = 0.05
tolerance = 0.01
iterations = 1000

# Extract cylindrical features
cylindrical_feature_indices = extract_cylindrical_features(point_cloud_data, cylinder_radius, tolerance, iterations)
cylindrical_features = point_cloud_data[cylindrical_feature_indices]

# Plot the point cloud and the cylindrical features
plot_point_cloud_and_cylindrical_features(point_cloud_data, cylindrical_features)
```

