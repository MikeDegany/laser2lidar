# Laser2Lidar ROS 2 Package

## Overview

The `laser2lidar` package is a **ROS 2** package that converts **2D laser scan** (`sensor_msgs::msg::LaserScan`) data into **3D point cloud** (`sensor_msgs::msg::PointCloud2`) data using the `laser_geometry` library. This is useful for integrating 2D LiDAR sensors into 3D mapping, perception, and navigation systems.

### Features:

- Subscribes to a **LaserScan** topic (default topic name must be set in config file)
- Converts laser scan data into **PointCloud2** format
- Publishes the point cloud to an output topic (default topic name must be set in config file)
- Configurable topic names via **YAML configuration file** and **command-line arguments**
- ROS 2 **launch file** for easy execution

---

## Installation

### Prerequisites

Make sure you have the following installed:

- **ROS 2 Humble (or later)**: [Installation Guide](https://docs.ros.org/en/humble/Installation.html)
- **Colcon build system**
- **Dependencies:** `laser_geometry`, `sensor_msgs`, `rclcpp`

### Clone and Build the Package

```bash
# Navigate to your ROS 2 workspace
cd ~/ros2_ws/src

# Clone the repository
git clone https://github.com/MikeDegany/laser2lidar.git

# Navigate back to the workspace root
cd ~/ros2_ws

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the package
colcon build --packages-select laser2lidar

# Source the setup file
source install/setup.bash
```

---

## Usage

### 1. Running the Node with Default Parameters

You can start the `laser2lidar` node using the **launch file**:

```bash
ros2 launch laser2lidar laser2lidar.launch.py
```

This will:

- Load parameters from `config/params.yaml`
- Subscribe to the default LaserScan topic
- Publish the converted PointCloud2 data to the default topic

### 2. Running with Custom Topics

To override the input and output topics, use command-line arguments:

```bash
ros2 launch laser2lidar laser2lidar.launch.py input_topic:=/custom_scan output_topic:=/custom_pointcloud
```

### 3. Running the Node Directly

If you prefer to run the node manually:

```bash
ros2 run laser2lidar laser2lidar_node --ros-args --params-file src/laser2lidar/config/params.yaml
```

### 4. Viewing the Published Point Cloud

To visualize the output, use **RViz2**:

```bash
rviz2
```

- Add a **PointCloud2** display
- Set the topic to `/pointcloud`
- Adjust color and size settings as needed
- Set the fixed frame as your laser scan frame
---

## Configuration

The node reads parameters from a YAML configuration file located at `config/params.yaml`. Example:

```yaml
laser2lidar_node:
  ros__parameters:
    input_topic: "/lase_scan_topic"
    output_topic: "/pointcloud_topic"
```

You can edit this file to set custom topic names.

---

## Package Structure

```
laser2lidar/
├── CMakeLists.txt        # Build system file
├── config/
│   └── params.yaml      # Configuration file for topic names
├── include/
│   └── laser2lidar/     # Header files (if needed in future)
├── launch/
│   └── laser2lidar.launch.py  # Launch file
├── package.xml          # ROS 2 package metadata
└── src/
    └── laser2lidar.cpp  # Main node implementation
```

---

## Troubleshooting

### 1. Node Fails to Start with "Invalid topic name must not be empty"

Ensure that:

- The `params.yaml` file is correctly formatted.
- You provide valid topic names in the configuration or as command-line arguments.

### 2. "Failed to Create Symbolic Link" Error During Build

Try cleaning the workspace and rebuilding:

```bash
rm -rf build/ install/ log/
colcon build --packages-select laser2lidar
```

### 3. Point Cloud Not Visible in RViz

- Verify that the `frame_id` in **LaserScan** matches what RViz expects.
- Ensure the sensor is publishing valid scan data.
- Check that the `laser_geometry` package is installed and working.

---

## Contributing

1. Fork the repository on GitHub.
2. Create a new feature branch: `git checkout -b feature-name`
3. Make changes and commit: `git commit -m "Add new feature"`
4. Push to your fork: `git push origin feature-name`
5. Submit a Pull Request.

---

## Future Work

Make this a one-header only library
---
## License

This package is released under the **MIT License**. See the [LICENSE](LICENSE) file for details.

---

## Contact

For issues or suggestions, create an issue on GitHub or contact:

- **GitHub:** [https://github.com/MikeDegany/laser2lidar](https://github.com/MikeDegany/laser2lidar)
- **Email:** [Mike.Degany@gmail.com](mailto\:Mike.Degany@gmail.com)

