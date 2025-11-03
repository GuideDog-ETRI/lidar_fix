# lidar_fix, Introduction

The lidar_fix module compensates for LiDAR sensor errors caused by ground reflections. It takes raw LiDAR sensor data as input, estimates the sensor's height from the ground, and provides a corrected point cloud based on this estimation.

This repository is the official ROS 2 implementation for the algorithm presented in the paper:
Restoration of Indoor LiDAR Point Clouds from Specular Ground Reflections Using Geometric Consistency Authors: Ui-Hyun Hwang, Jae-Yeong Lee

If you use this code or algorithm in your research, please consider citing our paper.

# How It Works
This node tackles the problem of ghost points created by specular(glossy) floor reflections, which can severely degrade LiDAR-based perception tasks like SLAM and ground segmentation4. 
The algorithm operates in real-time by:
1. Estimating the Ground:
   Dynamically estimates the ground plane height ($z_{ground}$) using a z-value histogram on points within a close, reliable range.
2. Building a Range Image:
   Projects the 3D point cloud into a 2D range image (Ring vs. Azimuth). This allows for efficient, O(1) lookup of points in a specific direction
3. Symmetric Point Validation:
   For each point P found below the estimated ground (z < z_{ground}):It calculates the position of its theoretical symmetric counterpart P_s above the ground (where z_{symm} = 2z_{ground} - z). It    searches the range image for an actual, sensed point Q_{actual} in the neighborhood of P_s.
4. Multi-Stage-Check:
   Case 1 (Pothole): If no Q_{actual} is found, P is considered real terrain (e.g., a pit or hole) and is kept
   Case 2 (Stairs): If Q_{actual} is found, but its intensity I_q is similar to P's intensity I_p (i.e., I_p / I_q \ge R), it's considered real terrain (e.g., stairs) and is kept. This is because      real reflections lose energy (I_p should be < I_q).
   Case 3 (Ghost Point): If Q_{actual} is found and its intensity is significantly higher than P's (I_p / I_q < R), P is confirmed as a ghost point.
5. Restoration: Confirmed ghost points (Case 3) are projected back onto their correct position on the ground plane (z' = z_{ground}) using the geometric projection formula.

# Usage
This package provides a ROS 2 node (lidar_fix_node). It subscribes to a raw sensor_msgs::msg::PointCloud2 topic and publishes a corrected sensor_msgs::msg::PointCloud2 topic.
1. Build the Package
: Clone the repository into your ROS 2 workspace and build it:
cd ~/ros2_ws/src
git clone https://github.com/GuideDog-ETRI/lidar_fix.git
cd ..
colcon build --packages-select lidar_fix

2. Run the Node
: Source your workspace and launch the node. Using ros2 run:
source ~/ros2_ws/install/setup.bash
ros2 run lidar_fix lidar_fix_node
Using ros2 launch (if you create a launch file):
Bashros2 launch lidar_fix lidar_fix.launch.py

4. Topic Remapping: You can remap the input and output topics using ROS 2 arguments.
ros2 run lidar_fix lidar_fix_node \
    --ros-args \
    -p topic_in:="/my_lidar/points_raw" \
    -p topic_out:="/my_lidar/points_corrected"
   
#Parameters
These parameters are declared in lidar_fix_node.cpp and can be set via launch file or command line.
