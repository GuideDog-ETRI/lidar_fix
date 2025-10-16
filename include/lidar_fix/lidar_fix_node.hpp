#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <opencv2/core.hpp>
#include <vector>
#include <cstdint>
#include <cmath>

namespace lidar_fix_params {

// Range image size
inline constexpr int   kRings = 64;     // H
inline constexpr int   kCols  = 1024;   // W

// Elevation range [rad] (adjust to your LiDAR, default value set for OS1)
inline constexpr float elMin = -static_cast<float>(M_PI)/8.0f;
inline constexpr float elMax =  +static_cast<float>(M_PI)/8.0f;

// Symmetry match thresholds (distance-adaptive)
inline constexpr float kDzBase     = 0.05f; // base z tol (m)
inline constexpr float kDzPerMeter = 0.03f; // per-meter increment
inline constexpr float kDrBase     = 0.05f; // base r_h tol (m)
inline constexpr float kDrPerMeter = 0.03f; // per-meter increment

// Neighborhood search (ix, iy)
inline constexpr int kAzTol   = 1;  // ±1 column
inline constexpr int kRingTol = 1;  // ±1 row

// Intensity ration threshold
inline constexpr float kIntensityR = 0.6f;

// Ground-z estimation config
inline constexpr float kGrndBinSz = 0.02f;  // 2 cm
inline constexpr float kGrndZMin  = -1.2f;
inline constexpr float kGrndZMax  =  0.2f;

// Ground-z estimation range gates (r^2)
inline constexpr float kD2Min =  1.0f * 1.0f; // r >= 1 m
inline constexpr float kD2Max = 3.0f * 3.0f; // r <= 3 m

}


class LidarFix : public rclcpp::Node {
public:
  LidarFix();

private:
  // ROS
  void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher  <sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

  // Ground z estimator (kept as-is)
  float estimate_ground_z(const sensor_msgs::msg::PointCloud2::SharedPtr msg,
                          float bin_sz = lidar_fix_params::kGrndBinSz,
                          float z_min  = lidar_fix_params::kGrndZMin,
                          float z_max  = lidar_fix_params::kGrndZMax);

  // Build range image via azimuth/elevation mapping
  void init_index_and_radius(const sensor_msgs::msg::PointCloud2& cloud,
                             cv::Mat& index,              // HxW, int, -1 init
                             std::vector<float>& r_h,     // horizontal range
                             std::vector<float>& az,      // azimuth
                             std::vector<float>& el);     // elevation

  // angle → index helpers
  inline int az_to_ix(float az) const; // [-pi, pi] → [0, kCols-1]

  int  nearest_ring_from_el(float elv) const;   // el → closest ring
  std::vector<float> el_table_;                 // size: kRings, filled in init_index_and_radius()
};