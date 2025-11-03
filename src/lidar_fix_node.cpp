#include "lidar_fix/lidar_fix_node.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <limits>
#include <cmath> 

using namespace lidar_fix_params;

namespace {
  const float NaN = std::numeric_limits<float>::quiet_NaN();
}

// --------------------------------------------------
// LidarFix Node
// --------------------------------------------------
LidarFix::LidarFix() : rclcpp::Node("lidar_fix_node")
{
  this->declare_parameter<std::string>("topic_in",  "/ouster/points");
  this->declare_parameter<std::string>("topic_out", "/ouster/points/fix");

  const auto topic_in  = this->get_parameter("topic_in").as_string();
  const auto topic_out = this->get_parameter("topic_out").as_string();

  sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    topic_in, rclcpp::SensorDataQoS(),
    std::bind(&LidarFix::lidar_callback, this, std::placeholders::_1));

  pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_out, 10);

  el_table_.assign(kRings, 0.0f); 
}

// helper function: azimuth → ix
int LidarFix::az_to_ix(float az) const {
  float u = 0.5f - az / (2.0f * static_cast<float>(M_PI)); // az ∈ [-π, π] → u ∈ [0,1]

  if (u < 0.0f) u = 0.0f;
  if (u > 1.0f) u = 1.0f;
  int ix = static_cast<int>(std::floor(u * static_cast<float>(kCols)));
  if (ix >= kCols) ix = kCols - 1;
  return ix;
}

// --------------------------------------------------
// 1. range image formulation (with fixed size: 64 x 1024)
// 2. el_table_ formulation (get iy from ring field, average ring points and save the value in el_table_)
// --------------------------------------------------
void LidarFix::init_index_and_radius(const sensor_msgs::msg::PointCloud2& cloud,
                                     cv::Mat& index,
                                     std::vector<float>& r_h,
                                     std::vector<float>& az,
                                     std::vector<float>& el)
{
  const int H = kRings, W = kCols; // (64, 1024)
  index = cv::Mat(H, W, CV_32SC1, cv::Scalar(-1));

  const size_t N = static_cast<size_t>(cloud.width) * static_cast<size_t>(cloud.height);
  r_h.assign(N, -1.f);
  az .assign(N,  0.f);
  el .assign(N,  0.f);

  // field offsets
  size_t off_x = SIZE_MAX, off_y = SIZE_MAX, off_z = SIZE_MAX, off_ring = SIZE_MAX;
  for (const auto& f : cloud.fields) {
    if      (f.name == "x") off_x = f.offset;
    else if (f.name == "y") off_y = f.offset;
    else if (f.name == "z") off_z = f.offset;
    else if (f.name == "ring") off_ring = f.offset;

  }
  if (off_x == SIZE_MAX || off_y == SIZE_MAX || off_z == SIZE_MAX || off_ring == SIZE_MAX) return;

  const uint8_t* base = cloud.data.data();
  const uint32_t step = cloud.point_step;
  const size_t   all  = cloud.data.size();

  // buffers for calculating mean elevation per ring
  std::vector<double> el_sum(H,0.0);
  std::vector<int>    el_cnt(H,0);

  size_t i = 0;
  for (size_t off = 0; off + step <= all; off += step, ++i) {
    const float x = *reinterpret_cast<const float*>(base + off + off_x);
    const float y = *reinterpret_cast<const float*>(base + off + off_y);
    const float z = *reinterpret_cast<const float*>(base + off + off_z);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

    const float rh  = std::hypot(x, y);   // root(x^2+y^2)
    const float pan = std::atan2(y, x);   // [-pi, pi]
    const float elv = std::atan2(z, rh);  // elevation

    if (i < r_h.size()) {r_h[i] = rh; az [i] = pan; el [i] = elv;}

    //get ring value from pcl2 field
    const uint16_t ring = *reinterpret_cast<const uint16_t*>(base + off + off_ring);
    if (ring >= static_cast<uint16_t>(H)) continue;

    const int ix = az_to_ix(pan);
    const int iy = static_cast<int>(ring);

    index.at<int>(iy, ix) = static_cast<int>(i);

    el_sum[iy] += static_cast<double>(elv);
    el_cnt[iy] += 1;
  }

  el_table_.resize(H);
  for (int r = 0; r < H; ++r) {
    if (el_cnt[r] > 0) {
      el_table_[r] = static_cast<float>(el_sum[r] / el_cnt[r]);
    } else {
      // assign the value from the previous element in el_table_ if available
      el_table_[r] = (r > 0) ? el_table_[r - 1] : 0.0f;
    }
  }
  // back-fill from the end to handle cases where only the front was filled with zeros
  for (int r = H - 2; r >= 0; --r) {
    if (el_cnt[r] == 0) el_table_[r] = el_table_[r + 1];
  }
}

// --------------------------------------------------
// allocate the closest ring number to el_sym
// --------------------------------------------------
int LidarFix::nearest_ring_from_el(float elv) const
{
  int best = 0;
  float best_err = std::fabs(elv - el_table_[0]);
  for (int r = 1; r < static_cast<int>(el_table_.size()); ++r) {
    float e = std::fabs(elv - el_table_[r]);
    if (e < best_err) { best_err = e; best = r; }
  }
  return best;
}

// --------------------------------------------------
// main callback: symmetric-point check + restore
// --------------------------------------------------
void LidarFix::lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // ground z estimation
  const float ground_z = estimate_ground_z(msg, kGrndBinSz, kGrndZMin, kGrndZMax);

  // range image formulation (index, r_h, az, el)
  cv::Mat index;
  std::vector<float> r_h, az, el;
  init_index_and_radius(*msg, index, r_h, az, el);

  // copy message for publishing
  auto out = *msg;

  // field offsets
  const uint32_t step = out.point_step;
  size_t off_x = SIZE_MAX, off_y = SIZE_MAX, off_z = SIZE_MAX, off_ring = SIZE_MAX, off_intensity = SIZE_MAX;
  for (const auto& f : out.fields) {
    if      (f.name == "x") off_x = f.offset;
    else if (f.name == "y") off_y = f.offset;
    else if (f.name == "z") off_z = f.offset;
    else if (f.name == "ring") off_ring = f.offset;
    else if (f.name == "intensity") off_intensity = f.offset;
  }
  if (off_x == SIZE_MAX || off_y == SIZE_MAX || off_z == SIZE_MAX || off_ring == SIZE_MAX) {
    pub_->publish(out);
    return;
  }

  uint8_t*       db  = out.data.data();
  const uint8_t* sb  = msg->data.data();
  const size_t   all = out.data.size();

  
  // Thresholds get bigger as r gets bigger, in order to consider the sparsity of pcl
  auto dz_thr = [&](float r){ return kDzBase + kDzPerMeter * r; }; 
  auto dr_thr = [&](float r){ return kDrBase + kDrPerMeter * r; };

  // 3) Iterate through the pcl
  size_t i = 0;
  for (size_t off = 0; off + step <= all; off += step, ++i) {
    float& X = *reinterpret_cast<float*>(db + off + off_x);
    float& Y = *reinterpret_cast<float*>(db + off + off_y);
    float& Z = *reinterpret_cast<float*>(db + off + off_z);

    const float x = *reinterpret_cast<const float*>(sb + off + off_x);
    const float y = *reinterpret_cast<const float*>(sb + off + off_y);
    const float z = *reinterpret_cast<const float*>(sb + off + off_z);

    // invalid → NaN
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      X = Y = Z = NaN; continue;
    }

    // Check the existence of symmetric point for pcls with z < ground_z 
    if (!(z < ground_z)) { X = x; Y = y; Z = z; continue; }

    const float rh  = (i < r_h.size()) ? r_h[i] : -1.f;
    const float azv = (i < az.size())  ? az[i]  :  0.f;
    if (rh < 0.f) { X = x; Y = y; Z = z; continue; } 

    const uint16_t ring = *reinterpret_cast<const uint16_t*>(sb + off + off_ring);
    if (ring >= static_cast<uint16_t>(kRings)) { X = x; Y = y; Z = z; continue; }

    // Infer (ix, iy) of the symmetric point
    const float z_sym = 2.f * ground_z - z;
    const float el_sym = std::atan2(z_sym, rh);
    const int   ix = az_to_ix(azv);
    const int   iy_tgt = nearest_ring_from_el(el_sym);

    // Search 3×3 points near the predicted point (ix, iy) (kAzTol = 3)
    const int xs = std::max(0, ix - kAzTol);
    const int xe = std::min(kCols - 1, ix + kAzTol);
    const int ys = std::max(0, iy_tgt - kRingTol);
    const int ye = std::min(kRings - 1, iy_tgt + kRingTol);

    bool mirrored = false;
    float bestCost = 1e9f;

    for (int cx = xs; cx <= xe; ++cx) {
      for (int cy = ys; cy <= ye; ++cy) {
        const int k = index.at<int>(cy, cx);
        if (k < 0) continue;

        const size_t offk = static_cast<size_t>(k) * step;
        if (offk + step > all) continue;

        const float qx = *reinterpret_cast<const float*>(sb + offk + off_x);
        const float qy = *reinterpret_cast<const float*>(sb + offk + off_y);
        const float qz = *reinterpret_cast<const float*>(sb + offk + off_z);
        if (!std::isfinite(qx) || !std::isfinite(qy) || !std::isfinite(qz)) continue;

        const float rh_q = std::hypot(qx, qy);
        const float dz   = std::fabs(qz - z_sym);
        const float dr   = std::fabs(rh_q - rh);

        if (dz <= dz_thr(rh) && dr <= dr_thr(rh)) {
          // check if it is a ghost point or a real under-ground-point
          bool is_likely_reflection = true;
          
          if (off_intensity != SIZE_MAX) {
            const float intensity_p = *reinterpret_cast<const float*>(sb + off + off_intensity); // reflection value at the current point (z < grnd_z)
            const float intensity_q = *reinterpret_cast<const float*>(sb + offk + off_intensity); // actual reflection value at the symmetric point
            if (intensity_q > 1e-6) {
              const float ratio = intensity_p / intensity_q;
              if (ratio >= kIntensityR) {
                is_likely_reflection = false; // if the ratio is high, it's more likely a real object rather than a reflection
              }
            }
          }
          if (is_likely_reflection) {
            const float cost = dz + dr;
            if (cost < bestCost) { bestCost = cost; mirrored = true; }
          }
        }
      }
    }

    if (mirrored) {
      const float ratio = (z == 0.f) ? 1.f : (ground_z / z);
      X = x * ratio;
      Y = y * ratio;
      Z = ground_z;
    } else {
      X = x; Y = y; Z = z;
    }
  }

  pub_->publish(out);
}

// --------------------------------------------------
// ground z estimator
// --------------------------------------------------
float LidarFix::estimate_ground_z(const sensor_msgs::msg::PointCloud2::SharedPtr msg,
                                  float bin_sz, float z_min, float z_max)
{
  const float d2_min = kD2Min;
  const float d2_max = kD2Max;

  int nbins = static_cast<int>((z_max - z_min) / bin_sz) + 1;
  if (nbins < 3) nbins = 3;

  std::vector<float> hist(nbins, 0.f);
  std::vector<int>   cnt (nbins, 0);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    const float z = *iter_z;
    if (!std::isfinite(z)) continue;
    if (z < z_min || z > z_max) continue;

    const float x  = *iter_x;
    const float y  = *iter_y;
    const float d2 = x*x + y*y;
    if (d2 < d2_min || d2 > d2_max) continue;

    int idx = static_cast<int>((z - z_min) / bin_sz);
    if (idx < 0) idx = 0;
    if (idx >= nbins) idx = nbins - 1;

    hist[idx] += z;
    cnt [idx] += 1;
  }

  int max_idx = -1, max_cnt = -1;
  for (int i = 1; i < nbins - 1; ++i) {
    if (cnt[i] > max_cnt) { max_cnt = cnt[i]; max_idx = i; }
  }
  if (max_idx < 0 || max_cnt <= 0) return 0.f;

  float ground_z = (hist[max_idx] + hist[max_idx - 1] + hist[max_idx + 1]) /
                   (cnt [max_idx] + cnt [max_idx - 1] + cnt [max_idx + 1]);

  int si1 = max_idx, ei1 = max_idx;
  while (si1 > 1 && (cnt[si1 - 1] <= cnt[si1])) si1--;
  while (ei1 < nbins - 1 && (cnt[ei1 + 1] <= cnt[ei1])) ei1++;

  int max_idx2 = -1, max_cnt2 = -1;
  for (int i = 1; i < si1; ++i) {
    if (cnt[i] > max_cnt2) { max_cnt2 = cnt[i]; max_idx2 = i; }
  }
  if (max_idx2 < 0) return ground_z;

  int si2 = max_idx2, ei2 = max_idx2;
  while (si2 > 1 && (cnt[si2 - 1] <= cnt[si2])) si2--;
  while (ei2 < si1 && (cnt[ei2 + 1] <= cnt[ei2])) ei2++;

  int vol1 = 0; for (int i = si1; i <= ei1; ++i) vol1 += cnt[i];
  int vol2 = 0; for (int i = si2; i <= ei2; ++i) vol2 += cnt[i];

  if (vol1 < vol2) {
    int m = max_idx2;
    ground_z = (hist[m] + hist[m - 1] + hist[m + 1]) /
               (cnt [m] + cnt [m - 1] + cnt [m + 1]);
  }
  return ground_z;

}

