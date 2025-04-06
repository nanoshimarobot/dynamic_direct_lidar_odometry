// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following papers:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.
//   T. Shan and B. Englot. LeGO-LOAM: Lightweight and Ground-Optimized Lidar
//   Odometry and Mapping on Variable Terrain
//      IEEE/RSJ International Conference on Intelligent Robots and Systems
//      (IROS). October 2018.
//
// Change Log: Jonathan Lichtenfeld (2023)
// - Added the following methods for DDLO integration:
//   - loadParams
//   - applySegmentation
//   - projectResiduals
//   - computeAllObjects
//   - trackDetections
//   - visualize
//   - all getter methods

#include <detection/detection.h>

DetectionModule::DetectionModule() : initialized_(false), icp_residuals_set_(false), it_(nh_)
{
  loadParams();
  if (evaluate_)
    setupEvaluation();

  allocateMemory();
  resetParameters();

  label_img_pub_ = it_.advertise("label_image", 1);
  range_img_pub_ = it_.advertise("range_image", 1);
  residual_img_pub_ = it_.advertise("residual_image", 1);

  time_prev_ = ros::Time(0);

  time_stats_.insert(std::make_pair("projectScan", AccumulatorData()));
  time_stats_.insert(std::make_pair("projectResiduals", AccumulatorData()));
  time_stats_.insert(std::make_pair("groundRemoval", AccumulatorData()));
  time_stats_.insert(std::make_pair("cloudSegmentation", AccumulatorData()));
  time_stats_.insert(std::make_pair("computeAllObjects", AccumulatorData()));
  time_stats_.insert(std::make_pair("trackDetections", AccumulatorData()));
}

void DetectionModule::loadParams()
{
  odom_frame_ = nh_.param<std::string>("odomFrame", "odom");
  child_frame_ = nh_.param<std::string>("childFrame", "base_link");

  H_ = nh_.param("odomNode/detection/rows", 128);
  W_ = nh_.param("odomNode/detection/columns", 1024);

  is_organized_ = nh_.param("odomNode/detection/organized", false);
  ang_bottom_ = nh_.param("odomNode/detection/ang_bottom", 45);
  ang_res_x_ = 360.0 / float(W_);
  ang_res_y_ = 2 * ang_bottom_ / float(H_ - 1);
  start_orientation_ = -9999;
  int tmp;  // for casting int to size_t
  tmp = nh_.param("odomNode/detection/groundRows", 30);
  ground_rows_ = tmp;
  if (ground_rows_ > H_)
    ROS_ERROR("\"ground_scan_ind_\" cannot be greater than \"H\"!");
  ground_angle_threshold_ = nh_.param("odomNode/detection/groundAngleThreshold", 10);
  minimum_range_ = nh_.param("odomNode/detection/minimumRange", 10);
  sensor_mount_angle_ = nh_.param("odomNode/detection/sensorMountAngle", 10);
  theta_ = nh_.param("odomNode/detection/theta", 60.0 / 180.0 * M_PI);
  tmp = nh_.param("odomNode/detection/minPointNum", 20);
  min_point_num_ = tmp;
  tmp = nh_.param("odomNode/detection/validPointNum", 15);
  valid_point_num_ = tmp;
  tmp = nh_.param("odomNode/detection/minLineNum", 5);
  min_line_num_ = tmp;
  tmp = nh_.param("odomNode/detection/validLineNum", 5);
  valid_line_num_ = tmp;
  min_delta_Z_ = nh_.param("odomNode/detection/minDeltaZ", 0.1);
  max_delta_Z_ = nh_.param("odomNode/detection/maxDeltaZ", 3.0);
  max_distance_ = nh_.param("odomNode/detection/maxDistance", 20);
  max_elevation_ = nh_.param("odomNode/detection/maxElevation", 2.0);
  max_dim_ratio_ = nh_.param("odomNode/detection/maxDimRatio", 10.0);
  residuum_height_ratio_ = nh_.param("odomNode/detection/residuumHeightRatio", 0.0);
  sin_alpha_X_ = sin(ang_res_x_ / 180.0 * M_PI);
  cos_alpha_X_ = cos(ang_res_x_ / 180.0 * M_PI);
  sin_alpha_Y_ = sin(ang_res_y_ / 180.0 * M_PI);
  cos_alpha_Y_ = cos(ang_res_y_ / 180.0 * M_PI);

  dilate_kernel_size_ = nh_.param("odomNode/detection/dilateKernelSize", 0);
  evaluation_dir_ = nh_.param<std::string>("odomNode/evaluation/dir", "");
  config_path_ = nh_.param<std::string>("odomNode/evaluation/cfgPath", "");
  evaluate_ = nh_.param<bool>("odomNode/evaluation/evaluate", false);

  nan_point_.x = std::numeric_limits<float>::quiet_NaN();
  nan_point_.y = std::numeric_limits<float>::quiet_NaN();
  nan_point_.z = std::numeric_limits<float>::quiet_NaN();
  nan_point_.intensity = -1;

  // create color lookup table
  label_color_lut_ =
      std::vector<cv::Vec3b>{ cv::Vec3b(255, 0, 0),    cv::Vec3b(0, 255, 0),     cv::Vec3b(0, 0, 255),
                              cv::Vec3b(98, 227, 190), cv::Vec3b(244, 247, 74),  cv::Vec3b(168, 43, 217),
                              cv::Vec3b(235, 141, 9),  cv::Vec3b(4, 110, 10),    cv::Vec3b(174, 219, 90),
                              cv::Vec3b(31, 27, 89),   cv::Vec3b(224, 175, 197), cv::Vec3b(130, 71, 16) };
}

void DetectionModule::allocateMemory()
{
  std::pair<int8_t, int8_t> neighbor;
  neighbor.first = -1;
  neighbor.second = 0;
  neighbor_iterator_.push_back(neighbor);
  neighbor.first = 0;
  neighbor.second = 1;
  neighbor_iterator_.push_back(neighbor);
  neighbor.first = 0;
  neighbor.second = -1;
  neighbor_iterator_.push_back(neighbor);
  neighbor.first = 1;
  neighbor.second = 0;
  neighbor_iterator_.push_back(neighbor);

  cloud_in_.reset(new pcl::PointCloud<PointType>());
  cloud_in_t_.reset(new pcl::PointCloud<PointType>());
  full_cloud_.reset(new pcl::PointCloud<PointType>());
  full_cloud_->resize(H_ * W_);

  all_pushed_ind_X_.resize(H_ * W_);
  all_pushed_ind_Y_.resize(H_ * W_);

  queue_ind_Y_.resize(H_ * W_);
  queue_ind_X_.resize(H_ * W_);

  avg_residuals_.resize(H_ * W_);
}

void DetectionModule::resetParameters()
{
  label_count_ = 1;
  clusters_.clear();
  label_indices_i_.clear();
  label_indices_yx_.clear();
  cloud_in_->clear();
  cloud_in_t_->clear();

  // size_t H_img = std::min(H_, W_);
  // size_t W_img = std::max(H_, W_);
  label_mat_ = cv::Mat(H_, W_, CV_32S, cv::Scalar::all(0));
  ground_mat_ = cv::Mat(H_, W_, CV_8S, cv::Scalar::all(0));
  range_mat_ = cv::Mat(H_, W_, CV_32F, cv::Scalar::all(0));

  std::fill(full_cloud_->points.begin(), full_cloud_->points.end(), nan_point_);
};

void DetectionModule::applySegmentation()
{
  // Mark ground points
  groundRemoval();

  // Point cloud segmentation on range image
  cloudSegmentation();

  // Bounding Boxes
  computeAllObjects();

  // Tracking
  trackDetections();

  // Visualization
  visualize();

  // Evaluation
  if (evaluate_)
    evaluate();

  icp_residuals_set_ = false;
}

void DetectionModule::projectResiduals(const pcl::PointCloud<PointType>::Ptr& cloud_in)
{
  time_stats_["projectResiduals"].tick();

  // ROS_INFO("Residual cloud size: %zu", cloud_in->points.size());
  // ROS_INFO("Residual cloud H:%zu, W:%zu", H_, W_);

  residuals_mat_ = cv::Mat(H_, W_, CV_32F, cv::Scalar::all(0));

  int row, col, cloudSize;
  PointType thisPoint;

  // for (size_t i = 0; i < cloud_in->points.size(); ++i)
  // {
  //   double x = cloud_in->points[i].x;
  //   double y = cloud_in->points[i].y;
  //   double z = cloud_in->points[i].z;
  //   double intensity = cloud_in->points[i].intensity;  // the residuals are stored in the intensity channel

  //   // Vertical angle and row
  //   double v_angle = atan2(z, sqrt(x * x + y * y)) * 180 / M_PI;
  //   row = H_ - (v_angle + ang_bottom_) / ang_res_y_;
  //   if (row < 0 || row >= H_)
  //     continue;

  //   // Horizontal angle and column
  //   double h_angle = atan2(x, y) * 180 / M_PI - start_orientation_;
  //   col = round(h_angle / ang_res_x_);

  //   if (col >= W_)
  //     col -= W_;
  //   else if (col < 0)
  //     col += W_;

  //   residuals_mat_.at<float>(row, col) = intensity;
  // }
  for(row = 0; row < H_; ++row){
    for(col = 0; col < W_; ++col){
      size_t idx = row * W_ + col;
      const auto& pt = cloud_in->points[idx];
      if(!pcl::isFinite(pt)){
        continue;
      }
      residuals_mat_.at<float>(row, col) = static_cast<double>(pt.intensity);
    }
  }

  icp_residuals_set_ = true;
  time_stats_["projectResiduals"].tock();
}

void DetectionModule::projectScan(const pcl::PointCloud<PointType>::Ptr& cloud_in,
                                  const pcl::PointCloud<PointType>::Ptr& cloud_in_t, const Eigen::Matrix4f T,
                                  const Eigen::Matrix4f T_s2s)
{
  time_stats_["projectScan"].tick();

  cloud_header_ = cloud_in->header;
  T_ = T;
  T_s2s_ = T_s2s;

  pcl_conversions::fromPCL(cloud_in->header, header_);

  resetParameters();
  pcl::copyPointCloud(*cloud_in, *cloud_in_);
  pcl::copyPointCloud(*cloud_in_t, *cloud_in_t_);

  ROS_INFO("Cloud is organized: %d", cloud_in_t_->isOrganized());

  bool is_organized = cloud_in_t_->isOrganized() && is_organized_;  // TODO check this

  // find start orientation for aligning range image with residual image
  if (start_orientation_ == -9999)
  {
    for (size_t row = 0; row < H_; ++row)
    {
      size_t idx = row * W_;
      if ((cloud_in_->points[idx].x != 0 && cloud_in_->points[idx].x < 1e9) &&
          (cloud_in_->points[idx].y != 0 && cloud_in_->points[idx].x < 1e9))
      {
        start_orientation_ = atan2(cloud_in_->points[idx].x, cloud_in_->points[idx].y) * 180 / M_PI;
        ROS_INFO("Start angle: %f", start_orientation_);
        break;
      }
    }
    if (start_orientation_ == -9999)
      ROS_WARN("Could not find start angle in %d rings", H_);
  }

  // range image projection
  float range;
  int row, col;
  PointType origin;
  auto x0 = -T_.coeff(0, 3);
  auto y0 = -T_.coeff(1, 3);
  auto z0 = -T_.coeff(2, 3);

#pragma omp parallel for
  for (int row = 0; row < H_; row++)
  {
    for (int col = 0; col < W_; col++)
    {
        range_mat_.at<float>(row, col) = 0.0;
        // organized cloud なので、
        //  index = row * W_ + col
        size_t idx = row * W_ + col;
        const auto& pt = cloud_in_t_->points[idx];

        // 点が無効かどうかチェック (NaN等)
        if (!pcl::isFinite(pt)) {
            continue;
        }

        // カメラ原点（0, 0, 0）からの距離を計算 (Azure Kinectの点群は既にカメラ座標系のはず)
        float x = pt.x + x0;
        float y = pt.y + y0;
        float z = pt.z + z0;

        float range = sqrtf(x * x + y * y + z * z);
        if (range < minimum_range_)
            continue;

        // range_mat_ に書き込み
        range_mat_.at<float>(row, col) = range;
        full_cloud_->points[idx] = cloud_in_t_->points[idx];
    }
  }
  // for (size_t i = 0; i < cloud_in_t_->size(); ++i)
  // {
  //   // Get distance from sensor
  //   auto x = cloud_in_t_->points[i].x + x0;
  //   auto y = cloud_in_t_->points[i].y + y0;
  //   auto z = cloud_in_t_->points[i].z + z0;
  //   if (is_organized)
  //   {
  //     row = (int)(i / W_);
  //     col = i % W_;
  //   }
  //   else
  //   {
  //     // Vertical angle
  //     auto vertical_angle = atan2(z, sqrt(x * x + y * y)) * 180 / M_PI;
  //     row = H_ - (vertical_angle + ang_bottom_) / ang_res_y_;
  //     if (row < 0 || row >= H_)
  //       row = H_ - 1;

  //     // Horizontal angle
  //     auto horizon_angle = atan2(x, y) * 180 / M_PI;
  //     col = round(horizon_angle / ang_res_x_);

  //     if (col >= W_)
  //       col -= W_;
  //     else if (col < 0)
  //       col += W_;
  //   }

  //   range = sqrt(x * x + y * y + z * z);

  //   // skip points that are too close or invalid
  //   if (range < minimum_range_)
  //     continue;

  //   range_mat_.at<float>(row, col) = range;
  //   full_cloud_->points[i] = cloud_in_t_->points[i];
  // }

  // cv::Mat range_mat_scaled;
  // float clip_range = 15.0;
  // range_mat_.convertTo(range_mat_scaled, CV_8UC1, 255.0 / clip_range);
  // range_mat_scaled = 255 - range_mat_scaled;  // invert colors

  // cv_bridge::CvImage img_bridge;
  // sensor_msgs::Image img_msg;
  // img_bridge = cv_bridge::CvImage(header_, sensor_msgs::image_encodings::TYPE_8UC1, range_mat_scaled);
  // img_bridge.toImageMsg(img_msg);
  // range_img_pub_.publish(img_msg);

  time_stats_["projectScan"].tock();
  ROS_INFO("projectScan done");
}

// void DetectionModule::projectScan(const pcl::PointCloud<PointType>::Ptr& cloud_in,
//                                     const pcl::PointCloud<PointType>::Ptr& cloud_in_t, const Eigen::Matrix4f T,
//                                     const Eigen::Matrix4f T_s2s)  // 使う座標変換がある場合だけ受け取る
// {
//   // （1）前処理・変数セットアップ
//   time_stats_["projectScan"].tick();

//   // 例：Azure Kinect 点群は organized 前提
//   //     幅 W_, 高さ H_ を cloud_in->width, cloud_in->height として設定
//   if (!cloud_in->isOrganized())
//   {
//       ROS_WARN("Input cloud is not organized. Skip.");
//       return;
//   }

//   H_ = cloud_in->height;
//   W_ = cloud_in->width;

//   ROS_INFO("Cloud Height: %d, Width: %d", H_, W_);
//   ROS_INFO("Cloud size: %zu", cloud_in->points.size());
//   ROS_INFO("Cloud HxW: %zu", H_ * W_);

//   // 必要なら transform
//   // pcl::PointCloud<PointType>::Ptr cloud_transformed(new pcl::PointCloud<PointType>);
//   // pcl::transformPointCloud(*cloud_in, *cloud_transformed, T);
//   // ※本当に座標を変換したい場合のみ。カメラ座標系のままでOKなら不要

//   // （2）レンジ画像用の行列を初期化
//   range_mat_ = cv::Mat(H_, W_, CV_32F, cv::Scalar::all(0));

//   // （3）organizedな点群を走査し、レンジ(深度)を計算して書き込む
//   //      ここでは transformCloud を使わない例とする
//   for (int row = 0; row < H_; row++)
//   {
//       for (int col = 0; col < W_; col++)
//       {
//           // organized cloud なので、
//           //  index = row * W_ + col
//           size_t idx = row * W_ + col;
//           const auto& pt = cloud_in->points[idx];

//           // 点が無効かどうかチェック (NaN等)
//           if (!pcl::isFinite(pt)) {
//               continue;
//           }

//           // カメラ原点（0, 0, 0）からの距離を計算 (Azure Kinectの点群は既にカメラ座標系のはず)
//           float x = pt.x;
//           float y = pt.y;
//           float z = pt.z;

//           float range = sqrtf(x * x + y * y + z * z);
//           if (range < minimum_range_)
//               continue;

//           // range_mat_ に書き込み
//           range_mat_.at<float>(row, col) = range;
//       }
//   }

//   time_stats_["projectScan"].tock();
// }


void DetectionModule::groundRemoval()
{
  time_stats_["groundRemoval"].tick();
  size_t lower_ind, upper_ind, count = 0;
  float diffX, diffY, diffZ, angle;
  // groundMat
  // -1, no valid info to check if ground of not
  //  0, initial value, after validation, means not ground
  //  1, ground

#pragma omp parallel for
  for (size_t col = 0; col < W_; ++col)
  {
    for (size_t row_inverse = 0; row_inverse < ground_rows_; ++row_inverse)
    {
      size_t row = H_ - 1 - row_inverse;  // inverse looping because we start from the bottom

      lower_ind = col + (row)*W_;
      upper_ind = col + (row - 1)*W_;
      if (lower_ind < 0 || upper_ind >= full_cloud_->size())
        ROS_ERROR("LOWER UPPER %zu %zu", lower_ind, upper_ind);

      if (full_cloud_->points[lower_ind].x == 0 || full_cloud_->points[upper_ind].x == 0)
      {
        // no info to check, invalid points
        ground_mat_.at<int8_t>(row, col) = -1;
        continue;
      }

      diffX = full_cloud_->points[upper_ind].x - full_cloud_->points[lower_ind].x;
      diffY = full_cloud_->points[upper_ind].y - full_cloud_->points[lower_ind].y;
      diffZ = full_cloud_->points[upper_ind].z - full_cloud_->points[lower_ind].z;

      angle = atan2(diffZ, sqrt(diffX * diffX + diffY * diffY)) * 180 / M_PI;

      // mark this pixel and the one below as ground
      if (abs(angle - sensor_mount_angle_) <= ground_angle_threshold_)
      {
        ground_mat_.at<int8_t>(row, col) = 1;
        ground_mat_.at<int8_t>(row - 1, col) = 1;
        count++;
      }
    }
  }

  // Mark all ground points and invalid points in label matrix so they can be skipped during labeling
  for (size_t i = 0; i < H_; ++i)
  {
    size_t row = H_ - 1 - i;
    for (size_t col = 0; col < W_; ++col)
    {
      if (ground_mat_.at<int8_t>(row, col) == 1 || range_mat_.at<float>(row, col) == 0)
      {
        label_mat_.at<int>(row, col) = -1;
      }
    }
  }
  time_stats_["groundRemoval"].tock();

  ROS_INFO("GroundSegmentation ok");
}

void DetectionModule::cloudSegmentation()
{
  time_stats_["cloudSegmentation"].tick();

  // segmentation process
  for (size_t i = 0; i < H_; ++i)
    for (size_t j = 0; j < W_; ++j)
      if (label_mat_.at<int>(i, j) == 0)
        labelComponents(i, j);

  // Store label indices for later usage  TODO move this to labelComponents(?)
  label_indices_i_.resize(label_count_);
  label_indices_yx_.resize(label_count_);
  for (size_t i = 0; i < H_; ++i)
  {
    for (size_t j = 0; j < W_; ++j)
    {
      int label = label_mat_.at<int>(i, j);
      if (label > 0 && label != 999999)
      {
        label_indices_i_[label].push_back(i * W_ + j);
        label_indices_yx_[label].push_back(cv::Vec3b(i, j));
      }
    }
  }

  time_stats_["cloudSegmentation"].tock();
  ROS_INFO("CloudSegmentation ok");
}

void DetectionModule::labelComponents(int row, int col)
{
  // use std::queue std::vector std::deque will slow the program down greatly
  float d1, d2, angle, min_z, max_z, min_dist, max_dist, total_residuum;
  int from_ind_Y, from_ind_X, this_ind_Y, this_ind_X, res_count;
  bool line_count_flag[H_];  // = { false };
  for (size_t i = 0; i < H_; i++)
    line_count_flag[i] = false;

  queue_ind_Y_[0] = row;
  queue_ind_X_[0] = col;
  int queue_size = 1;
  int queue_start_ind = 0;
  int queue_end_ind = 1;
  min_z = 1e6;
  max_z = -1e6;
  min_dist = 1e6;
  max_dist = -1e6;
  res_count = 0;
  total_residuum = 0;

  all_pushed_ind_X_[0] = row;
  all_pushed_ind_Y_[0] = col;
  int all_pushed_ind_size = 1;

  while (queue_size > 0)
  {
    // Pop point
    from_ind_Y = queue_ind_Y_[queue_start_ind];
    from_ind_X = queue_ind_X_[queue_start_ind];
    --queue_size;
    ++queue_start_ind;

    // Mark popped point
    label_mat_.at<int>(from_ind_Y, from_ind_X) = label_count_;
    // Loop through all the neighboring grids of popped grid
    for (auto iter = neighbor_iterator_.begin(); iter != neighbor_iterator_.end(); ++iter)
    {
      // get new index
      this_ind_Y = from_ind_Y + (*iter).first;
      this_ind_X = from_ind_X + (*iter).second;

      // skip above top scan line / below bottom scan line
      if (this_ind_Y < 0 || this_ind_Y >= H_)
        continue;

      // warp at image borders
      if (this_ind_X < 0)
        this_ind_X = W_ - 1;
      if (this_ind_X >= W_)
        this_ind_X = 0;

      // prevent infinite loop (caused by put already examined point back)
      if (label_mat_.at<int>(this_ind_Y, this_ind_X) != 0)
        continue;

      d1 = std::max(range_mat_.at<float>(from_ind_Y, from_ind_X), range_mat_.at<float>(this_ind_Y, this_ind_X));
      d2 = std::min(range_mat_.at<float>(from_ind_Y, from_ind_X), range_mat_.at<float>(this_ind_Y, this_ind_X));

      // choose vertical / horizontal resolution depending on the neighbor we are checking
      float sin_alpha, cos_alpha;
      if ((*iter).first == 0)
      {
        sin_alpha = sin_alpha_X_;
        cos_alpha = cos_alpha_X_;
      }
      else
      {
        sin_alpha = sin_alpha_Y_;
        cos_alpha = cos_alpha_Y_;
      }

      // the actual threshold computation as described in the paper
      angle = atan2(d2 * sin_alpha, (d1 - d2 * cos_alpha));

      if (angle > theta_)
      {
        // update min/max z value and min_dist of current segment
        double z = cloud_in_t_->points[this_ind_X + this_ind_Y * W_].z;
        if (z < min_z && z != 0)  // z==0 means invalid point
          min_z = z;
        else if (z > max_z)
          max_z = z;
        min_dist = std::min(min_dist, std::min(d1, d2));
        max_dist = std::max(max_dist, std::max(d1, d2));

        queue_ind_Y_[queue_end_ind] = this_ind_Y;
        queue_ind_X_[queue_end_ind] = this_ind_X;
        ++queue_size;
        ++queue_end_ind;

        label_mat_.at<int>(this_ind_Y, this_ind_X) = label_count_;

        line_count_flag[this_ind_Y] = true;

        all_pushed_ind_X_[all_pushed_ind_size] = this_ind_Y;
        all_pushed_ind_Y_[all_pushed_ind_size] = this_ind_X;
        ++all_pushed_ind_size;

        if (residuals_mat_.at<float>(this_ind_Y, this_ind_X) > 0)
        {
          total_residuum += residuals_mat_.at<float>(this_ind_Y, this_ind_X);
          res_count++;
        }
      }
    }
  }

  // check if this segment is valid
  bool feasible_segment = false;

  // get line count
  int line_count = 0;
  for (size_t i = 0; i < H_; ++i)
    if (line_count_flag[i])
      ++line_count;

  // segments with at least 50 points are always considered feasible
  if (all_pushed_ind_size >= 50 && line_count >= min_line_num_)
    feasible_segment = true;

  // smaller segments need to have the given min size and span over min number of lines
  else if (all_pushed_ind_size >= valid_point_num_)
    if (line_count >= valid_line_num_)
      feasible_segment = true;

  // skip segments that are too far away from robot
  if (feasible_segment)
    feasible_segment = max_dist <= max_distance_;

  // segment should not be too flat or too tall - skip those with delta z < threshold
  float delta_z;
  if (feasible_segment)
  {
    delta_z = max_z - min_z;
    feasible_segment = min_delta_Z_ <= delta_z && delta_z <= max_delta_Z_;
  }

  // minimal average residuum
  double avg_residuum;
  if (feasible_segment && icp_residuals_set_)
    avg_residuum = res_count > 0 ? total_residuum / res_count : 0;

  // skip segments that fly around (belonging to trees and stuff)
  if (feasible_segment)
  {
    float current_height = T_.coeff(2, 3);  // height of sensor in global frame
    feasible_segment = min_z - current_height <= max_elevation_;
  }

  // segment is valid, mark these points
  if (feasible_segment)
  {
    if (icp_residuals_set_)
    {
      avg_residuals_[label_count_] = avg_residuum;
    }
    else
    {
      ROS_WARN("[Detection] You need to call \"projectResiduals\" before labelComponents");
      avg_residuals_[label_count_] = 0;
    }
    ++label_count_;
  }
  else
  {
    // segment is invalid, mark these points
    for (size_t i = 0; i < all_pushed_ind_size; ++i)
    {
      label_mat_.at<int>(all_pushed_ind_X_[i], all_pushed_ind_Y_[i]) = 999999;
    }
  }
  // ROS_INFO("LabelComponents ok");
}

Object DetectionModule::getObject(const pcl::PointCloud<PointType>& cloud)
{
  // Minimal Bounding Box
  PointType orig_min_point, orig_max_point;
  pcl::getMinMax3D(cloud, orig_min_point, orig_max_point);

  pcl::PointCloud<PointType>::Ptr cloud_transformed(new pcl::PointCloud<PointType>);
  pcl::copyPointCloud<PointType>(cloud, *cloud_transformed);

  // Set z components zero
  for (size_t idx = 0u; idx < cloud_transformed->size(); ++idx)
    cloud_transformed->points[idx].z = 0;

  // Compute principal directions
  Eigen::Vector4f pca_centroid;
  pcl::compute3DCentroid(*cloud_transformed, pca_centroid);
  Eigen::Matrix3f covariance;
  computeCovarianceMatrixNormalized(*cloud_transformed, pca_centroid, covariance);
  Eigen::Matrix2f small_covariance = covariance.block(0, 0, 2, 2);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eigen_solver(small_covariance, Eigen::ComputeEigenvectors);
  Eigen::Matrix2f eigen_vectors_PCA = eigen_solver.eigenvectors();

  // Transform the original cloud to the origin where the principal components correspond to the axes.
  Eigen::Matrix4f projection_transform(Eigen::Matrix4f::Identity());
  projection_transform.block<2, 2>(0, 0) = eigen_vectors_PCA.transpose();
  projection_transform.block<2, 1>(0, 3) = -1.f * (projection_transform.block<2, 2>(0, 0) * pca_centroid.head<2>());
  pcl::PointCloud<PointType>::Ptr cloud_points_projected(new pcl::PointCloud<PointType>);
  pcl::transformPointCloud(*cloud_transformed, *cloud_points_projected, projection_transform);

  // Get the minimum and maximum points of the transformed cloud.
  PointType min_point, max_point;
  pcl::getMinMax3D(*cloud_points_projected, min_point, max_point);
  pcl::PointXYZ min_point_XYZ, max_point_XYZ;
  min_point_XYZ.x = min_point.x;
  min_point_XYZ.y = min_point.y;
  max_point_XYZ.x = max_point.x;
  max_point_XYZ.y = max_point.y;

  const Eigen::Vector2f mean_XY =
      0.5f * (max_point_XYZ.getVector3fMap().head<2>() + min_point_XYZ.getVector3fMap().head<2>());
  const float mean_Z = 0.5f * (orig_max_point.z + orig_min_point.z);

  Eigen::Vector2f bbox_transform = eigen_vectors_PCA * mean_XY + pca_centroid.head<2>();
  Eigen::Vector3f position(bbox_transform[0], bbox_transform[1], mean_Z);
  float orientation = std::atan2(eigen_vectors_PCA.col(0)[1], eigen_vectors_PCA.col(0)[0]);
  Eigen::Vector3f dimension(max_point.x - min_point.x, max_point.y - min_point.y, orig_max_point.z - orig_min_point.z);

  Object obj;
  obj.state << bbox_transform[0], bbox_transform[1], mean_Z, std::sin(orientation / 2.0), dimension[0], dimension[1],
      dimension[2];

  obj.num_points = cloud.size();
  float volume = dimension[0] * dimension[1] * dimension[2];
  obj.density = cloud.size() / volume;

  return obj;
}

void DetectionModule::computeAllObjects()
{
  time_stats_["computeAllObjects"].tick();

  detected_objects_.clear();
  //  detected_objects_.reserve(label_indices_i.size());

  // extract clusters

  for (int label = 1; label < label_indices_i_.size(); label++)
  {
    pcl::PointCloud<PointType> tmp_cloud;
    pcl::copyPointCloud(*cloud_in_t_, label_indices_i_[label], tmp_cloud);

    Object obj = getObject(tmp_cloud);

    // One more feasibility check: bbox dimension ratio, largest vs. 2nd largest dimension
    std::vector<double> dimensions({ obj.state[4], obj.state[5], obj.state[6] });
    std::sort(dimensions.begin(), dimensions.end());  // smallest to largest
    if (dimensions[2] / dimensions[1] >= max_dim_ratio_)
      continue;

    obj.id = label;
    obj.indices = label_indices_i_[label];
    obj.avg_residuum = avg_residuals_[label];

    // std::cout << "Object vector: " << obj.state.transpose() << std::endl;

    detected_objects_.push_back(obj);
  }

  time_stats_["computeAllObjects"].tock();

  ROS_INFO("ComputeAllObjects ok");
}

void DetectionModule::trackDetections()
{
  time_stats_["trackDetections"].tick();

  ros::Duration dt = header_.stamp - time_prev_;
  time_prev_ = header_.stamp;

  // Update the tracking module
  tracker_.update(dt.toSec(), detected_objects_, header_);

  time_stats_["trackDetections"].tock();
  ROS_INFO("TrackDetections ok");
}

void DetectionModule::visualize()
{
  if (range_img_pub_.getNumSubscribers() > 0)
  {
    cv::Mat range_mat_scaled;
    float clip_range = 15.0;
    range_mat_.convertTo(range_mat_scaled, CV_8UC1, 255.0 / clip_range);
    range_mat_scaled = 255 - range_mat_scaled;  // invert colors

    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image img_msg;
    img_bridge = cv_bridge::CvImage(header_, sensor_msgs::image_encodings::TYPE_8UC1, range_mat_scaled);
    img_bridge.toImageMsg(img_msg);
    range_img_pub_.publish(img_msg);
  }

  if (residual_img_pub_.getNumSubscribers() > 0)
  {
    cv::Mat residuals_mat_scaled;
    residuals_mat_.convertTo(residuals_mat_scaled, CV_8UC1, 255.0);
    // Dilate the image for better visibility of single pixels
    cv::Mat kernel = cv::Mat::ones(dilate_kernel_size_, dilate_kernel_size_, CV_8U);
    cv::dilate(residuals_mat_scaled, residuals_mat_scaled, kernel);
    residuals_mat_scaled = 255 - residuals_mat_scaled;  // invert colors

    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image img_msg;
    img_bridge = cv_bridge::CvImage(header_, sensor_msgs::image_encodings::TYPE_8UC1, residuals_mat_scaled);
    img_bridge.toImageMsg(img_msg);
    residual_img_pub_.publish(img_msg);
  }

  if (label_img_pub_.getNumSubscribers() > 0)
  {
    cv::Mat label_mat_vis(label_mat_.rows, label_mat_.cols, CV_8UC3, cv::Scalar(0, 0, 0));

    for (size_t y = 0; y < label_mat_vis.rows; y++)
    {
      for (size_t x = 0; x < label_mat_vis.cols; x++)
      {
        int label = label_mat_.at<int>(y, x);
        if (label == -1)
        {
          if (ground_mat_.at<int8_t>(y, x) != 0)
          {
            // ground: black
            label_mat_vis.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);
          }
          else
          {
            // unlabeled: white (invalid measure or < min range)
            label_mat_vis.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255);
          }
        }
        else if (label == 999999)
        {
          // infeasible segments: gray
          label_mat_vis.at<cv::Vec3b>(y, x) = cv::Vec3b(150, 150, 150);
        }
        else
        {
          // feasible segments: color from lut
          label_mat_vis.at<cv::Vec3b>(y, x) = label_color_lut_[label % label_color_lut_.size()];
        }
      }
    }

    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image img_msg;
    img_bridge = cv_bridge::CvImage(header_, sensor_msgs::image_encodings::TYPE_8UC3, label_mat_vis);
    img_bridge.toImageMsg(img_msg);
    label_img_pub_.publish(img_msg);
  }

  ROS_INFO("visualize ok");
}

void DetectionModule::setupEvaluation()
{
  std::stringstream timestamp;
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);
  timestamp << std::put_time(&tm, "%Y_%m_%d-%H_%M_%S");
  output_dir_ = evaluation_dir_ + "/" + timestamp.str();

  std::filesystem::create_directories(output_dir_);
  ROS_INFO("Created evaluation output directory %s", output_dir_.c_str());

  std::string dest_path = output_dir_ + "/cfg.yaml";

  std::ifstream source_file(config_path_, std::ios::binary);
  std::ofstream dest_file(dest_path, std::ios::binary);
  if (!source_file.is_open() || !dest_file.is_open())
  {
    ROS_ERROR("Could not open %s or %s", config_path_.c_str(), dest_path.c_str());
    return;
  }

  dest_file << source_file.rdbuf();
  ROS_INFO("Copied %s to %s", config_path_.c_str(), dest_path.c_str());
}

void DetectionModule::evaluate()
{
  std::stringstream seq_ss;
  seq_ss << std::setw(4) << std::setfill('0') << cloud_header_.seq;
  std::string filename = output_dir_ + "/" + seq_ss.str() + ".txt";
  std::ofstream indices_file(filename, std::ios::app);

  std::vector<int> idcs;
  getIndices(idcs, ObjectStatus::DYNAMIC);
  for (int i : idcs)
  {
    indices_file << i << "\n";
  }

  indices_file.close();

  std::ofstream pose_file(output_dir_ + "/poses.txt", std::ios::app);
  pose_file << header_.stamp.toNSec() << std::endl << T_ << ";" << std::endl;
}

double DetectionModule::getTime(std::string name, std::string type = "mean")
{
  if (type == "mean")
    return time_stats_[name].getMean();
  else if (type == "var")
    return time_stats_[name].getVariance();
  else if (type == "min")
    return time_stats_[name].getMin();
  else if (type == "max")
    return time_stats_[name].getMax();
  else if (type == "last")
    return time_stats_[name].getLast();
  else
    return -1;
}

size_t DetectionModule::getSegmentsCount()
{
  return label_count_;
}

size_t DetectionModule::getObjectsCount()
{
  return tracker_.getObjectsCount();
}

size_t DetectionModule::getObjectsCount(const std::vector<ObjectStatus> object_status)
{
  return tracker_.getObjectsCount(object_status);
}

size_t DetectionModule::getObjectsCount(const ObjectStatus object_status)
{
  return tracker_.getObjectsCount(object_status);
}

void DetectionModule::getIndices(std::vector<int>& indices, const std::vector<ObjectStatus> object_status)
{
  return tracker_.getIndices(indices, object_status);
}

void DetectionModule::getIndices(std::vector<int>& indices, const ObjectStatus object_status)
{
  return tracker_.getIndices(indices, object_status);
}

void DetectionModule::getGroundIndices(std::vector<int>& ground_indices)
{
  for (size_t col = 0; col < W_; ++col)
  {
    for (size_t row_inverse = 0; row_inverse < ground_rows_; ++row_inverse)
    {
      size_t row = H_ - 1 - row_inverse;
      if (ground_mat_.at<int8_t>(row, col) == 1)
        ground_indices.push_back(row * W_ + col);
    }
  }
}
