// MIT License

// Copyright (c) 2022 Kenny J. Chen, Brett T. Lopez

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "odometry/odom.h"

std::atomic<bool> OdomNode::abort_(false);

OdomNode::OdomNode()
{
  getParams();

  stop_publish_thread_ = false;
  stop_publish_keyframe_thread_ = false;
  stop_metrics_thread_ = false;
  stop_debug_thread_ = false;

  ddlo_initialized_ = false;
  imu_calibrated_ = false;

  icp_sub_ = nh_.subscribe("pointcloud", 10, &OdomNode::icpCB, this);
  imu_sub_ = nh_.subscribe("imu", 1, &OdomNode::imuCB, this);
  map_info_sub_ = nh_.subscribe("map_info", 1, &OdomNode::mapCB, this);

  path_pub_ = nh_.advertise<nav_msgs::Path>("path", 1);
  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("pose", 1);
  kf_pub_ = nh_.advertise<nav_msgs::Odometry>("kfs", 1, true);
  keyframe_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("keyframe", 1, true);
  submap_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("active_submap", 1, true);
  dynamic_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("dynamic_points", 1);
  non_static_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("non_static_points", 1);
  static_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("static_points", 1);
  residuals_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("residuals", 1);
  ground_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("ground_points", 1);

  origin_ = Eigen::Vector3f(0., 0., 0.);

  T_ = Eigen::Matrix4f::Identity();
  T_s2s_ = Eigen::Matrix4f::Identity();
  T_s2s_prev_ = Eigen::Matrix4f::Identity();

  pose_s2s_ = Eigen::Vector3f(0., 0., 0.);
  rotq_s2s_ = Eigen::Quaternionf(1., 0., 0., 0.);

  pose_ = Eigen::Vector3f(0., 0., 0.);
  rotq_ = Eigen::Quaternionf(1., 0., 0., 0.);

  imu_SE3_ = Eigen::Matrix4f::Identity();

  registration_scan_ = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
  registration_scan_t_ = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
  segmentation_scan_ = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
  segmentation_scan_t_ = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
  residuals_cloud_ = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
  non_static_cloud_ = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
  static_cloud_ = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);

  keyframe_cloud_ = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
  keyframes_cloud_ = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
  num_keyframes_ = 0;

  submap_cloud_ = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
  submap_hasChanged_ = true;
  submap_kf_idx_prev_.clear();

  source_cloud_ = nullptr;
  target_cloud_ = nullptr;

  convex_hull_.setDimension(3);
  concave_hull_.setDimension(3);
  concave_hull_.setAlpha(keyframe_thresh_dist_);
  concave_hull_.setKeepInformation(true);

  gicp_s2s_.setCorrespondenceRandomness(gicps2s_k_correspondences_);
  gicp_s2s_.setMaxCorrespondenceDistance(gicps2s_max_corr_dist_);
  gicp_s2s_.setMaximumIterations(gicps2s_max_iter_);
  gicp_s2s_.setTransformationEpsilon(gicps2s_transformation_ep_);
  gicp_s2s_.setEuclideanFitnessEpsilon(gicps2s_euclidean_fitness_ep_);
  gicp_s2s_.setRANSACIterations(gicps2s_ransac_iter_);
  gicp_s2s_.setRANSACOutlierRejectionThreshold(gicps2s_ransac_inlier_thresh_);

  gicp_s2m_.setCorrespondenceRandomness(gicps2m_k_correspondences_);
  gicp_s2m_.setMaxCorrespondenceDistance(gicps2m_max_corr_dist_);
  gicp_s2m_.setMaximumIterations(gicps2m_max_iter_);
  gicp_s2m_.setTransformationEpsilon(gicps2m_transformation_ep_);
  gicp_s2m_.setEuclideanFitnessEpsilon(gicps2m_euclidean_fitness_ep_);
  gicp_s2m_.setRANSACIterations(gicps2m_ransac_iter_);
  gicp_s2m_.setRANSACOutlierRejectionThreshold(gicps2m_ransac_inlier_thresh_);

  pcl::Registration<PointType, PointType>::KdTreeReciprocalPtr temp;
  gicp_s2s_.setSearchMethodSource(temp, true);
  gicp_s2s_.setSearchMethodTarget(temp, true);
  gicp_s2m_.setSearchMethodSource(temp, true);
  gicp_s2m_.setSearchMethodTarget(temp, true);

  crop1_.setNegative(true);
  crop1_.setMin(Eigen::Vector4f(-crop_size_, -crop_size_, -crop_size_, 1.0));
  crop1_.setMax(Eigen::Vector4f(crop_size_, crop_size_, crop_size_, 1.0));
  crop2_.setNegative(true);
  crop2_.setMin(Eigen::Vector4f(-crop_size_, -crop_size_, -crop_size_, 1.0));
  crop2_.setMax(Eigen::Vector4f(crop_size_, crop_size_, crop_size_, 1.0));

  vf_scan_.setLeafSize(vf_scan_res_, vf_scan_res_, vf_scan_res_);
  vf_submap_.setLeafSize(vf_submap_res_, vf_submap_res_, vf_submap_res_);

  // Create filter mask
  downsample_filter_indices_ = pcl::IndicesPtr(new pcl::Indices);
  downsample_filter_.setNegative(false);
  downsample_filter_.setIndices(downsample_filter_indices_);
  for (size_t row = 0; row < cloud_height_; row += downsample_filter_row_)
    for (size_t col = 0; col < cloud_width_; col += downsample_filter_col_)
      downsample_filter_indices_->push_back(row * cloud_width_ + col);

  metrics_.spaciousness.push_back(0.);

  current_seq_ = 0;
  first_seq_ = 0;
  scan_count_ = 0;

  // CPU Specs
  char CPUBrandString[0x40];
  memset(CPUBrandString, 0, sizeof(CPUBrandString));
  cpu_type_ = "";

#ifdef HAS_CPUID
  unsigned int CPUInfo[4] = { 0, 0, 0, 0 };
  __cpuid(0x80000000, CPUInfo[0], CPUInfo[1], CPUInfo[2], CPUInfo[3]);
  unsigned int nExIds = CPUInfo[0];
  for (unsigned int i = 0x80000000; i <= nExIds; ++i)
  {
    __cpuid(i, CPUInfo[0], CPUInfo[1], CPUInfo[2], CPUInfo[3]);
    if (i == 0x80000002)
      memcpy(CPUBrandString, CPUInfo, sizeof(CPUInfo));
    else if (i == 0x80000003)
      memcpy(CPUBrandString + 16, CPUInfo, sizeof(CPUInfo));
    else if (i == 0x80000004)
      memcpy(CPUBrandString + 32, CPUInfo, sizeof(CPUInfo));
  }

  cpu_type_ = CPUBrandString;
  boost::trim(cpu_type_);
#endif

  FILE* file;
  struct tms timeSample;
  char line[128];

  lastCPU_ = times(&timeSample);
  lastSysCPU_ = timeSample.tms_stime;
  lastUserCPU_ = timeSample.tms_utime;

  file = fopen("/proc/cpuinfo", "r");
  numProcessors_ = 0;
  while (fgets(line, 128, file) != NULL)
  {
    if (strncmp(line, "processor", 9) == 0)
      numProcessors_++;
  }
  fclose(file);

  // Statistics setup
  time_stats_.insert(std::make_pair("total", AccumulatorData()));
  time_stats_.insert(std::make_pair("odometry", AccumulatorData()));
  time_stats_.insert(std::make_pair("dynamic", AccumulatorData()));

  ROS_INFO("DLO Odom Node Initialized");
}

void OdomNode::getParams()
{
  // Global settings
  print_status_ = nh_.param("printStatus", true);
  dynamic_detection_ = nh_.param("dynamicDetection", true);
  gravity_align_ = nh_.param("gravityAlign", false);
  odom_frame_ = nh_.param<std::string>("odomFrame", "odom");
  child_frame_ = nh_.param<std::string>("childFrame", "base_link");

  // Keyframe Threshold
  keyframe_thresh_dist_ = nh_.param("odomNode/keyframe/threshD", 0.1);
  keyframe_thresh_rot_ = nh_.param("odomNode/keyframe/threshR", 1.0);

  // Submap
  submap_knn_ = nh_.param("odomNode/submap/keyframe/knn", 10);
  submap_kcv_ = nh_.param("odomNode/submap/keyframe/kcv", 10);
  submap_kcc_ = nh_.param("odomNode/submap/keyframe/kcc", 10);

  // Crop Box Filter
  crop_use_ = nh_.param("odomNode/preprocessing/cropBoxFilter/use", false);
  crop_size_ = nh_.param("odomNode/preprocessing/cropBoxFilter/size", 1.0);

  // downsample filter
  cloud_height_ = nh_.param("odomNode/detection/rows", 128);
  cloud_width_ = nh_.param("odomNode/detection/columns", 1024);
  downsample_filter_use_ = nh_.param("odomNode/preprocessing/downsampling/use", false);
  downsample_filter_row_ = nh_.param("odomNode/preprocessing/downsampling/row", 1);
  downsample_filter_col_ = nh_.param("odomNode/preprocessing/downsampling/col", 1);

  // Voxel Grid Filter
  vf_scan_use_ = nh_.param("odomNode/preprocessing/voxelFilter/scan/use", false);
  vf_scan_res_ = nh_.param("odomNode/preprocessing/voxelFilter/scan/res", 0.05);
  vf_submap_use_ = nh_.param("odomNode/preprocessing/voxelFilter/submap/use", false);
  vf_submap_res_ = nh_.param("odomNode/preprocessing/voxelFilter/submap/res", 0.1);

  // GICP
  gicp_min_num_points_ = nh_.param("odomNode/gicp/minNumPoints", 100);
  gicps2s_k_correspondences_ = nh_.param("odomNode/gicp/s2s/kCorrespondences", 20);
  gicps2s_max_corr_dist_ = nh_.param("odomNode/gicp/s2s/maxCorrespondenceDistance",
                                     std::sqrt(std::numeric_limits<double>::max()));  // ???
  gicps2s_max_iter_ = nh_.param("odomNode/gicp/s2s/maxIterations", 64);
  gicps2s_transformation_ep_ = nh_.param("odomNode/gicp/s2s/transformationEpsilon", 0.0005);
  gicps2s_euclidean_fitness_ep_ =
      nh_.param("odomNode/gicp/s2s/euclideanFitnessEpsilon", -std::numeric_limits<double>::max());
  gicps2s_ransac_iter_ = nh_.param("odomNode/gicp/s2s/ransac/iterations", 0);
  gicps2s_ransac_inlier_thresh_ = nh_.param("odomNode/gicp/s2s/ransac/outlierRejectionThresh", 0.05);
  gicps2m_k_correspondences_ = nh_.param("odomNode/gicp/s2m/kCorrespondences", 20);
  gicps2m_max_corr_dist_ = nh_.param("odomNode/gicp/s2m/maxCorrespondenceDistance",
                                     std::sqrt(std::numeric_limits<double>::max()));  //???
  gicps2m_max_iter_ = nh_.param("odomNode/gicp/s2m/maxIterations", 64);
  gicps2m_transformation_ep_ = nh_.param("odomNode/gicp/s2m/transformationEpsilon", 0.0005);

  gicps2m_euclidean_fitness_ep_ =
      nh_.param("odomNode/gicp/s2m/euclideanFitnessEpsilon", -std::numeric_limits<double>::max());
  gicps2m_ransac_iter_ = nh_.param("odomNode/gicp/s2m/ransac/iterations", 0);
  gicps2m_ransac_inlier_thresh_ = nh_.param("odomNode/gicp/s2m/ransac/outlierRejectionThresh", 0.05);
}

void OdomNode::start()
{
  ROS_INFO("Starting DLO Odometry Node");

  printf("\033[2J\033[1;1H");
  std::cout << std::endl << "==== Direct LiDAR Odometry ====" << std::endl << std::endl;
}

void OdomNode::stop()
{
  ROS_WARN("Stopping DLO Odometry Node");

  stop_publish_thread_ = true;
  if (publish_thread_.joinable())
    publish_thread_.join();

  stop_publish_keyframe_thread_ = true;
  if (publish_keyframe_thread_.joinable())

    publish_keyframe_thread_.join();

  stop_metrics_thread_ = true;
  if (metrics_thread_.joinable())

    metrics_thread_.join();

  stop_debug_thread_ = true;
  if (debug_thread_.joinable())

    debug_thread_.join();

  ros::shutdown();
}

void OdomNode::abortTimerCB(const ros::TimerEvent& e)
{
  if (abort_)
    stop();
}

void OdomNode::publishToROS()
{
  publishPath();
  publishTransform();
  publishSubmap();
  publishPoints();
}

void OdomNode::publishPoints()
{
  if (dynamic_points_pub_.getNumSubscribers() > 0)
  {
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*dynamic_cloud_, msg);
    msg.header.frame_id = odom_frame_;
    msg.header.stamp = scan_stamp_;
    dynamic_points_pub_.publish(msg);
  }
  if (non_static_points_pub_.getNumSubscribers() > 0)
  {
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*non_static_cloud_, msg);
    msg.header.frame_id = odom_frame_;
    msg.header.stamp = scan_stamp_;
    non_static_points_pub_.publish(msg);
  }
  if (residuals_pub_.getNumSubscribers() > 0)
  {
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*residuals_cloud_, msg);
    msg.header.frame_id = child_frame_;
    msg.header.stamp = scan_stamp_;
    residuals_pub_.publish(msg);
  }
  if (static_points_pub_.getNumSubscribers() > 0)
  {
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*static_cloud_, msg);
    msg.header.frame_id = odom_frame_;
    msg.header.stamp = scan_stamp_;
    static_points_pub_.publish(msg);
  }
  if (ground_pub_.getNumSubscribers() > 0)
  {
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*ground_cloud_, msg);
    msg.header.frame_id = odom_frame_;
    msg.header.stamp = scan_stamp_;
    ground_pub_.publish(msg);
  }
}

void OdomNode::publishPath()
{
  // Sign flip check
  static Eigen::Quaternionf q_diff{ 1., 0., 0., 0. };
  static Eigen::Quaternionf q_last{ 1., 0., 0., 0. };

  q_diff = q_last.conjugate() * rotq_;

  // If q_diff has negative real part then there was a sign flip
  if (q_diff.w() < 0)
  {
    rotq_.w() = -rotq_.w();
    rotq_.vec() = -rotq_.vec();
  }

  q_last = rotq_;

  pose_ros_.header.stamp = scan_stamp_;
  pose_ros_.header.frame_id = odom_frame_;

  pose_ros_.pose.position.x = pose_[0];
  pose_ros_.pose.position.y = pose_[1];
  pose_ros_.pose.position.z = pose_[2];

  pose_ros_.pose.orientation.w = rotq_.w();
  pose_ros_.pose.orientation.x = rotq_.x();
  pose_ros_.pose.orientation.y = rotq_.y();
  pose_ros_.pose.orientation.z = rotq_.z();
  pose_pub_.publish(pose_ros_);

  path_.poses.push_back(pose_ros_);
  path_.header.stamp = scan_stamp_;
  path_.header.frame_id = odom_frame_;
  if (path_pub_.getNumSubscribers() > 0)
    path_pub_.publish(path_);
}

void OdomNode::publishTransform()
{
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = scan_stamp_;
  transformStamped.header.frame_id = odom_frame_;
  transformStamped.child_frame_id = child_frame_;

  transformStamped.transform.translation.x = pose_[0];
  transformStamped.transform.translation.y = pose_[1];
  transformStamped.transform.translation.z = pose_[2];

  transformStamped.transform.rotation.w = rotq_.w();
  transformStamped.transform.rotation.x = rotq_.x();
  transformStamped.transform.rotation.y = rotq_.y();
  transformStamped.transform.rotation.z = rotq_.z();

  br.sendTransform(transformStamped);
}

void OdomNode::publishKeyframe()
{
  // Publish keyframe pose
  kf_.header.stamp = scan_stamp_;
  kf_.header.frame_id = odom_frame_;
  kf_.child_frame_id = child_frame_;

  kf_.pose.pose.position.x = pose_[0];
  kf_.pose.pose.position.y = pose_[1];
  kf_.pose.pose.position.z = pose_[2];

  kf_.pose.pose.orientation.w = rotq_.w();
  kf_.pose.pose.orientation.x = rotq_.x();
  kf_.pose.pose.orientation.y = rotq_.y();
  kf_.pose.pose.orientation.z = rotq_.z();

  kf_pub_.publish(kf_);

  // Publish keyframe scan
  if (keyframe_cloud_->points.size() == keyframe_cloud_->width * keyframe_cloud_->height)
  {
    sensor_msgs::PointCloud2 keyframe_cloud_ros;
    pcl::toROSMsg(*keyframe_cloud_, keyframe_cloud_ros);
    keyframe_cloud_ros.header.stamp = scan_stamp_;
    keyframe_cloud_ros.header.frame_id = odom_frame_;
    keyframe_pub_.publish(keyframe_cloud_ros);
  }
}

void OdomNode::publishSubmap()
{
  sensor_msgs::PointCloud2 submap_cloud_ros;
  pcl::toROSMsg(*submap_cloud_, submap_cloud_ros);
  submap_cloud_ros.header.stamp = scan_stamp_;
  submap_cloud_ros.header.frame_id = odom_frame_;
  submap_pub_.publish(submap_cloud_ros);
}

void OdomNode::preprocessPoints()
{
  ROS_INFO("[Before downsampleFilter] registration_scan_ : H:%d x W:%d", registration_scan_->width, registration_scan_->height);
  if (downsample_filter_use_)
  {
    if (!registration_scan_->isOrganized())
    {
      ROS_WARN("Downsample filter requires organized point cloud");
      return;
    }
    downsample_filter_.setInputCloud(registration_scan_);
    downsample_filter_.setKeepOrganized(true);
    downsample_filter_.filter(*registration_scan_);
  }
  ROS_INFO("[After downsampleFilter] registration_scan_ : H:%d x W:%d", registration_scan_->width, registration_scan_->height);

  ROS_INFO("[Before cropboxFilter] registration_scan_ : H:%d x W:%d", registration_scan_->width, registration_scan_->height);
  // Crop Box Filter
  if (crop_use_)
  {
    crop1_.setInputCloud(registration_scan_);
    crop1_.setKeepOrganized(true);
    crop1_.filter(*registration_scan_);
  }
  ROS_INFO("[After cropboxFilter] registration_scan_ : H:%d x W:%d", registration_scan_->width, registration_scan_->height);
  ROS_INFO("[Before voxelGridFilter] registration_scan_ : H:%d x W:%d", registration_scan_->width, registration_scan_->height);
  // Voxel Grid Filter
  if (vf_scan_use_)
  {
    vf_scan_.setInputCloud(registration_scan_);
    vf_scan_.setSaveLeafLayout(true);
    vf_scan_.filter(*registration_scan_);
    // downsample_filter_indices_vec_ = vf_scan_.getLeafLayout();
  }
  // ROS_INFO("downsample_filter_indices_vec_ size: %zu", downsample_filter_indices_vec_.size());
  ROS_INFO("[After voxelGridFilter] registration_scan_ : H:%d x W:%d", registration_scan_->width, registration_scan_->height);
}

void OdomNode::initializeInputTarget()
{
  prev_frame_stamp_ = curr_frame_stamp_;

  // Convert ros message
  target_cloud_ = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
  target_cloud_ = registration_scan_;
  gicp_s2s_.setInputTarget(target_cloud_);
  gicp_s2s_.calculateTargetCovariances();

  // initialize keyframes
  pcl::PointCloud<PointType>::Ptr first_keyframe(new pcl::PointCloud<PointType>);
  pcl::transformPointCloud(*target_cloud_, *first_keyframe, T_);

  // voxelization for submap
  if (vf_submap_use_)
  {
    vf_submap_.setInputCloud(first_keyframe);
    vf_submap_.filter(*first_keyframe);
  }

  // keep history of keyframes
  keyframes_.push_back(std::make_pair(std::make_pair(pose_, rotq_), first_keyframe));
  *keyframes_cloud_ += *first_keyframe;
  *keyframe_cloud_ = *first_keyframe;

  // compute kdtree and keyframe normals (use gicp_s2s input source as temporary
  // storage because it will be overwritten by setInputSources())
  gicp_s2s_.setInputSource(keyframe_cloud_);
  gicp_s2s_.calculateSourceCovariances();
  keyframe_normals_.push_back(gicp_s2s_.getSourceCovariances());

  publish_keyframe_thread_ = std::thread(&OdomNode::publishKeyframe, this);
  publish_keyframe_thread_.detach();

  ++num_keyframes_;
}

void OdomNode::setInputSources()
{
  // set the input source for the S2S gicp
  // this builds the KdTree of the source cloud
  // this does not build the KdTree for s2m because force_no_update is true
  gicp_s2s_.setInputSource(registration_scan_);

  // set pcl::Registration input source for S2M gicp using custom NanoGICP
  // function
  gicp_s2m_.registerInputSource(registration_scan_);

  // now set the KdTree of S2M gicp using previously built KdTree
  gicp_s2m_.source_kdtree_ = gicp_s2s_.source_kdtree_;
  gicp_s2m_.source_covs_.clear();
}

void OdomNode::gravityAlign()
{
  while (imu_msgs_.size() < 1000)
  {
    ros::Duration(0.1).sleep();
  }

  Eigen::Vector3f lin_accel = Eigen::Vector3f::Zero();
  for (const auto& imu : imu_msgs_)
  {
    lin_accel[0] += imu.linear_acceleration.x;
    lin_accel[1] += imu.linear_acceleration.y;
    lin_accel[2] += imu.linear_acceleration.z;
  }
  lin_accel[0] /= imu_msgs_.size();
  lin_accel[1] /= imu_msgs_.size();
  lin_accel[2] /= imu_msgs_.size();

  // normalize
  double lin_norm = sqrt(pow(lin_accel[0], 2) + pow(lin_accel[1], 2) + pow(lin_accel[2], 2));
  lin_accel[0] /= lin_norm;
  lin_accel[1] /= lin_norm;
  lin_accel[2] /= lin_norm;

  // define gravity vector (assume point downwards)
  Eigen::Vector3f grav;
  grav << 0, 0, 1;

  // calculate angle between the two vectors
  Eigen::Quaternionf grav_q = Eigen::Quaternionf::FromTwoVectors(lin_accel, grav);

  // normalize
  double grav_norm =
      sqrt(grav_q.w() * grav_q.w() + grav_q.x() * grav_q.x() + grav_q.y() * grav_q.y() + grav_q.z() * grav_q.z());
  grav_q.w() /= grav_norm;
  grav_q.x() /= grav_norm;
  grav_q.y() /= grav_norm;
  grav_q.z() /= grav_norm;

  // IMU on our is rotated
  Eigen::Quaternionf imu_lidar_tf;
  imu_lidar_tf.x() = 0;
  imu_lidar_tf.y() = 0;
  imu_lidar_tf.z() = -0.7071067811865474;
  imu_lidar_tf.w() = 0.7071067811865474;

  // set gravity aligned orientation
  rotq_ = grav_q * imu_lidar_tf;
  T_.block(0, 0, 3, 3) = rotq_.toRotationMatrix();
  T_s2s_.block(0, 0, 3, 3) = rotq_.toRotationMatrix();
  T_s2s_prev_.block(0, 0, 3, 3) = rotq_.toRotationMatrix();

  // rpy
  auto euler = grav_q.toRotationMatrix().eulerAngles(2, 1, 0);
  double yaw = euler[0] * (180.0 / M_PI);
  double pitch = euler[1] * (180.0 / M_PI);
  double roll = euler[2] * (180.0 / M_PI);

  std::cout << "done" << std::endl;
  std::cout << "  Roll [deg]: " << roll << std::endl;
  std::cout << "  Pitch [deg]: " << pitch << std::endl << std::endl;

  imu_msgs_.clear();
}

void OdomNode::initializeDDLO()
{
  // Gravity Align
  if (gravity_align_)
  {
    std::cout << "Aligning to gravity... ";
    std::cout.flush();
    gravityAlign();
    imu_sub_.shutdown();
  }

  ddlo_initialized_ = true;
  std::cout << "DLO initialized! Starting localization..." << std::endl;
}

void OdomNode::icpCB(const sensor_msgs::PointCloud2ConstPtr& pc)
{
  ROS_INFO("icpCB, input pc width: %d, height: %d", pc->width, pc->height);
  time_stats_["total"].tick();
  time_stats_["odometry"].tick();

  scan_stamp_ = pc->header.stamp;
  curr_frame_stamp_ = pc->header.stamp.toSec();
  scan_size_ = pc->width * pc->height;

  registration_scan_ = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
  segmentation_scan_ = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
  pcl::PointCloud<pcl::PointXYZI> tmp_xyzi;
  pcl::fromROSMsg(*pc, tmp_xyzi);
  pcl::copyPointCloud(tmp_xyzi, *registration_scan_);
  ROS_INFO("[icpCB] registration_scan_ : H:%d x W:%d", registration_scan_->width, registration_scan_->height);
  // ROS_INFO("icpCB, registration_scan size: %d", registration_scan_->size());
  if (dynamic_detection_)
    pcl::copyPointCloud(tmp_xyzi, *segmentation_scan_);

  // If there are too few points in the pointcloud, try again
  if (registration_scan_->points.size() < gicp_min_num_points_)
  {
    ROS_WARN("Low number of points!");
    return;
  }

  // DLO Initialization procedures (IMU calib, gravity align)
  if (!ddlo_initialized_)
  {
    initializeDDLO();
    return;
  }
  if (first_seq_ == 0)
    first_seq_ = pc->header.seq - 1;

  current_seq_ = pc->header.seq;

  // Preprocess points
  preprocessPoints();

  // Compute Metrics
  metrics_thread_ = std::thread(&OdomNode::computeMetrics, this);
  metrics_thread_.detach();

  // Set Adaptive Parameters
  setAdaptiveParams();

  // Set initial frame as target
  if (target_cloud_ == nullptr)
  {
    initializeInputTarget();
    return;
  }

  // Set source frame
  source_cloud_ = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
  source_cloud_ = registration_scan_;

  // Set new frame as input source for both gicp objects
  setInputSources();
  ROS_INFO("[setInputSources] registration_scan_ : H:%d x W:%d", registration_scan_->width, registration_scan_->height);

  // Get the next pose via S2S + S2M
  scanMatching();
  ROS_INFO("[scanMatching] registration_scan_ : H:%d x W:%d", registration_scan_->width, registration_scan_->height);

  // transform point clouds
  transformScans();
  time_stats_["odometry"].tock();

  // Image projection and segmentation
  // Without this step it's basically plain DLO
  if (dynamic_detection_)
  {
    time_stats_["dynamic"].tick();
    applySegmentation();
    *registration_scan_t_ = *segmentation_scan_t_;
    time_stats_["dynamic"].tock();
  }

  // Swap source and target
  *target_cloud_ = *source_cloud_;

  // Update current keyframe poses and map
  updateKeyframes();

  // Update trajectory
  trajectory.push_back(std::make_pair(pose_, rotq_));

  // Update next time stamp
  prev_frame_stamp_ = curr_frame_stamp_;

  // Update statistics
  time_stats_["total"].tock();

  // Publish stuff to ROS
  publish_thread_ = std::thread(&OdomNode::publishToROS, this);
  publish_thread_.detach();

  // Debug statements and publish custom DLO message
  if (print_status_)
  {
    debug_thread_ = std::thread(&OdomNode::debug, this);
    debug_thread_.detach();
  }

  scan_count_++;
}

void OdomNode::imuCB(const sensor_msgs::Imu::ConstPtr& imu)
{
  if (!gravity_align_ || ddlo_initialized_)
    return;

  if (imu_msgs_.size() < 1000)
    imu_msgs_.push_back(*imu);
}

void OdomNode::mapCB(const std_msgs::Int32ConstPtr& map_info)
{
  full_map_size_ = map_info->data;
}

void OdomNode::scanMatching()
{
  //
  // FRAME-TO-FRAME PROCEDURE
  //

  // Align using IMU prior if available
  pcl::PointCloud<PointType>::Ptr aligned(new pcl::PointCloud<PointType>);

  gicp_s2s_.align(*aligned);

  // Get the local S2S transform
  Eigen::Matrix4f T_S2S = gicp_s2s_.getFinalTransformation();
  T_s2s_orig_ = T_S2S;  // the T_s2s later gets overriden (global), so we need to
                        // save a copy here

  // Get the global S2S transform
  propagateS2S(T_S2S);

  // reuse covariances from s2s for s2m
  gicp_s2m_.source_covs_ = gicp_s2s_.source_covs_;

  // Swap source and target (which also swaps KdTrees internally) for next S2S
  gicp_s2s_.swapSourceAndTarget();

  //
  // FRAME-TO-SUBMAP
  //

  // Get current global submap
  getSubmapKeyframes();

  if (submap_hasChanged_)
  {
    // Set the current global submap as the target cloud
    gicp_s2m_.setInputTarget(submap_cloud_);

    // Set target cloud's normals as submap normals
    gicp_s2m_.setTargetCovariances(submap_normals_);
  }

  // Align with current submap with global S2S transformation as initial guess
  gicp_s2m_.align(*aligned, T_s2s_);

  // Get final transformation in global frame
  T_ = gicp_s2m_.getFinalTransformation();

  std::vector<double> residuals;
  gicp_s2m_.getResiduals(residuals, T_);

  // Set residals as intensity
  // ROS_INFO("registration_scan_ : H:%d x W:%d", registration_scan_->width, registration_scan_->height);
  // ROS_INFO("registration_scan_ size : %d", registration_scan_->size());
  // pcl::copyPointCloud(*registration_scan_, *residuals_cloud_);
  // ROS_INFO("residuals_cloud_ : H:%d x W:%d", residuals_cloud_->width, residuals_cloud_->height);
  // ROS_INFO("ResidualsCloud size : %d", residuals_cloud_->size());
  
  // ROS_INFO("ResidualsCloud width : %d , height : %d", residuals_cloud_->width, residuals_cloud_->height);
  // ROS_INFO("Residuals size: %d", residuals.size());
  double theta_min = -60 * M_PI / 180;
  double theta_max = 60 * M_PI / 180;
  residuals_cloud_->points.clear();
  residuals_cloud_->points.resize(512 * 512);
  residuals_cloud_->width = 512;
  residuals_cloud_->height = 512;
  // residuals_cloud_->is_dense = false;
  ROS_INFO("residuals_cloud_ : H:%d x W:%d", residuals_cloud_->width, residuals_cloud_->height);
  for(int i = 0; i < registration_scan_->size(); ++i){
    const PointType& pt = registration_scan_->points[i];
    double theta = atan2(pt.x, pt.z);
    double phi = atan2(pt.y, sqrt(pt.x * pt.x + pt.z * pt.z));
    int u = static_cast<int>((theta - theta_min) / (theta_max - theta_min) * 512);
    int v = static_cast<int>((phi - theta_min) / (theta_max - theta_min) * 512);

    int idx = v * 512 + u;
    if (u < 0 || u >= 512 || v < 0 || v >= 512)
      continue;

    residuals_cloud_->points[idx].x = pt.x;
    residuals_cloud_->points[idx].y = pt.y;
    residuals_cloud_->points[idx].z = pt.z;
    residuals_cloud_->points[idx].intensity = residuals[i];
  }
  ROS_INFO("done");
  // for (int i = 0; i < residuals_cloud_->size(); i++)
  //   residuals_cloud_->points[i].intensity = residuals[i];
  // size_t num_valid_points = 0;
  // for(int row = 0; row < 512; ++row){
  //   for(int col = 0; col < 512; ++col){
  //     int idx = row * 512 + col;
  //     if(std::isnan(registration_scan_->points[idx].x) || std::isnan(registration_scan_->points[idx].y) || std::isnan(registration_scan_->points[idx].z))
  //       continue;
  //     ++num_valid_points;
  //     // residuals_cloud_->points[idx].intensity = residuals[idx];
  //     // std::cout << registration_scan_->points[idx].x << ", " << registration_scan_->points[idx].y << ", " << registration_scan_->points[idx].z << std::endl;
  //   }
  // }
  // ROS_INFO("num_valid_points: %d", num_valid_points);

  // Update the S2S transform for next propagation
  T_s2s_prev_ = T_;

  // Update next global pose
  // Both source and target clouds are in the global frame now, so tranformation
  // is global
  propagateS2M();
}

void OdomNode::applySegmentation()
{
  detection_module_.projectScan(segmentation_scan_, segmentation_scan_t_, T_, T_s2s_orig_);
  detection_module_.projectResiduals(residuals_cloud_);
  detection_module_.applySegmentation();

  if (ground_pub_.getNumSubscribers() > 0)
  {
    std::vector<int> ground_indices;
    detection_module_.getGroundIndices(ground_indices);
    ground_cloud_ = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
    pcl::copyPointCloud(*segmentation_scan_t_, ground_indices, *ground_cloud_);
  }

  // Retrieve the indices of points that were marked dynamic/undefined
  pcl::PointIndicesPtr non_static_indices = pcl::PointIndicesPtr(new pcl::PointIndices());
  detection_module_.getIndices(non_static_indices->indices,
                               std::vector<ObjectStatus>{ ObjectStatus::UNDEFINED, ObjectStatus::DYNAMIC });

  // Save clouds for visualization (will be overridden otherwise before publishing)
  if (non_static_points_pub_.getNumSubscribers() > 0)
  {
    non_static_cloud_ = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
    pcl::copyPointCloud(*segmentation_scan_t_, *non_static_indices, *non_static_cloud_);
  }

  if (dynamic_points_pub_.getNumSubscribers() > 0)
  {
    pcl::PointIndicesPtr dynamic_indices = pcl::PointIndicesPtr(new pcl::PointIndices());
    detection_module_.getIndices(dynamic_indices->indices, ObjectStatus::DYNAMIC);
    dynamic_cloud_ = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
    pcl::copyPointCloud(*segmentation_scan_t_, *dynamic_indices, *dynamic_cloud_);
  }

  // Remove non-static indices from original scan
  pcl::ExtractIndices<PointType> extract;
  extract.setInputCloud(segmentation_scan_t_);
  extract.setIndices(non_static_indices);
  extract.setNegative(true);
  extract.filter(*segmentation_scan_t_);

  // Prepare for visualizetion
  if (static_points_pub_.getNumSubscribers() > 0)
  {
    static_cloud_ = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
    pcl::copyPointCloud(*segmentation_scan_t_, *static_cloud_);
  }

  // Filter scan (preprocessing of filtered scan)
  if (downsample_filter_use_)
  {
    downsample_filter_.setInputCloud(segmentation_scan_t_);
    downsample_filter_.filter(*segmentation_scan_t_);
  }
  if (crop_use_)
  {
    crop2_.setInputCloud(segmentation_scan_t_);
    crop2_.setTranslation(pose_);  // input cloud is transformed, so croping
                                   // within [-1 1] doesn't work
    crop2_.filter(*segmentation_scan_t_);
  }
  if (vf_scan_use_)
  {
    vf_scan_.setInputCloud(segmentation_scan_t_);
    vf_scan_.filter(*segmentation_scan_t_);
  }
}

void OdomNode::propagateS2S(Eigen::Matrix4f T)
{
  T_s2s_ = T_s2s_prev_ * T;
  T_s2s_prev_ = T_s2s_;

  pose_s2s_ << T_s2s_(0, 3), T_s2s_(1, 3), T_s2s_(2, 3);
  rotSO3_s2s_ << T_s2s_(0, 0), T_s2s_(0, 1), T_s2s_(0, 2), T_s2s_(1, 0), T_s2s_(1, 1), T_s2s_(1, 2), T_s2s_(2, 0),
      T_s2s_(2, 1), T_s2s_(2, 2);

  Eigen::Quaternionf q(rotSO3_s2s_);

  // Normalize quaternion
  double norm = sqrt(q.w() * q.w() + q.x() * q.x() + q.y() * q.y() + q.z() * q.z());
  q.w() /= norm;
  q.x() /= norm;
  q.y() /= norm;
  q.z() /= norm;
  rotq_s2s_ = q;
}

void OdomNode::propagateS2M()
{
  pose_ << T_(0, 3), T_(1, 3), T_(2, 3);
  rotSO3_ << T_(0, 0), T_(0, 1), T_(0, 2), T_(1, 0), T_(1, 1), T_(1, 2), T_(2, 0), T_(2, 1), T_(2, 2);

  Eigen::Quaternionf q(rotSO3_);

  // Normalize quaternion
  double norm = sqrt(q.w() * q.w() + q.x() * q.x() + q.y() * q.y() + q.z() * q.z());
  q.w() /= norm;
  q.x() /= norm;
  q.y() /= norm;
  q.z() /= norm;
  rotq_ = q;
}

void OdomNode::transformScans()
{
  registration_scan_t_ = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
  pcl::transformPointCloud(*registration_scan_, *registration_scan_t_, T_);
  segmentation_scan_t_ = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
  pcl::transformPointCloud(*segmentation_scan_, *segmentation_scan_t_, T_);
}

void OdomNode::computeMetrics()
{
  computeSpaciousness();
}

void OdomNode::computeSpaciousness()
{
  // compute range of points
  std::vector<float> ds;

  for (int i = 0; i <= registration_scan_->points.size(); i++)
  {
    float d = std::sqrt(pow(registration_scan_->points[i].x, 2) + pow(registration_scan_->points[i].y, 2) +
                        pow(registration_scan_->points[i].z, 2));
    ds.push_back(d);
  }

  // median
  std::nth_element(ds.begin(), ds.begin() + ds.size() / 2, ds.end());
  float median_curr = ds[ds.size() / 2];
  static float median_prev = median_curr;
  float median_lpf = 0.95 * median_prev + 0.05 * median_curr;
  median_prev = median_lpf;

  // push
  metrics_.spaciousness.push_back(median_lpf);
}

void OdomNode::computeConvexHull()
{
  // at least 4 keyframes for convex hull
  if (num_keyframes_ < 4)
  {
    return;
  }

  // create a pointcloud with points at keyframes
  pcl::PointCloud<PointType>::Ptr cloud = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);

  for (const auto& k : keyframes_)
  {
    PointType pt;
    pt.x = k.first.first[0];
    pt.y = k.first.first[1];
    pt.z = k.first.first[2];
    cloud->push_back(pt);
  }

  // calculate the convex hull of the point cloud
  convex_hull_.setInputCloud(cloud);

  // get the indices of the keyframes on the convex hull
  pcl::PointCloud<PointType>::Ptr convex_points = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
  convex_hull_.reconstruct(*convex_points);

  pcl::PointIndices::Ptr convex_hull_point_idx = pcl::PointIndices::Ptr(new pcl::PointIndices);
  convex_hull_.getHullPointIndices(*convex_hull_point_idx);

  keyframe_convex_.clear();
  for (int i = 0; i < convex_hull_point_idx->indices.size(); ++i)
  {
    keyframe_convex_.push_back(convex_hull_point_idx->indices[i]);
  }
}

void OdomNode::computeConcaveHull()
{
  // at least 5 keyframes for concave hull
  if (num_keyframes_ < 5)
  {
    return;
  }

  // create a pointcloud with points at keyframes
  pcl::PointCloud<PointType>::Ptr cloud = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);

  for (const auto& k : keyframes_)
  {
    PointType pt;
    pt.x = k.first.first[0];
    pt.y = k.first.first[1];
    pt.z = k.first.first[2];
    cloud->push_back(pt);
  }

  // calculate the concave hull of the point cloud
  concave_hull_.setInputCloud(cloud);

  // get the indices of the keyframes on the concave hull
  pcl::PointCloud<PointType>::Ptr concave_points = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
  concave_hull_.reconstruct(*concave_points);

  pcl::PointIndices::Ptr concave_hull_point_idx = pcl::PointIndices::Ptr(new pcl::PointIndices);
  concave_hull_.getHullPointIndices(*concave_hull_point_idx);

  keyframe_concave_.clear();
  for (int i = 0; i < concave_hull_point_idx->indices.size(); ++i)
  {
    keyframe_concave_.push_back(concave_hull_point_idx->indices[i]);
  }
}

void OdomNode::updateKeyframes()
{
  // calculate difference in pose and rotation to all poses in trajectory
  float closest_d = std::numeric_limits<float>::infinity();
  int closest_idx = 0;
  int keyframes_idx = 0;

  int num_nearby = 0;

  for (const auto& k : keyframes_)
  {
    // calculate distance between current pose and pose in keyframes
    float delta_d = sqrt(pow(pose_[0] - k.first.first[0], 2) + pow(pose_[1] - k.first.first[1], 2) +
                         pow(pose_[2] - k.first.first[2], 2));

    // count the number nearby current pose
    if (delta_d <= keyframe_thresh_dist_ * 1.5)
    {
      ++num_nearby;
    }

    // store into variable
    if (delta_d < closest_d)
    {
      closest_d = delta_d;
      closest_idx = keyframes_idx;
    }

    keyframes_idx++;
  }

  // get closest pose and corresponding rotation
  Eigen::Vector3f closest_pose = keyframes_[closest_idx].first.first;
  Eigen::Quaternionf closest_pose_r = keyframes_[closest_idx].first.second;

  // calculate distance between current pose and closest pose from above
  float dd = sqrt(pow(pose_[0] - closest_pose[0], 2) + pow(pose_[1] - closest_pose[1], 2) +
                  pow(pose_[2] - closest_pose[2], 2));

  // calculate difference in orientation
  Eigen::Quaternionf dq = rotq_ * (closest_pose_r.inverse());

  float theta_rad = 2. * atan2(sqrt(pow(dq.x(), 2) + pow(dq.y(), 2) + pow(dq.z(), 2)), dq.w());
  float theta_deg = theta_rad * (180.0 / M_PI);

  // update keyframe
  bool newKeyframe = false;

  if (abs(dd) > keyframe_thresh_dist_ || abs(theta_deg) > keyframe_thresh_rot_)
  {
    newKeyframe = true;
  }
  if (abs(dd) <= keyframe_thresh_dist_)
  {
    newKeyframe = false;
  }
  if (abs(dd) <= keyframe_thresh_dist_ && abs(theta_deg) > keyframe_thresh_rot_ && num_nearby <= 1)
  {
    newKeyframe = true;
  }

  if (newKeyframe)
  {
    ++num_keyframes_;

    // voxelization for submap
    if (vf_submap_use_)
    {
      vf_submap_.setInputCloud(registration_scan_t_);
      vf_submap_.filter(*registration_scan_t_);
    }

    // update keyframe vector
    keyframes_.push_back(std::make_pair(std::make_pair(pose_, rotq_), registration_scan_t_));

    // compute kdtree and keyframe normals (use gicp_s2s input source as
    // temporary storage because it will be overwritten by setInputSources())
    *keyframes_cloud_ += *registration_scan_t_;
    *keyframe_cloud_ = *registration_scan_t_;

    gicp_s2s_.setInputSource(keyframe_cloud_);
    gicp_s2s_.calculateSourceCovariances();
    keyframe_normals_.push_back(gicp_s2s_.getSourceCovariances());

    publish_keyframe_thread_ = std::thread(&OdomNode::publishKeyframe, this);
    publish_keyframe_thread_.detach();
  }
}

void OdomNode::setAdaptiveParams()
{
  // Set Keyframe Thresh from Spaciousness Metric
  if (metrics_.spaciousness.back() > 20.0)
  {
    keyframe_thresh_dist_ = 10.0;
  }
  else if (metrics_.spaciousness.back() > 10.0 && metrics_.spaciousness.back() <= 20.0)
  {
    keyframe_thresh_dist_ = 5.0;
  }
  else if (metrics_.spaciousness.back() > 5.0 && metrics_.spaciousness.back() <= 10.0)
  {
    keyframe_thresh_dist_ = 1.0;
  }
  else if (metrics_.spaciousness.back() <= 5.0)
  {
    keyframe_thresh_dist_ = 0.5;
  }

  // set concave hull alpha
  concave_hull_.setAlpha(keyframe_thresh_dist_);
}

void OdomNode::pushSubmapIndices(std::vector<float> dists, int k, std::vector<int> frames)
{
  // make sure dists is not empty
  if (dists.empty())
  {
    return;
  }

  // maintain max heap of at most k elements
  std::priority_queue<float> pq;

  for (auto d : dists)
  {
    if (pq.size() >= k && pq.top() > d)
    {
      pq.push(d);
      pq.pop();
    }
    else if (pq.size() < k)
    {
      pq.push(d);
    }
  }

  // get the kth smallest element, which should be at the top of the heap
  float kth_element = pq.top();

  // get all elements smaller or equal to the kth smallest element
  for (int i = 0; i < dists.size(); ++i)
  {
    if (dists[i] <= kth_element)
      submap_kf_idx_curr_.push_back(frames[i]);
  }
}

void OdomNode::getSubmapKeyframes()
{
  // clear vector of keyframe indices to use for submap
  submap_kf_idx_curr_.clear();

  //
  // TOP K NEAREST NEIGHBORS FROM ALL KEYFRAMES
  //

  // calculate distance between current pose and poses in keyframe set
  std::vector<float> ds;
  std::vector<int> keyframe_nn;
  int i = 0;
  Eigen::Vector3f curr_pose = T_s2s_.block(0, 3, 3, 1);

  for (const auto& k : keyframes_)
  {
    float d = sqrt(pow(curr_pose[0] - k.first.first[0], 2) + pow(curr_pose[1] - k.first.first[1], 2) +
                   pow(curr_pose[2] - k.first.first[2], 2));
    ds.push_back(d);
    keyframe_nn.push_back(i);
    i++;
  }

  // get indices for top K nearest neighbor keyframe poses
  pushSubmapIndices(ds, submap_knn_, keyframe_nn);

  //
  // TOP K NEAREST NEIGHBORS FROM CONVEX HULL
  //

  // get convex hull indices
  computeConvexHull();

  // get distances for each keyframe on convex hull
  std::vector<float> convex_ds;
  for (const auto& c : keyframe_convex_)
  {
    convex_ds.push_back(ds[c]);
  }

  // get indicies for top kNN for convex hull
  pushSubmapIndices(convex_ds, submap_kcv_, keyframe_convex_);

  //
  // TOP K NEAREST NEIGHBORS FROM CONCAVE HULL
  //

  // get concave hull indices
  computeConcaveHull();

  // get distances for each keyframe on concave hull
  std::vector<float> concave_ds;
  for (const auto& c : keyframe_concave_)
  {
    concave_ds.push_back(ds[c]);
  }

  // get indicies for top kNN for convex hull
  pushSubmapIndices(concave_ds, submap_kcc_, keyframe_concave_);

  //
  // BUILD SUBMAP
  //

  // concatenate all submap clouds and normals
  std::sort(submap_kf_idx_curr_.begin(), submap_kf_idx_curr_.end());
  auto last = std::unique(submap_kf_idx_curr_.begin(), submap_kf_idx_curr_.end());
  submap_kf_idx_curr_.erase(last, submap_kf_idx_curr_.end());

  // sort current and previous submap kf list of indices
  std::sort(submap_kf_idx_curr_.begin(), submap_kf_idx_curr_.end());
  std::sort(submap_kf_idx_prev_.begin(), submap_kf_idx_prev_.end());

  // check if submap has changed from previous iteration
  if (submap_kf_idx_curr_ == submap_kf_idx_prev_)
  {
    submap_hasChanged_ = false;
  }
  else
  {
    submap_hasChanged_ = true;

    // reinitialize submap cloud, normals
    pcl::PointCloud<PointType>::Ptr submap_cloud(boost::make_shared<pcl::PointCloud<PointType>>());
    submap_normals_.clear();

    for (auto k : submap_kf_idx_curr_)
    {
      // create current submap cloud
      *submap_cloud += *keyframes_[k].second;

      // grab corresponding submap cloud's normals
      submap_normals_.insert(std::end(submap_normals_), std::begin(keyframe_normals_[k]),
                             std::end(keyframe_normals_[k]));
    }

    submap_cloud_ = submap_cloud;
    submap_kf_idx_prev_ = submap_kf_idx_curr_;
  }
}

void OdomNode::debug()
{
  // Total length traversed
  double length_traversed = 0.;
  Eigen::Vector3f p_curr = Eigen::Vector3f(0., 0., 0.);
  Eigen::Vector3f p_prev = Eigen::Vector3f(0., 0., 0.);
  for (const auto& t : trajectory)
  {
    if (p_prev == Eigen::Vector3f(0., 0., 0.))
    {
      p_prev = t.first;
      continue;
    }
    p_curr = t.first;
    double l = sqrt(pow(p_curr[0] - p_prev[0], 2) + pow(p_curr[1] - p_prev[1], 2) + pow(p_curr[2] - p_prev[2], 2));

    if (l >= 0.05)
    {
      length_traversed += l;
      p_prev = p_curr;
    }
  }

  if (length_traversed == 0)
  {
    publish_keyframe_thread_ = std::thread(&OdomNode::publishKeyframe, this);
    publish_keyframe_thread_.detach();
  }

  // RAM Usage
  double vm_usage = 0.0;
  double resident_set = 0.0;
  std::ifstream stat_stream("/proc/self/stat",
                            std::ios_base::in);  // get info from proc directory
  std::string pid, comm, state, ppid, pgrp, session, tty_nr;
  std::string tpgid, flags, minflt, cminflt, majflt, cmajflt;
  std::string utime, stime, cutime, cstime, priority, nice;
  std::string num_threads, itrealvalue, starttime;
  unsigned long vsize;
  long rss;
  stat_stream >> pid >> comm >> state >> ppid >> pgrp >> session >> tty_nr >> tpgid >> flags >> minflt >> cminflt >>
      majflt >> cmajflt >> utime >> stime >> cutime >> cstime >> priority >> nice >> num_threads >> itrealvalue >>
      starttime >> vsize >> rss;  // don't care about the rest
  stat_stream.close();
  long page_size_kb = sysconf(_SC_PAGE_SIZE) / 1024;  // for x86-64 is configured to use 2MB pages
  vm_usage = vsize / 1024.0;
  resident_set = rss * page_size_kb;

  // CPU Usage
  struct tms timeSample;
  clock_t now;
  double cpu_percent;
  now = times(&timeSample);
  if (now <= lastCPU_ || timeSample.tms_stime < lastSysCPU_ || timeSample.tms_utime < lastUserCPU_)
  {
    cpu_percent = -1.0;
  }
  else
  {
    cpu_percent = (timeSample.tms_stime - lastSysCPU_) + (timeSample.tms_utime - lastUserCPU_);
    cpu_percent /= (now - lastCPU_);
    cpu_percent /= numProcessors_;
    cpu_percent *= 100.;
  }
  lastCPU_ = now;
  lastSysCPU_ = timeSample.tms_stime;
  lastUserCPU_ = timeSample.tms_utime;
  cpu_percents_.push_back(cpu_percent);
  double avg_cpu_usage = std::accumulate(cpu_percents_.begin(), cpu_percents_.end(), 0.0) / cpu_percents_.size();

  std::stringstream output;

  output << "\033[2J\033[1;1H";
  output << std::endl << "==== Dynamic Direct LiDAR Odometry ====" << std::endl;

  if (!cpu_type_.empty())
  {
    output << std::endl << cpu_type_ << " x " << numProcessors_ << std::endl;
  }

  output << std::endl << std::setprecision(4) << std::fixed;
  output << "Scan count: " << scan_count_ << "  Dropped scans: "
         << " " << (int)current_seq_ - (int)first_seq_ - (int)scan_count_;
  output << std::endl << std::endl;
  output << "Position    [xyz]  :: " << pose_[0] << " " << pose_[1] << " " << pose_[2] << std::endl;
  output << "Orientation [wxyz] :: " << rotq_.w() << " " << rotq_.x() << " " << rotq_.y() << " " << rotq_.z()
         << std::endl;
  output << "Distance Traveled  :: " << length_traversed << " meters" << std::endl;
  output << "Distance to Origin :: "
         << sqrt(pow(pose_[0] - origin_[0], 2) + pow(pose_[1] - origin_[1], 2) + pow(pose_[2] - origin_[2], 2))
         << " meters" << std::endl;

  output << std::endl << std::right << std::setprecision(2) << std::fixed;
  output << "Process             Time      Avg       Var       Min       Max" << std::endl;
  std::vector<std::string> time_stats_names({ "total", "odometry", "dynamic" });
  for (const auto& name : time_stats_names)
  {
    output << std::left << std::setw(20) << name << std::setfill(' ') << std::setw(10) << time_stats_[name].getLast()
           << std::setw(10) << time_stats_[name].getMean() << std::setw(10) << time_stats_[name].getVariance()
           << std::setw(10) << time_stats_[name].getMin() << std::setw(10) << time_stats_[name].getMax() << std::endl;
  }
  time_stats_names = std::vector<std::string>({ "groundRemoval", "cloudSegmentation", "computeAllObjects",
                                                "trackDetections", "projectScan", "projectResiduals" });
  for (auto name : time_stats_names)
  {
    output << std::left << std::setw(20) << "  " + name << std::setfill(' ') << std::setw(10)
           << detection_module_.getTime(name, "last") << std::setw(10) << detection_module_.getTime(name, "mean")
           << std::setw(10) << detection_module_.getTime(name, "var") << std::setw(10)
           << detection_module_.getTime(name, "min") << std::setw(10) << detection_module_.getTime(name, "max")
           << std::endl;
  }

  output << std::endl;
  output << "Cores Utilized   :: " << std::setfill(' ') << std::setw(6) << (cpu_percent / 100.) * numProcessors_
         << " cores // Avg: " << std::setw(5) << (avg_cpu_usage / 100.) * numProcessors_ << std::endl;
  output << "CPU Load         :: " << std::setfill(' ') << std::setw(6) << cpu_percent
         << " %     // Avg: " << std::setw(5) << avg_cpu_usage << std::endl;
  output << "RAM Allocation   :: " << std::setfill(' ') << std::setw(6) << resident_set / 1000.
         << " MB    // VSZ: " << vm_usage / 1000. << " MB" << std::endl
         << std::endl;
  output << "Scan Size        :: " << std::setfill(' ') << std::setw(6) << scan_size_
         << " Points, after filter: " << registration_scan_->size() << " ("
         << float(registration_scan_->size()) / float(scan_size_) * 100 << "%)" << std::endl;
  output << "Submap Size      :: " << std::setfill(' ') << std::setw(6) << submap_cloud_->points.size() << " Points"
         << std::endl;
  output << "Full Map Size    :: " << std::setfill(' ') << std::setw(6) << full_map_size_ << " Points" << std::endl;
  output << "Keyframes active :: " << std::setfill(' ') << std::setw(6) << submap_kf_idx_curr_.size() << " of "
         << keyframes_.size() << std::endl;
  output << "Spaciousness     :: " << std::setfill(' ') << std::setw(6) << metrics_.spaciousness.back() << " m"
         << std::endl;
  if (dynamic_detection_)
  {
    output << "Detected Segments:: " << std::setfill(' ') << std::setw(6) << detection_module_.getSegmentsCount()
           << std::endl;
    output << "Tracked Objects  :: " << std::setfill(' ') << std::setw(6) << detection_module_.getObjectsCount()
           << std::endl;
    output << "Dynamic Objects  :: " << std::setfill(' ') << std::setw(6)
           << detection_module_.getObjectsCount(ObjectStatus::DYNAMIC) << std::endl;
    output << "Undefined Objects:: " << std::setfill(' ') << std::setw(6)
           << detection_module_.getObjectsCount(ObjectStatus::UNDEFINED) << std::endl;
    output << "Removed Points   :: " << std::setfill(' ') << std::setw(6) << non_static_cloud_->size() << std::endl;
  }

  // std::cout << output.str();
}
