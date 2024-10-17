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

#include "odometry/ddlo.h"

#include <detection/detection.h>

class OdomNode
{
public:
  OdomNode();
  ~OdomNode() = default;

  static void abort()
  {
    abort_ = true;
  }

  void start();
  void stop();

private:
  void abortTimerCB(const ros::TimerEvent& e);
  void icpCB(const sensor_msgs::PointCloud2ConstPtr& pc);
  void imuCB(const sensor_msgs::Imu::ConstPtr& imu);
  void mapCB(const std_msgs::Int32ConstPtr& map_info);

  void getParams();

  void publishToROS();
  void publishPath();
  void publishTransform();
  void publishKeyframe();
  void publishSubmap();
  void publishPoints();

  void preprocessPoints();
  void initializeInputTarget();
  void setInputSources();

  void initializeDDLO();
  void gravityAlign();

  void scanMatching();

  void applySegmentation();

  void propagateS2S(Eigen::Matrix4f T);
  void propagateS2M();

  void setAdaptiveParams();

  void computeMetrics();
  void computeSpaciousness();

  void transformScans();
  void updateKeyframes();
  void computeConvexHull();
  void computeConcaveHull();
  void pushSubmapIndices(std::vector<float> dists, int k, std::vector<int> frames);
  void getSubmapKeyframes();

  jsk_recognition_msgs::BoundingBox getBoundingBox(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);

  void debug();

  ros::NodeHandle nh_;

  ros::Subscriber icp_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber map_info_sub_;  // for evaluation

  // DLO publishers
  ros::Publisher path_pub_;
  ros::Publisher pose_pub_;
  ros::Publisher keyframe_pub_;
  ros::Publisher kf_pub_;
  ros::Publisher dynamic_points_pub_;
  ros::Publisher non_static_points_pub_;
  ros::Publisher static_points_pub_;
  ros::Publisher submap_pub_;
  ros::Publisher residuals_pub_;
  ros::Publisher ground_pub_;

  Eigen::Vector3f origin_;
  std::vector<std::pair<Eigen::Vector3f, Eigen::Quaternionf>> trajectory;
  std::vector<std::pair<std::pair<Eigen::Vector3f, Eigen::Quaternionf>, pcl::PointCloud<PointType>::Ptr>> keyframes_;
  std::vector<std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>>
      keyframe_normals_;  // = covariances

  std::atomic<bool> ddlo_initialized_;
  std::atomic<bool> imu_calibrated_;

  std::string odom_frame_;
  std::string child_frame_;

  pcl::PointCloud<PointType>::Ptr registration_scan_;
  pcl::PointCloud<PointType>::Ptr registration_scan_t_;
  pcl::PointCloud<PointType>::Ptr segmentation_scan_;
  pcl::PointCloud<PointType>::Ptr segmentation_scan_t_;
  pcl::PointCloud<PointType>::Ptr residuals_cloud_;
  pcl::PointCloud<PointType>::Ptr keyframes_cloud_;  // contains all keyframes
  pcl::PointCloud<PointType>::Ptr keyframe_cloud_;   // contains only current keyframe
  int num_keyframes_;

  pcl::ConvexHull<PointType> convex_hull_;
  pcl::ConcaveHull<PointType> concave_hull_;
  std::vector<int> keyframe_convex_;
  std::vector<int> keyframe_concave_;

  pcl::PointCloud<PointType>::Ptr submap_cloud_;
  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> submap_normals_;

  std::vector<int> submap_kf_idx_curr_;
  std::vector<int> submap_kf_idx_prev_;
  std::atomic<bool> submap_hasChanged_;

  pcl::PointCloud<PointType>::Ptr source_cloud_;
  pcl::PointCloud<PointType>::Ptr target_cloud_;

  pcl::PointCloud<PointType>::Ptr non_static_cloud_;
  pcl::PointCloud<PointType>::Ptr static_cloud_;
  pcl::PointCloud<PointType>::Ptr dynamic_cloud_;
  pcl::PointCloud<PointType>::Ptr ground_cloud_;

  ros::Time scan_stamp_;

  size_t current_seq_;
  size_t first_seq_;
  size_t scan_count_;
  double curr_frame_stamp_;
  double prev_frame_stamp_;

  // Statistics
  bool write_stats_;
  std::string stats_path_;
  std::unordered_map<std::string, AccumulatorData> time_stats_;
  std::ofstream stat_f_;

  nano_gicp::NanoGICP<PointType, PointType> gicp_s2s_;
  nano_gicp::NanoGICP<PointType, PointType> gicp_s2m_;

  DetectionModule detection_module_;

  pcl::CropBox<PointType> crop1_;  // for the cloud used for scan matching
  pcl::CropBox<PointType> crop2_;  // for the cloud after removing dynamic points
  pcl::VoxelGrid<PointType> vf_scan_;
  pcl::VoxelGrid<PointType> vf_submap_;
  pcl::ExtractIndices<PointType> downsample_filter_;
  pcl::IndicesPtr downsample_filter_indices_;
  size_t downsample_filter_row_;
  size_t downsample_filter_col_;
  size_t cloud_height_;
  size_t cloud_width_;

  nav_msgs::Path path_;
  nav_msgs::Odometry kf_;

  geometry_msgs::PoseStamped pose_ros_;

  Eigen::Matrix4f T_;
  Eigen::Matrix4f T_s2s_, T_s2s_prev_, T_s2s_orig_;
  Eigen::Quaternionf q_final;

  Eigen::Vector3f pose_s2s_;
  Eigen::Matrix3f rotSO3_s2s_;
  Eigen::Quaternionf rotq_s2s_;

  Eigen::Vector3f pose_;
  Eigen::Matrix3f rotSO3_;
  Eigen::Quaternionf rotq_;

  Eigen::Matrix4f imu_SE3_;

  std::vector<sensor_msgs::Imu> imu_msgs_;

  struct Metrics
  {
    std::vector<float> spaciousness;
  };

  Metrics metrics_;

  static std::atomic<bool> abort_;
  std::atomic<bool> stop_publish_thread_;
  std::atomic<bool> stop_publish_keyframe_thread_;
  std::atomic<bool> stop_metrics_thread_;
  std::atomic<bool> stop_debug_thread_;

  std::thread publish_thread_;
  std::thread publish_keyframe_thread_;
  std::thread metrics_thread_;
  std::thread debug_thread_;

  std::string cpu_type_;
  std::vector<double> cpu_percents_;
  clock_t lastCPU_, lastSysCPU_, lastUserCPU_;
  int numProcessors_;

  int scan_size_;
  int full_map_size_;
  int filtered_scan_size_;

  // Parameters
  bool gravity_align_;

  double keyframe_thresh_dist_;
  double keyframe_thresh_rot_;

  int submap_knn_;
  int submap_kcv_;
  int submap_kcc_;
  double submap_concave_alpha_;

  bool crop_use_;
  double crop_size_;

  bool downsample_filter_use_;

  bool vf_scan_use_;
  double vf_scan_res_;

  bool vf_submap_use_;
  double vf_submap_res_;

  bool icp_residuals_use_;

  int gicp_min_num_points_;

  int gicps2s_k_correspondences_;
  double gicps2s_max_corr_dist_;
  int gicps2s_max_iter_;
  double gicps2s_transformation_ep_;
  double gicps2s_euclidean_fitness_ep_;
  int gicps2s_ransac_iter_;
  double gicps2s_ransac_inlier_thresh_;

  int gicps2m_k_correspondences_;
  double gicps2m_max_corr_dist_;
  int gicps2m_max_iter_;
  double gicps2m_transformation_ep_;
  double gicps2m_euclidean_fitness_ep_;
  int gicps2m_ransac_iter_;
  double gicps2m_ransac_inlier_thresh_;

  bool dynamic_detection_;
  bool print_status_;
};
