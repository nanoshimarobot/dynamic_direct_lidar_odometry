#pragma once

#include <tracking/tracking.h>

class DetectionModule
{
  typedef pcl::PointXYZI PointType;

private:
  ros::NodeHandle nh_;

  image_transport::ImageTransport it_;
  image_transport::Publisher label_img_pub_;
  image_transport::Publisher range_img_pub_;
  image_transport::Publisher residual_img_pub_;

  pcl::PointCloud<PointType>::Ptr cloud_in_;    // in local frame
  pcl::PointCloud<PointType>::Ptr cloud_in_t_;  // in global frame
  pcl::PointCloud<PointType>::Ptr full_cloud_;  // here we store the points that
                                                // are inserted in the range img

  PointType nan_point_;  // fill in fullCloud at each iteration

  cv::Mat ground_mat_;  // binary matrix for ground points
  cv::Mat range_mat_;   // range matrix for range image
  cv::Mat label_mat_;   // label matrix for segmentaiton marking
  cv::Mat residuals_mat_;

  int label_count_;
  std::vector<std::vector<int>> label_indices_i_;
  std::vector<std::vector<cv::Vec3b>> label_indices_yx_;
  std::vector<double> avg_residuals_;

  pcl::PCLHeader cloud_header_;

  std::vector<std::pair<int8_t, int8_t>> neighbor_iterator_;  // neighbor iterator for segmentation process

  std::vector<uint16_t> all_pushed_ind_X_;
  std::vector<uint16_t> all_pushed_ind_Y_;

  std::vector<uint16_t> queue_ind_Y_;
  std::vector<uint16_t> queue_ind_X_;

  std::vector<pcl::PointCloud<PointType>> clusters_;
  std::vector<Object> detected_objects_;

  std::vector<cv::Vec3b> label_color_lut_;

  std_msgs::Header header_;
  ros::Time time_prev_;

  TrackingModule tracker_;

  // The global transform (start_position=odom -> current position=lidar_frame)
  Eigen::Matrix4f T_;
  // The local transform from previous to current scan
  Eigen::Matrix4f T_s2s_;

  // Image parameters
  int H_;
  int W_;
  float ang_bottom_;
  float ang_res_x_;
  float ang_res_y_;
  float start_orientation_;
  size_t ground_rows_;
  float ground_angle_threshold_;
  float minimum_range_;
  float sensor_mount_angle_;
  float theta_;
  size_t min_point_num_;
  size_t valid_point_num_;
  size_t min_line_num_;
  size_t valid_line_num_;
  float min_delta_Z_;
  float max_delta_Z_;
  float max_distance_;
  float max_elevation_;
  float max_dim_ratio_;
  float residuum_height_ratio_;
  float segment_alpha_X_;
  float segment_alpha_Y_;
  float sin_alpha_X_;
  float cos_alpha_X_;
  float sin_alpha_Y_;
  float cos_alpha_Y_;
  bool initialized_;
  bool has_intensity_values_;
  bool icp_residuals_set_;
  bool filter_within_bbox_;
  bool row_major_;
  bool is_organized_;
  float filter_margin_;

  std::string odom_frame_;
  std::string child_frame_;

  // Evaluation
  double projection_time_;
  double ground_removal_time_;
  double segmentation_time_;
  double compute_objects_time_;
  double tracking_time_;
  int dilate_kernel_size_;
  bool evaluate_;
  std::string evaluation_dir_;
  std::string output_dir_;
  std::string config_path_;
  std::unordered_map<std::string, AccumulatorData> time_stats_;

  // Methods
  void loadParams();
  void allocateMemory();
  void groundRemoval();
  void cloudSegmentation();
  void labelComponents(int row, int col);
  void resetParameters();
  Object getObject(const pcl::PointCloud<PointType>& cloud);
  void computeAllObjects();
  void trackDetections();
  void visualize();
  void setupEvaluation();
  void evaluate();

public:
  DetectionModule();
  ~DetectionModule() = default;

  void applySegmentation();
  void projectScan(const pcl::PointCloud<PointType>::Ptr& cloud_in, const pcl::PointCloud<PointType>::Ptr& cloud_in_t,
                   const Eigen::Matrix4f T, const Eigen::Matrix4f T_s2s);
  void projectResiduals(const pcl::PointCloud<PointType>::Ptr& cloud_in);

  void getIndices(std::vector<int>& indices, const std::vector<ObjectStatus> object_status);
  void getIndices(std::vector<int>& indices, const ObjectStatus object_status);
  void getGroundIndices(std::vector<int>& ground_indices);

  // Statistics
  size_t getSegmentsCount();
  size_t getObjectsCount();
  size_t getObjectsCount(const std::vector<ObjectStatus> object_status);
  size_t getObjectsCount(const ObjectStatus object_status);
  double getTime(std::string name, std::string type);
};
