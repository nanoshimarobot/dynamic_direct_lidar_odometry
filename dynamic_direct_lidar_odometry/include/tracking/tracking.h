#pragma once

#include <odometry/ddlo.h>
#include <tracking/bounding_box_filter.h>
#include <tracking/hungarian.h>

class TrackingModule
{
public:
  TrackingModule();
  ~TrackingModule() = default;

  void load_params();

  void update(float dt, const std::vector<Object>& detected_objects, const std_msgs::Header header);

  void associate(std::vector<Object> detected_objects, std::vector<Object> current_tracked_objects, float dt);

  static double getCostSimple(Object detected_object, Object current_tracked_object);
  static double getBBoxIoU(Object& detected_object, Object& current_tracked_object);
  static double getCostFull(Object detected_object, Object current_tracked_object);

  void getIndices(std::vector<int>& indices, const std::vector<ObjectStatus> object_status) const;
  void getIndices(std::vector<int>& indices, const ObjectStatus object_status) const;
  size_t getObjectsCount();
  size_t getObjectsCount(const std::vector<ObjectStatus> object_status);
  size_t getObjectsCount(const ObjectStatus object_status);

private:
  void generateAssociations();
  void publishBBoxes();

  size_t max_no_hit_before_removal_;
  size_t min_dynamic_hits_;
  size_t max_undefined_hits_;
  float max_object_velocity_;
  double min_travel_dist_;  // min distance that the object has to move
  double res_height_ratio_;
  std::string odom_frame_;
  std_msgs::Header header_;

  int filter_id_ = 0;

  std::vector<BoundingBoxFilter> filters_;
  HungarianAlgorithm hungarian_alg_;

  std::vector<int> assignment_;
  std::vector<std::pair<int, int>> matches_;
  std::vector<int> unmatched_detections_;
  std::vector<int> unmatched_states_;

  ros::NodeHandle nh_;
  ros::Publisher undefined_bbox_pub_;
  ros::Publisher static_bbox_pub_;
  ros::Publisher dynamic_bbox_pub_;
  ros::Publisher label_vis_pub_;
  ros::Publisher clear_map_pub_;
};
