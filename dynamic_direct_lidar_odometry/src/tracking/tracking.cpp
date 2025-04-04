#include <tracking/tracking.h>
#include <util/bbox_iou.h>

TrackingModule::TrackingModule()
{
  load_params();

  undefined_bbox_pub_ = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>("bboxes_undefined", 1);
  static_bbox_pub_ = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>("bboxes_static", 1);
  dynamic_bbox_pub_ = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>("bboxes_dynamic", 1);

  label_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("range_img/label_vis", 1);
  clear_map_pub_ = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>("clear_map", 1);
}

void TrackingModule::load_params()
{
  odom_frame_ = nh_.param<std::string>("odomFrame", "odom");
  max_no_hit_before_removal_ = nh_.param("odomNode/tracking/maxNoHits", 5);
  min_dynamic_hits_ = nh_.param("odomNode/tracking/minDynamicHits", 3);
  max_undefined_hits_ = nh_.param("odomNode/tracking/maxUndefinedHits", 10);
  max_object_velocity_ = nh_.param("odomNode/tracking/maxObjVelocity", 10);
  min_travel_dist_ = nh_.param("odomNode/tracking/minDistFromOrigin", 0.5);
  res_height_ratio_ = nh_.param("odomNode/detection/residuumHeightRatio", 0.0);
}

void TrackingModule::update(float dt, const std::vector<Object>& detected_objects, const std_msgs::Header header)
{
  header_ = header;
  matches_.clear();
  unmatched_detections_.clear();
  unmatched_states_.clear();
  std::vector<Object> current_tracked_objects;

  // Predict each filter and store it in current_tracked_objects
  for (BoundingBoxFilter& bbf : filters_)
  {
    bbf.predict(dt);
    Object tracked_object = bbf.getObject();
    current_tracked_objects.push_back(tracked_object);
  }

  // Get associations
  associate(detected_objects, current_tracked_objects, dt);

  // Update matched filters
  for (auto& idx_pair : matches_)
  {
    filters_[idx_pair.second].update(detected_objects[idx_pair.first], header);
  }

  // add filters for unmatched detections
  for (int& det_idx : unmatched_detections_)
  {
    // // skip those with low residuum  TODO is this done somewhere else already?
    // double min_required_avg_res = detected_objects[det_idx].state[6] * res_height_ratio_;
    // if (detected_objects[det_idx].avg_residuum < min_required_avg_res)
    //     continue;

    filters_.emplace_back(detected_objects[det_idx], filter_id_, min_travel_dist_, res_height_ratio_, min_dynamic_hits_,
                          max_undefined_hits_);
    filter_id_++;
  }

  // Erase filters that haven't been updated in a while. SSLU: steps since last
  // update
  for (auto it = filters_.begin(); it != filters_.end();)
  {
    if (it->getSSLU() >= max_no_hit_before_removal_)
      it = filters_.erase(it);
    else
      ++it;
  }

  publishBBoxes();  // TODO detach a thread for this

  ROS_INFO("track updated");
}

void TrackingModule::associate(std::vector<Object> detected_objects, std::vector<Object> current_tracked_objects,
                               float dt)
{
  // ROS_INFO("Associating objects...");
  if (current_tracked_objects.size() == 0)
  {
    unmatched_detections_ = std::vector<int>(detected_objects.size());
    // Fill with increasing integers
    std::iota(std::begin(unmatched_detections_), std::end(unmatched_detections_), 0);
    return;
  }

  if (detected_objects.size() == 0)
    return;

  // Create cost matrix
  std::vector<std::vector<double>> cost_matrix;
  std::vector<std::vector<float>> disp_matrix;

  for (int i = 0; i < detected_objects.size(); i++)
  {
    std::vector<double> cost_matrix_row;
    std::vector<float> disp_matrix_row;
    for (int j = 0; j < current_tracked_objects.size(); j++)
    {
      double cost = getCostFull(detected_objects[i], current_tracked_objects[j]);
      float displacement = getCostSimple(detected_objects[i], current_tracked_objects[j]);
      // ROS_INFO("cost: %f", cost);

      cost_matrix_row.push_back(cost);
      disp_matrix_row.push_back(displacement);
    }
    cost_matrix.push_back(cost_matrix_row);
    disp_matrix.push_back(disp_matrix_row);
  }

  assignment_.clear();
  // ROS_INFO("Hungarian algorithm...");
  double cost = hungarian_alg_.Solve(cost_matrix, assignment_);
  // ROS_INFO("Hungarian algorithm done");

  // get matched (!=1) / unmatched (-1) detections
  for (unsigned int i = 0; i < detected_objects.size(); i++)
  {
    if (assignment_[i] != -1)
      matches_.push_back(std::make_pair(i, assignment_[i]));
    else
      unmatched_detections_.push_back(i);
  }

  // get tracklets that are not matched
  for (unsigned int i = 0; i < current_tracked_objects.size(); i++)
  {
    if (std::find(assignment_.begin(), assignment_.end(), i) == assignment_.end())
      unmatched_states_.push_back(i);
  }

  // remove matches with disparity > threshold
  for (std::vector<std::pair<int, int>>::iterator it = matches_.begin(); it != matches_.end();)
  {
    if (disp_matrix[it->first][it->second] > max_object_velocity_ * dt)
    {
      unmatched_detections_.push_back(it->first);
      unmatched_states_.push_back(it->second);
      it = matches_.erase(it);
    }
    else
      ++it;
  }
  // ROS_INFO("Associating objects done");
}

double TrackingModule::getCostSimple(Object detected_object, Object current_tracked_object)
{
  Vector7d det_state = detected_object.state.head<7>();
  Vector7d tracked_state = current_tracked_object.state.head<7>();

  double dist = std::sqrt((det_state[0] - tracked_state[0]) * (det_state[0] - tracked_state[0]) +
                          (det_state[1] - tracked_state[1]) * (det_state[1] - tracked_state[1]) +
                          (det_state[2] - tracked_state[2]) * (det_state[2] - tracked_state[2]));
  return dist;
}

double TrackingModule::getBBoxIoU(Object& detected_object, Object& current_tracked_object)
{
  Vector7d det_state = detected_object.state.head<7>();
  Vector7d tracked_state = current_tracked_object.state.head<7>();

  double iou = OBBIoU(det_state, tracked_state);
  return iou;
}

double TrackingModule::getCostFull(Object detected_object, Object current_tracked_object)
{
  double bbox_cost = 1 - getBBoxIoU(detected_object, current_tracked_object);

  // ROS_INFO("bbox_cost: %f", bbox_cost);

  float np_det = (float)detected_object.num_points;
  float np_track = (float)current_tracked_object.num_points;
  float point_diff_rel_inv = (np_track >= np_det) ? np_det / np_track : np_track / np_det;
  float point_diff_rel = 1 - point_diff_rel_inv;

  float alpha = 0.1;  // TODO remove
  float beta = 0.8;
  float gamma = 0.1;

  double cost = beta * bbox_cost + gamma * point_diff_rel;

  return cost;
}

void TrackingModule::getIndices(std::vector<int>& indices, const std::vector<ObjectStatus> object_status) const
{
  indices.clear();

  for (auto& bbf : filters_)
  {
    const auto status = bbf.getObjectStatus();
    for (auto requested_status : object_status)
    {
      if (status == requested_status)
      {
        std::vector<int> bbf_indices = bbf.getIndices();
        indices.insert(indices.end(), bbf_indices.begin(), bbf_indices.end());
      }
    }
  }
}

void TrackingModule::getIndices(std::vector<int>& indices, const ObjectStatus object_status) const
{
  indices.clear();

  for (auto& bbf : filters_)
  {
    if (bbf.getObjectStatus() == object_status)
    {
      std::vector<int> bbf_indices = bbf.getIndices();
      indices.insert(indices.end(), bbf_indices.begin(), bbf_indices.end());
    }
  }
}

size_t TrackingModule::getObjectsCount(const std::vector<ObjectStatus> object_status)
{
  size_t count = 0;
  for (auto& bbf : filters_)
  {
    for (auto status : object_status)
    {
      if (bbf.getObjectStatus() == status)
      {
        count++;
        break;
      }
    }
  }
  return count;
}

size_t TrackingModule::getObjectsCount()
{
  return filters_.size();
}

size_t TrackingModule::getObjectsCount(const ObjectStatus object_status)
{
  size_t count = 0;
  for (auto& bbf : filters_)
  {
    if (bbf.getObjectStatus() == object_status)
      count++;
  }
  return count;
}

void TrackingModule::publishBBoxes()
{
  // Publish the bboxes history of objects that transitioned from static to dynamic
  jsk_recognition_msgs::BoundingBoxArray static_bboxes_history;
  for (auto& bbf : filters_)
  {
    jsk_recognition_msgs::BoundingBoxArray bboxArray;
    if (bbf.getStaticBBoxes(bboxArray))
    {
      for (auto& bbox : bboxArray.boxes)
      {
        bbox.header = header_;
        bbox.header.frame_id = odom_frame_;
      }

      static_bboxes_history.boxes.insert(static_bboxes_history.boxes.end(), bboxArray.boxes.begin(),
                                         bboxArray.boxes.end());
    }
  }

  if (!static_bboxes_history.boxes.empty())
  {
    static_bboxes_history.header = header_;
    static_bboxes_history.header.frame_id = odom_frame_;
    clear_map_pub_.publish(static_bboxes_history);
  }

  if (dynamic_bbox_pub_.getNumSubscribers() == 0 && static_bbox_pub_.getNumSubscribers() == 0 &&
      undefined_bbox_pub_.getNumSubscribers() == 0 && label_vis_pub_.getNumSubscribers() == 0)
    return;

  jsk_recognition_msgs::BoundingBoxArray undefined_bboxes;
  jsk_recognition_msgs::BoundingBoxArray dynamic_bboxes;
  jsk_recognition_msgs::BoundingBoxArray static_bboxes;
  visualization_msgs::MarkerArray label_marker;

  // Delete current label markers
  visualization_msgs::MarkerArray delete_array;
  visualization_msgs::Marker delete_marker;
  delete_marker.action = visualization_msgs::Marker::DELETEALL;
  for (const auto& status : std::vector<std::string>{ "UNDEFINED", "STATIC", "DYNAMIC" })
  {
    delete_marker.ns = status;
    delete_array.markers.push_back(delete_marker);
  }
  label_vis_pub_.publish(delete_array);

  for (auto& bbf : filters_)
  {
    const auto state = bbf.getFullState();

    // Copy object to bbox msg
    jsk_recognition_msgs::BoundingBox bbox;
    bbox.pose.position.x = state[0];
    bbox.pose.position.y = state[1];
    bbox.pose.position.z = state[2];
    bbox.pose.orientation.x = 0;
    bbox.pose.orientation.y = 0;
    bbox.pose.orientation.z = state[3];
    bbox.pose.orientation.w = std::cos(std::asin(state[3]));  // = sqrt(1-x**2)
    bbox.dimensions.x = state[4];
    bbox.dimensions.y = state[5];
    bbox.dimensions.z = state[6];
    bbox.label = bbf.getFilterID();
    bbox.value = bbf.getAvgResiduum();
    bbox.header.frame_id = odom_frame_;
    bbox.header.stamp = header_.stamp;

    switch (bbf.getObjectStatus())
    {
      case ObjectStatus::UNDEFINED:
        undefined_bboxes.boxes.push_back(bbox);
        break;
      case ObjectStatus::STATIC:
        static_bboxes.boxes.push_back(bbox);
        break;
      case ObjectStatus::DYNAMIC:
        dynamic_bboxes.boxes.push_back(bbox);
        break;
    }

    // Create text label
    if (label_vis_pub_.getNumSubscribers() > 0)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = odom_frame_;
      marker.header.stamp = header_.stamp;
      marker.id = bbf.getFilterID();
      marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      marker.action = visualization_msgs::Marker::ADD;
      marker.ns = bbf.getObjectStatusString();
      marker.pose.position.x = state[0];
      marker.pose.position.y = state[1];
      marker.pose.position.z = state[2] + state[6];
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 1.8;
      marker.scale.y = .15;
      marker.scale.z = .15;
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;
      std::stringstream text;
      text << std::setprecision(2) << std::fixed;
      text << "ID: " << bbf.getFilterID();
      float velocity = std::sqrt(state[7] * state[7] + state[8] * state[8] + state[9] * state[9]);
      text << std::endl << "v: " << velocity;
      text << std::endl << "hits: " << bbf.getHits();
      text << std::endl << "avg res: " << bbf.getAvgResiduum() << " height: " << state[6];
      text << std::endl << "dist_to_origin: " << bbf.getDistToOrigin() << std::fixed;
      if (bbf.getSSLU() > 0)
        text << " sslu: " << bbf.getSSLU();
      marker.text = text.str();
      marker.lifetime = ros::Duration(0);
      label_marker.markers.push_back(marker);
    }
  };

  if (undefined_bbox_pub_.getNumSubscribers() > 0)
  {
    undefined_bboxes.header = header_;
    undefined_bboxes.header.frame_id = odom_frame_;
    undefined_bbox_pub_.publish(undefined_bboxes);
  }

  if (static_bbox_pub_.getNumSubscribers() > 0)
  {
    static_bboxes.header = header_;
    static_bboxes.header.frame_id = odom_frame_;
    static_bbox_pub_.publish(static_bboxes);
  }

  if (dynamic_bbox_pub_.getNumSubscribers() > 0)
  {
    dynamic_bboxes.header = header_;
    dynamic_bboxes.header.frame_id = odom_frame_;
    dynamic_bbox_pub_.publish(dynamic_bboxes);
  }

  if (label_vis_pub_.getNumSubscribers() > 0)
    label_vis_pub_.publish(label_marker);
}
