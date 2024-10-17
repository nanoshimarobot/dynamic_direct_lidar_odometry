#include <util/trajectories_server.h>

TrajectoriesServer::TrajectoriesServer() : data_has_changed_(false)
{
  // Get parameters
  ros::NodeHandle private_nh("~");
  if (!private_nh.getParam("sub_topic", sub_topic_))
    ROS_ERROR("[TrajectoriesServer] Parameter \"sub_topic\" required");
  private_nh.param("pub_topic", pub_topic_, std::string("trajectories"));
  private_nh.param<float>("publish_rate", pub_rate_, 1.0);
  if (pub_rate_ <= 0)
  {
    ROS_ERROR(
        "[TrajectoriesServer] Parameter \"publish_rate\" must be positive. "
        "Falling back to default value 1Hz");
    pub_rate_ = 1.0;
  }

  bbox_sub_ =
      nh_.subscribe<jsk_recognition_msgs::BoundingBoxArray>(sub_topic_, 10, &TrajectoriesServer::callback, this);
  marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(pub_topic_, 10);
  timer_ = nh_.createTimer(ros::Duration(1.0 / pub_rate_), &TrajectoriesServer::timerCb, this);

  clear_trajs_srv_ = nh_.advertiseService("clear_trajectories", &TrajectoriesServer::clearTrajs, this);
  save_trajs_srv_ = nh_.advertiseService("save_trajectories", &TrajectoriesServer::saveTrajs, this);
}

void TrajectoriesServer::callback(const jsk_recognition_msgs::BoundingBoxArrayConstPtr& bboxes)
{
  for (const auto& box : bboxes->boxes)
  {
    // If no trajectory with this ID exists, create a new one
    trajectories_.try_emplace(box.label, box.label);

    // Update (append new position)
    geometry_msgs::Point point = box.pose.position;
    point.z -= box.dimensions.z / 2;  // Bottom of box
    trajectories_.at(box.label).update(point, box.header.stamp);
  }
  header_ = bboxes->header;
  data_has_changed_ = true;
}

void TrajectoriesServer::timerCb(const ros::TimerEvent& event)
{
  if (marker_pub_.getNumSubscribers() == 0)
    return;

  visualization_msgs::MarkerArray array;

  for (auto& it : trajectories_)
  {
    // At least 2 points required for LINE_STRIP marker
    if (it.second.getSize() < 2)
      continue;

    visualization_msgs::Marker marker;
    it.second.toMarker(marker, header_);
    array.markers.emplace_back(marker);
  }

  marker_pub_.publish(array);
  data_has_changed_ = false;
}

bool TrajectoriesServer::clearTrajs(ddlo_msgs::clear_trajectoriesRequest& req,
                                    ddlo_msgs::clear_trajectoriesResponse& res)
{
  ROS_INFO("[TrajectoriesServer] Clearing all saved trajectories");
  trajectories_.clear();

  // Send a DELETE marker to clear the rviz trajectories
  visualization_msgs::Marker delete_marker;
  delete_marker.action = visualization_msgs::Marker::DELETEALL;
  visualization_msgs::MarkerArray array;
  array.markers.push_back(delete_marker);
  marker_pub_.publish(array);

  res.success = true;
  return res.success;
}

bool TrajectoriesServer::saveTrajs(ddlo_msgs::save_trajectories::Request& req,
                                   ddlo_msgs::save_trajectories::Response& res)
{
  std::string path = req.save_path;
  ROS_INFO("[TrajectoriesServer] Writing trajectories to file %s", path.c_str());
  std::ofstream f;
  f.open(path);

  f << "# DDLO trajectory - format: x y z stamp.sec stamp.nsec" << std::endl;
  f << "trajectories: " << std::to_string(trajectories_.size()) << std::endl << std::endl;

  for (const auto& traj : trajectories_)
  {
    f << "id: " << std::to_string(traj.second.getId()) << std::endl;
    f << "points: " << std::to_string(traj.second.getSize()) << std::endl;

    auto points = traj.second.getPoints();
    auto stamps = traj.second.getStamps();

    if (points.size() != stamps.size())
    {
      ROS_ERROR("[TrajectoriesServer] Error while saving trajectory %i: Number of points and stamps must be equal.",
                traj.second.getId());
      f.close();
      res.success = false;
      return res.success;
    }

    // Write points stamped: x y z time.sec time.nsec
    for (size_t idx = 0; idx < points.size(); idx++)
    {
      f << points[idx].x << " " << points[idx].y << " " << points[idx].z << " " << std::to_string(stamps[idx].sec)
        << " " << std::to_string(stamps[idx].nsec) << std::endl;
    }

    f << std::endl;
  }
  f.close();
  res.success = true;
  ROS_INFO("[TrajectoriesServer] Done");
  return res.success;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectories_server_node");

  TrajectoriesServer server;
  ros::spin();

  return 0;
}
