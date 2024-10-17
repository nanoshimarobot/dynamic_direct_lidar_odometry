#pragma once

#include <odometry/ddlo.h>

class TrajectoriesServer
{
public:
  TrajectoriesServer();

  ~TrajectoriesServer() = default;

private:
  void callback(const jsk_recognition_msgs::BoundingBoxArrayConstPtr& bboxes);

  void timerCb(const ros::TimerEvent& event);

  bool saveTrajs(ddlo_msgs::save_trajectoriesRequest& req, ddlo_msgs::save_trajectoriesResponse& res);

  bool clearTrajs(ddlo_msgs::clear_trajectoriesRequest& req, ddlo_msgs::clear_trajectoriesResponse& res);

  ros::Subscriber bbox_sub_;
  ros::Publisher marker_pub_;
  ros::Timer timer_;

  ros::ServiceServer clear_trajs_srv_;
  ros::ServiceServer save_trajs_srv_;

  ros::NodeHandle nh_;

  std::unordered_map<int, Trajectory> trajectories_;

  std_msgs::Header header_;

  std::string pub_topic_;
  std::string sub_topic_;
  float pub_rate_;
  bool data_has_changed_;
};
