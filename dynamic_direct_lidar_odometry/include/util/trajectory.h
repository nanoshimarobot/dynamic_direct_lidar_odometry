#pragma once
#include <odometry/ddlo.h>

class Trajectory
{
public:
  explicit Trajectory(int id);

  void update(geometry_msgs::Point point, ros::Time stamp);

  u_int getId() const;

  u_int getSize() const;

  void toMarker(visualization_msgs::Marker& msg, const std_msgs::Header& header);

  std::vector<geometry_msgs::Point> getPoints() const;

  std::vector<ros::Time> getStamps() const;

private:
  std::string status_;
  u_int id_;
  visualization_msgs::Marker marker_;
  std::vector<ros::Time> stamps_;
  std::vector<float> colors_{ .2, .3, .4, .5, .6, .7, .8, .9 };
};
