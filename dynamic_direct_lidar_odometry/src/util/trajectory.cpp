#include <util/trajectory.h>

Trajectory::Trajectory(int id) : id_(id)
{
  // Setup marker properties with pseudo random color
  marker_.type = visualization_msgs::Marker::LINE_STRIP;
  marker_.color.a = 1.0;
  // marker_.color.r = colors_[id_ % colors_.size()];
  // marker_.color.g = colors_[id_ * id_ % colors_.size()];
  // marker_.color.b = colors_[id_ * id_ * id_ % colors_.size()];
  marker_.color.r = 0.0;
  marker_.color.g = 0.0;
  marker_.color.b = 1.0;

  marker_.scale.x = 0.04;
  marker_.pose.orientation.w = 1.0;
}

void Trajectory::update(geometry_msgs::Point point, ros::Time stamp)
{
  marker_.points.emplace_back(point);
  stamps_.emplace_back(stamp);
}

u_int Trajectory::getId() const
{
  return id_;
}

u_int Trajectory::getSize() const
{
  return marker_.points.size();
}

void Trajectory::toMarker(visualization_msgs::Marker& msg, const std_msgs::Header& header)
{
  marker_.header = header;
  marker_.id = id_;
  marker_.ns = "dynamic";
  msg = marker_;
}

std::vector<geometry_msgs::Point> Trajectory::getPoints() const
{
  return marker_.points;
}

std::vector<ros::Time> Trajectory::getStamps() const
{
  return stamps_;
}
