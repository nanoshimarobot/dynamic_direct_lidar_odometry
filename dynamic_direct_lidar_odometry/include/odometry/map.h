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

// Changelog Jonathan Lichtenfeld, 2023:
// - integrate into dynamic_direct_lidar_odometry framework

#include "odometry/ddlo.h"

class MapNode
{
public:
  MapNode();
  ~MapNode() = default;

  static void abort()
  {
    abort_ = true;
  }

  void start();
  void stop();

private:
  void abortTimerCB(const ros::TimerEvent& e);
  void publishTimerCB(const ros::TimerEvent& e);

  void keyframeCB(const sensor_msgs::PointCloud2ConstPtr& keyframe);
  void dynamicObjectsCB(const jsk_recognition_msgs::BoundingBoxArrayConstPtr& bboxes);

  bool savePcd(ddlo_msgs::save_pcd::Request& req, ddlo_msgs::save_pcd::Response& res);

  void getParams();

  ros::NodeHandle nh_;
  ros::Timer abort_timer_;
  ros::Timer publish_timer_;

  ros::Subscriber keyframe_sub_;
  ros::Subscriber dynamic_objects_sub_;
  ros::Publisher map_pub_;
  ros::Publisher map_info_pub_;  // sends map size to odom node for print

  ros::ServiceServer save_pcd_srv_;

  pcl::PointCloud<PointType>::Ptr ddlo_map_;
  pcl::VoxelGrid<PointType> voxelgrid_;

  ros::Time map_stamp_;
  std::string odom_frame_;

  bool voxel_filter_use_;
  bool publish_full_map_;
  bool filter_bboxes_;
  double filter_margin_;
  double publish_freq_;
  double leaf_size_;

  static std::atomic<bool> abort_;
};
