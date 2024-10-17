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

#include "odometry/map.h"

std::atomic<bool> MapNode::abort_(false);

MapNode::MapNode()
{
  getParams();

  abort_timer_ = nh_.createTimer(ros::Duration(0.01), &MapNode::abortTimerCB, this);

  if (publish_full_map_)
    publish_timer_ = nh_.createTimer(ros::Duration(publish_freq_), &MapNode::publishTimerCB, this);

  keyframe_sub_ = nh_.subscribe("keyframe", 1, &MapNode::keyframeCB, this);

  if (filter_bboxes_)
    dynamic_objects_sub_ = nh_.subscribe("clear_map", 10, &MapNode::dynamicObjectsCB, this);

  map_info_pub_ = nh_.advertise<std_msgs::Int32>("map_info", 1);
  map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("map", 1);

  save_pcd_srv_ = nh_.advertiseService("save_pcd", &MapNode::savePcd, this);

  // initialize clouds
  ddlo_map_ = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);

  ROS_INFO("DLO Map Node Initialized");
}

void MapNode::getParams()
{
  odom_frame_ = nh_.param<std::string>("odomFrame", "odom");
  publish_full_map_ = nh_.param("mapNode/publishFullMap", true);

  publish_freq_ = nh_.param("mapNode/publishFreq", 1.0);
  leaf_size_ = nh_.param("mapNode/leafSize", 0.5);
  voxel_filter_use_ = nh_.param("mapNode/useVoxelFilter", true);
  filter_bboxes_ = nh_.param("mapNode/filterBboxHistory", false);
  filter_margin_ = nh_.param("mapNode/filterMargin", 0.0);
}

void MapNode::start()
{
  ROS_INFO("Starting DLO Map Node");
}

void MapNode::stop()
{
  ROS_WARN("Stopping DLO Map Node");

  // shutdown
  ros::shutdown();
}

void MapNode::abortTimerCB(const ros::TimerEvent& e)
{
  if (abort_)
    stop();
}

void MapNode::publishTimerCB(const ros::TimerEvent& e)
{
  if (map_pub_.getNumSubscribers() > 0)
  {
    sensor_msgs::PointCloud2 map_ros;
    pcl::toROSMsg(*ddlo_map_, map_ros);
    map_ros.header.stamp = ros::Time::now();
    map_ros.header.frame_id = odom_frame_;
    map_pub_.publish(map_ros);
  }
  if (map_info_pub_.getNumSubscribers() > 0)
  {
    std_msgs::Int32 info;
    info.data = ddlo_map_->height * ddlo_map_->width;
    map_info_pub_.publish(info);
  }
}

void MapNode::keyframeCB(const sensor_msgs::PointCloud2ConstPtr& keyframe)
{
  // convert scan to pcl format
  pcl::PointCloud<PointType>::Ptr keyframe_pcl = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
  pcl::fromROSMsg(*keyframe, *keyframe_pcl);

  // voxel filter
  if (voxel_filter_use_)
  {
    voxelgrid_.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    voxelgrid_.setInputCloud(keyframe_pcl);
    voxelgrid_.filter(*keyframe_pcl);
  }

  // save keyframe to map
  map_stamp_ = keyframe->header.stamp;
  *ddlo_map_ += *keyframe_pcl;

  // Publish keyframe only if map not activated
  if (!publish_full_map_ && map_pub_.getNumSubscribers() > 0)
  {
    if (keyframe_pcl->points.size() == keyframe_pcl->width * keyframe_pcl->height)
    {
      sensor_msgs::PointCloud2 map_ros;
      pcl::toROSMsg(*keyframe_pcl, map_ros);
      map_ros.header.stamp = ros::Time::now();
      map_ros.header.frame_id = odom_frame_;
      map_pub_.publish(map_ros);
    }
  }
}

void MapNode::dynamicObjectsCB(const jsk_recognition_msgs::BoundingBoxArrayConstPtr& bboxes)
{
  // remove points within object bboxes
  pcl::CropBox<PointType> crop;
  crop.setNegative(true);
  crop.setInputCloud(ddlo_map_);

  auto size_before = ddlo_map_->size();
  for (const auto& bbox : bboxes->boxes)
  {
    float crop_min_x = -bbox.dimensions.x / 2 - filter_margin_;
    float crop_min_y = -bbox.dimensions.y / 2 - filter_margin_;
    float crop_min_z = -bbox.dimensions.z / 2 - filter_margin_;
    float crop_max_x = bbox.dimensions.x / 2 + filter_margin_;
    float crop_max_y = bbox.dimensions.y / 2 + filter_margin_;
    float crop_max_z = bbox.dimensions.z / 2 + filter_margin_;
    crop.setMin(Eigen::Vector4f(crop_min_x, crop_min_y, crop_min_z, 1.0));
    crop.setMax(Eigen::Vector4f(crop_max_x, crop_max_y, crop_max_z, 1.0));
    crop.setTranslation(Eigen::Vector3f(bbox.pose.position.x, bbox.pose.position.y, bbox.pose.position.z));
    crop.setRotation(Eigen::Vector3f(0, 0, 2 * std::asin(bbox.pose.orientation.z)));
    crop.filter(*ddlo_map_);
    auto size_after = ddlo_map_->size();
  }
}

bool MapNode::savePcd(ddlo_msgs::save_pcd::Request& req, ddlo_msgs::save_pcd::Response& res)
{
  pcl::PointCloud<PointType>::Ptr m =
      pcl::PointCloud<PointType>::Ptr(boost::make_shared<pcl::PointCloud<PointType>>(*ddlo_map_));

  float leaf_size = req.leaf_size;
  std::string path = req.save_path + "/ddlo_map.pcd";

  std::cout << std::setprecision(2) << "Saving map to " << path << "... ";
  std::cout.flush();

  // voxelize map
  if (leaf_size > 0)
  {
    std::cout << " applying voxelgrid filter of size " << std::setprecision(2) << leaf_size << std::endl;
    pcl::VoxelGrid<PointType> vg;
    vg.setLeafSize(leaf_size, leaf_size, leaf_size);
    vg.setInputCloud(m);
    vg.filter(*m);
  }

  // save map
  int ret = pcl::io::savePCDFileBinary(path, *m);
  res.success = ret == 0;

  if (res.success)
    std::cout << "done" << std::endl;
  else
    std::cout << "failed" << std::endl;

  return res.success;
}
