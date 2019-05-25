// Include guard to prevent multiple declarations
#pragma once

// ROS header
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "PointMap.hpp"
#include "pcl_ros/point_cloud.h"

// Namespace matches ROS package name
namespace perception {

  class Perception {
  public:
    Perception(ros::NodeHandle& n, ros::NodeHandle& pn);

  private:
    std::vector< ros::Subscriber > lidar_subs;
    void LIDARCallback(int id, const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&);
    void LIDARCallback0(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&);
    
    PointMap point_map;

    std::map<int, int> lidar_id_to_update_id;
  };

}