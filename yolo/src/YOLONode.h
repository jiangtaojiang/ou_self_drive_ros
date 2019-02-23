// Include guard to prevent multiple declarations
#pragma once

// ROS header
#include <ros/ros.h>

// Namespace matches ROS package name
namespace yolo {

  class YOLONode {

  public:
    YOLONode(ros::NodeHandle& n, ros::NodeHandle& pn);

  private:
	// Node-specific stuff here

  };



}

