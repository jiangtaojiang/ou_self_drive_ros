// Include guard to prevent multiple declarations
#ifndef PATHPLANNING_H
#define PATHPLANNING_H

// ROS header
#include <ros/ros.h>

// Namespace matches ROS package name
namespace path_planning {

  class PathPlanning {
  public:
    PathPlanning(ros::NodeHandle& n, ros::NodeHandle& pn);

  private:
	// Node-specific stuff here
  };

}
#endif // PATHPLANNING_H
