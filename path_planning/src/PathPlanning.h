// Include guard to prevent multiple declarations
#ifndef PATHPLANNING_H
#define PATHPLANNING_H

// ROS header
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

// Namespace matches ROS package name
namespace path_planning {

  class PathPlanning {
  public:
    PathPlanning(ros::NodeHandle& n, ros::NodeHandle& pn);
    void run();

  private:
	// Node-specific stuff here
    void ResetVehicleControlMsg();
    
    ros::Publisher vehicle_control_publisher;
    geometry_msgs::Twist vehicle_control_msg;
  };

}
#endif // PATHPLANNING_H
