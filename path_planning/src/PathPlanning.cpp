// Header file for the class
#include "PathPlanning.h"
#include <geometry_msgs/Twist.h>

// Namespace matches ROS package name
namespace path_planning {

  // Constructor with global and private node handle arguments
  PathPlanning::PathPlanning(ros::NodeHandle& n, ros::NodeHandle& pn)
  {
    vehicle_control_publisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ResetVehicleControlMsg();

    // Initialize a timer and bind its trigger callback to the 'timerCallback' method
    timer_ = n.createTimer(ros::Duration(0.1), &PathPlanning::timerCallback, this);
    
  }
  void PathPlanning::timerCallback(const ros::TimerEvent& event)
  {
    // Code here will execute every time the timer triggers.
    // The 'event' argument contains some useful timestamps that can be used
    //    in some useful situations, particularly populating timestamps in ROS messages
  }
  void PathPlanning::run()
  {
    //debug testing
    vehicle_control_msg.linear.x = 5;

    //publish all of our topics.
    vehicle_control_publisher.publish(vehicle_control_msg);
  }
  void PathPlanning::ResetVehicleControlMsg()
  {
    vehicle_control_msg.linear.x = 0;
    vehicle_control_msg.linear.y = 0;
    vehicle_control_msg.linear.z = 0;
    vehicle_control_msg.angular.x = 0;
    vehicle_control_msg.angular.y = 0;
    vehicle_control_msg.angular.z = 0;
  }

}
