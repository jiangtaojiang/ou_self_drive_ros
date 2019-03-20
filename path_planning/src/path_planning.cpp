// ROS and node class header file
#include <ros/ros.h>
#include "PathPlanning.h"

int main(int argc, char** argv)
{
  // Initialize ROS and declare node handles
  ros::init(argc, argv, "PathPlanning");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  
  // Instantiate node class
  path_planning::PathPlanning node(n, pn);

  ros::Rate loop_rate(10);
  
  // Spin and process callbacks
  while(true)
  {
    node.run();
    ros::spinOnce();
    loop_rate.sleep();
  }
}
