#include <ros/ros.h>
#include "GemSpeedControl.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gem_speed_control");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  gem_speed_control::GemSpeedControl node(n, pn);

  ros::spin();
}
