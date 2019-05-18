#ifndef GEMSPEEDCONTROL_H
#define GEMSPEEDCONTROL_H

#include <ros/ros.h>

// Command
#include <geometry_msgs/Twist.h>

// Feedback and outputs
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>


// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <gem_speed_control/SpeedControlConfig.h>

namespace gem_speed_control
{

class GemSpeedControl
{
public:
  GemSpeedControl(ros::NodeHandle n, ros::NodeHandle pn);

private:
  void reconfig(SpeedControlConfig& config, uint32_t level);
  void recvCmd(const geometry_msgs::TwistConstPtr& msg);
  void recvFeedback(const std_msgs::Float32ConstPtr& msg);
  
  ros::Subscriber sub_cmd;
  ros::Subscriber sub_feedback;
  ros::Publisher pub_throttle;
  ros::Publisher pub_brake;
  dynamic_reconfigure::Server<SpeedControlConfig> srv;
  SpeedControlConfig cfg;
  
  float cmd_speed;
  float error_int;

};

}

#endif // GEMSPEEDCONTROL_H
