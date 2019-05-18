#include "GemSpeedControl.h"

namespace gem_speed_control {

GemSpeedControl::GemSpeedControl(ros::NodeHandle n, ros::NodeHandle pn)
{
  sub_cmd = n.subscribe("cmd_vel", 1, &GemSpeedControl::recvCmd, this);
  sub_feedback = n.subscribe("c_speed", 1, &GemSpeedControl::recvFeedback, this);
  pub_brake = n.advertise<std_msgs::Int16>("brake_cmd", 1);
  pub_throttle = n.advertise<std_msgs::Int16>("throttle_cmd", 1);
  
  cmd_speed = 0.0;
  error_int = 0.0;
  
  srv.setCallback(boost::bind(&GemSpeedControl::reconfig, this, _1, _2));
}

void GemSpeedControl::reconfig(SpeedControlConfig& config, uint32_t level)
{
  cfg = config;
  error_int = 0.0;
}

void GemSpeedControl::recvCmd(const geometry_msgs::TwistConstPtr& msg)
{
  cmd_speed = msg->linear.x;
}

void GemSpeedControl::recvFeedback(const std_msgs::Float32ConstPtr& msg)
{

  // Implement PI
  float error = cmd_speed - msg->data;
  error_int += error * cfg.sample_time;
  float throttle_pct = cfg.kp * error + cfg.ki * error_int;
  bool brakes = false;
  
  if (throttle_pct < 0) {
    throttle_pct = 0.0;
    brakes = true;
  } else if (throttle_pct >= 1.0) {
    throttle_pct = 1.0;
  }
  
  std_msgs::Int16 acc;
//  acc.data = (int)(180+32*throttle_pct);
  acc.data = (int)(180+32*cmd_speed);
  pub_throttle.publish(acc);
  
  std_msgs::Int16 brake;
  brake.data = 0;
  if(brakes) brake.data = 100;
  pub_brake.publish(brake);
}

}
