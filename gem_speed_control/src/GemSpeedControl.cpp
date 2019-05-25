#include "GemSpeedControl.h"

namespace gem_speed_control {

GemSpeedControl::GemSpeedControl(ros::NodeHandle n, ros::NodeHandle pn)
{
  sub_cmd = n.subscribe("cmd_vel", 1, &GemSpeedControl::recvCmd, this);
  sub_feedback = n.subscribe("c_speed", 1, &GemSpeedControl::recvFeedback, this);
  pub_brake = n.advertise<std_msgs::Int16>("brake_cmd", 1);
  pub_throttle = n.advertise<std_msgs::Int16>("throttle_cmd", 1);
  pub_steer = n.advertise<std_msgs::Float32>("steering_cmd", 1);
  cmd_steer = 0.0;
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
  cmd_steer = msg->angular.z;
}

void GemSpeedControl::recvFeedback(const std_msgs::Float32ConstPtr& msg)
{

  // Implement PI
  float error = (cmd_speed - msg->data) ;
  error_int += error * 0.05; //cfg.sample_time;
  cfg.kp = 1.6;  cfg.ki = 5 * cfg.kp;
  float throttle_pct = cfg.kp * error + cfg.ki*error_int;
  bool brakes = false;
  
  if (throttle_pct < 0) {
    throttle_pct = 0.0;
    brakes = true;
  } else if (throttle_pct >= 10.0) {
    throttle_pct = 10.0;
  }
  
  std_msgs::Int16 acc; // m/sec   1mile/hr->1/2.237 m/s
  acc.data = (int)(180 + 32.0 * throttle_pct);
  pub_throttle.publish(acc);
  
//  if(brakes) {
//    std_msgs::Int16 brake;
//    brake.data = 0;
//    brake.data = 100;
//    pub_brake.publish(brake);
//  }
  std_msgs::Float32 steer;// rad/sec
  steer.data = (float)cmd_steer * 180.0 / 3.141592 / cfg.sample_time;
  pub_steer.publish(steer);
}

}
