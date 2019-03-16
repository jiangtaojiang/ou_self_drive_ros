// Include guard to prevent multiple declarations
#pragma once

// ROS header
#include <ros/ros.h>
#include <string>
#include <sensor_msgs/Image.h>

class YOLO;
// Namespace matches ROS package name
namespace yolo {

  class YOLONode {

  public:
    YOLONode(ros::NodeHandle& n, ros::NodeHandle& pn);
    ~YOLONode();

    std::string mLastError;

  private:
	// Node-specific stuff here
        bool InitializeYOLO();
		void ImageTopicCallback(const sensor_msgs::Image&);
        YOLO* mYOLO;

		ros::Publisher status_publisher;
		ros::Subscriber image_subscriber;
        ros::Publisher detection_image_publisher;
        bool mError;

  };



}

