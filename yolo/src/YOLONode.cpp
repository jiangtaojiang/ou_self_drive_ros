// Header file for the class
#include "YOLONode.h"

#include <unistd.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <sstream>

#include "YOLO.h"

#define WEIGHTS_NAME "run.weights"
#define CONFIG_NAME "run.cfg"
#define IMAGE_TOPIC "/camera_front/image_raw"

// Namespace matches ROS package name
namespace yolo {

    // Constructor with global and private node handle arguments
    YOLONode::YOLONode(ros::NodeHandle& n, ros::NodeHandle& pn)
    {
		status_publisher = n.advertise<std_msgs::String>("status", 1);		
        detection_image_publisher = n.advertise<sensor_msgs::Image>("yolo/detections", 1);
		std_msgs::String message; 
		message.data = "Starting up...";
		status_publisher.publish(message);
        mError = false;
       
		if(InitializeYOLO())
		{
			printf("\nWinning!!!\n");
			sleep(1);			
			message.data = "YOLO successfully initialized";
		}
		else
		{
			printf("\nWell shit...\n");
			sleep(1);			
			message.data = "YOLO failed to initialize";
		}
		printf("\nPlease don't crash!!!\n");
			sleep(1);
		status_publisher.publish(message);
		printf("\nPlease don't crash!!!\n");
			sleep(1);
		try{
			image_subscriber = n.subscribe("/camera_front/image_raw", 1, &YOLONode::ImageTopicCallback, this);
		}catch(...)
		{
			printf("\nShit's broke my friend...\n");
		}
    }
    YOLONode::~YOLONode()
    {
        delete mYOLO;
    }

    bool YOLONode::InitializeYOLO()
    {
		printf("\nYOLOOOOO!!!\n");
		sleep(1);        
		mYOLO = new YOLO();
        
        char buff[512];
        memset(buff, 0, sizeof(buff));
        if (getcwd(buff, sizeof(buff)) == NULL)
        {
            mLastError = "Unable to determine working directory";
            return false;
        }

        std::string working_directory = buff;

        std::string weights_path = working_directory;
        weights_path += "/darknet/";
        weights_path += WEIGHTS_NAME;
        mYOLO->SetWeightsPath(weights_path);

        std::string config_path = working_directory;
        config_path += "/darknet/";
        config_path += CONFIG_NAME;
        mYOLO->SetConfigurationPath(config_path);

        mYOLO->SetDetectionThreshold(0.2);
        mYOLO->SetNMSThreshold(0.1);
        mYOLO->DrawDetections(true);

        if(!mYOLO->Initialize())
        {
            mLastError = "Failed to initialize YOLO. Are these paths correct?\nWeights:\t";
            mLastError += weights_path;
            mLastError += "\nConfig:\t";
            mLastError += config_path;
            return false;
        }
		printf("\nInit Finished\n");
        return true;
    }

	void YOLONode::ImageTopicCallback(const sensor_msgs::Image& image_msg)
	{   
        //we are just going to throw away images that come in while the network is running.
        //perhaps it would be better to buffer 1 image so that the network could always be 
        //running but.. eh.. that's future John's problem.
        if(mYOLO->IsRunning())
        {
            printf("Yolo Running, frame discarded\n");
            return;
        }
        else if(mError)
        {
            printf("Error Detected, frame discarded\n");
            return;  
        }
        
            
        
        
        cv_bridge::CvImagePtr cv_image;
        try 
        {
            cv_image = cv_bridge::toCvCopy(image_msg, "rgb8");
            printf("copied\n");
        } 
        catch (cv_bridge::Exception& e) 
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        IplImage* ipl_image = new IplImage(cv_image->image);
        try{
            printf("before detect\n");
            std::vector<Detection_t> detections = mYOLO->Detect(ipl_image);
            if(detections.size() > 0)
            {
                std::stringstream ss;
                ss << "Detection:\n";
                ss << "class\t\t" << detections[0].class_id;
                ss << "confidence\t\t" << detections[0].confidence;

                std_msgs::String message; 
                message.data = ss.str();
                status_publisher.publish(message);
            }
            else
            {
                std_msgs::String message; 
                message.data = "No Detections";
                status_publisher.publish(message);           
            }
        }
        catch(...)
        {
            mError = true;
            std_msgs::String message; 
            message.data = mYOLO->mLastError.c_str();
            while(true)
            {           
                status_publisher.publish(message);
                sleep(1000);  
            } 
        }
        
        
        delete ipl_image;
    }


}
