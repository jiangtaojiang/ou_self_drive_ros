// Header file for the class
#include "Perception.h"
#include <string>
#include "visualization_msgs/Marker.h"


// Namespace matches ROS package name
namespace perception {

  // Constructor with global and private node handle arguments
  Perception::Perception(ros::NodeHandle& n, ros::NodeHandle& pn)
  {
      ros::Subscriber lidar1 = n.subscribe("/LIDAR/FrontCenter", 1, &Perception::LIDARCallback0, this);
      lidar_subs.push_back(lidar1);
  }

  void Perception::LIDARCallback(int id, const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& point_cloud)
  {
    
    if(lidar_id_to_update_id.find(id) != lidar_id_to_update_id.end())
      point_map.RemoveUpdate(lidar_id_to_update_id[id]);

    std::vector<Point> map_points;
    for( auto point : point_cloud->points )
    {
        Point map_point( point.x, point.y, point.z );
        map_points.push_back(map_point);
    }

    int update_id = point_map.AddLIDARPoints(map_points);
    lidar_id_to_update_id[id] = update_id;
  }

  void Perception::LIDARCallback0(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& point_cloud)
  {
      LIDARCallback(0, point_cloud);
  }
  

}
