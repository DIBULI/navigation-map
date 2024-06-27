#include "ros/ros.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include "occupancy_grid_map/ogm.hpp"


ros::Publisher gridsPub;

nav_msgs::Odometry currentOdom;

OccupancyGridMap *ogm;

void translateOGMMessage(pcl::PointCloud<pcl::PointXYZ> &cloud) {
  float x, y, z;
  int xIndex, yIndex, zIndex;
  for (size_t i = 0; i < ogm->mapGrids.size(); i++) {
    if (ogm->mapGrids[i].isOccupied()) {
      ogm->getGridIndex(i, xIndex, yIndex, zIndex);
      ogm->gridIndexToPosition(xIndex, yIndex, zIndex, x, y, z);
      pcl::PointXYZ point;
      point.x = x;
      point.y = y;
      point.z = z;
      cloud.push_back(point);
    }
  }
}

void vinsOdomCb(const nav_msgs::Odometry &odom) {
  currentOdom = odom;
}

void vinsPCCb(const sensor_msgs::PointCloud &pc) {
  std::vector<std::tuple<float, float, float>> grids;
  float x, y, z;
  unsigned int xIndex, yIndex, zIndex;
  for (auto p : pc.points) {
    x = p.x;
    y = p.y;
    z = p.z;
    grids.push_back(std::make_tuple(x, y, z));
  }
  ogm->updateMap(
    currentOdom.pose.pose.position.x,
    currentOdom.pose.pose.position.y,
    currentOdom.pose.pose.position.z,
    grids
  );
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "navigation_map_ros_adapter_vins_node");
  ros::NodeHandle nh;

  float mapResolution = 0.5;
  int mapXLength = 61;
  int mapYLength = 61;
  int mapZLength = 11;
  float mapOriginX = 0.0;
  float mapOriginY = 0.0;
  float mapOriginZ = 0.0;

  ogm = new OccupancyGridMap(mapOriginX, mapOriginY, mapOriginZ, mapResolution,
   -mapXLength / 2.0 + mapOriginX, mapXLength / 2.0 + mapOriginX, 
   -mapYLength / 2.0 + mapOriginY, mapYLength / 2.0 + mapOriginY, 
   -mapZLength / 2.0 + mapOriginZ, mapZLength / 2.0 + mapOriginZ);

  ros::Subscriber pointCloudSub = nh.subscribe("point_cloud", 1, vinsPCCb);
  ros::Subscriber vinsOdomSub = nh.subscribe("odometry", 1, vinsOdomCb);
  gridsPub = nh.advertise<sensor_msgs::PointCloud2>("grid_pointcloud", 1);

  sensor_msgs::PointCloud2 cloudMsg;
  
  ros::Rate rate(2);
  while (ros::ok()) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    translateOGMMessage(cloud);
    pcl::toROSMsg(cloud, cloudMsg);
    cloudMsg.header.stamp = ros::Time::now();
    cloudMsg.header.frame_id = "map";

    gridsPub.publish(cloudMsg);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}