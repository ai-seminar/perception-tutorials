#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void cb_(const sensor_msgs::PointCloud2ConstPtr& inputCloud)
{

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>());

  pcl::fromROSMsg(*inputCloud,*cloud_in );
  // pcl::PCDWriter writer;
  ROS_INFO("I got a new point cloud size = %d",cloud_in->points.size());
  //writer.writeASCII("test.pcd", *cloud_in);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/camera/depth_registered/points", 10, cb_);
  ros::spin();
  return 0;
}
