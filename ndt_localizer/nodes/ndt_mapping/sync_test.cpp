#include <iostream>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

using namespace message_filters;

void callback(const sensor_msgs::PointCloud2::ConstPtr& input_pt,
             const sensor_msgs::Imu::ConstPtr &input_imu,
             const nav_msgs::Odometry::ConstPtr &input_odom)
{
  // Solve all of perception here...
    std::cout <<"vt: "<< input_pt->header.stamp <<std::endl;
	std::cout <<"imu: "<< input_imu->header.stamp <<std::endl;
	std::cout << "odom: "<< input_odom->header.stamp <<std::endl;
    std::cout << "------------------------------------" <<std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_node");

  ros::NodeHandle nh;
  message_filters::Subscriber<sensor_msgs::PointCloud2> pt_sub(nh, "/velodyne_points",1);
  message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh, "/imu/data/transform", 1);
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "/atrv/odom", 1);

  typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Imu, nav_msgs::Odometry> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(30), pt_sub, imu_sub, odom_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));
  ROS_WARN("before spin");

  ros::spin();

  return 0;
}
