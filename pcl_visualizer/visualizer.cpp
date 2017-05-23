#include <iostream>
#include <ros/ros.h>
#include <cmath>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "pcl_ros/impl/transforms.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

int main (int argc, char* argv[])
{
    ros::init(argc, argv, "visualizer");
    ros::NodeHandle n;

    ros::Publisher pcd_pub = n.advertise<sensor_msgs::PointCloud2>("saved_map", 1000000);
    ros::Rate loop_rate(0.5);
//    for(int i = 0; i< argc; i++)
//    {
//        std::cout << i << " " << argv[i] << '\n';
//    }
    //load pcd file
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);

    if (pcl::io::loadPCDFile<pcl::PointXYZI> (argv[1], *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file %s \n", argv[1]);
        return (-1);
    }
    std::cout << "Loaded "
              << cloud->points.size()
              << " data points from"
              << argv[1]
              << std::endl;

    //downsample
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>());
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (0.2f, 0.2f, 0.2f);
    sor.filter (*cloud_filtered);
    std::cout <<cloud_filtered->points.size()<<std::endl;

    //todo add color to z axis
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
    colored_cloud->points.resize(cloud_filtered->points.size());
    for(size_t i = 0; i<cloud_filtered->points.size(); i++)
    {
        colored_cloud->points[i].x = cloud_filtered->points[i].x;
        colored_cloud->points[i].y = cloud_filtered->points[i].y;
        colored_cloud->points[i].z = cloud_filtered->points[i].z;
        colored_cloud->points[i].r = fmin(cloud_filtered->points[i].z/5 * 255,255);
        colored_cloud->points[i].g = 255;
        colored_cloud->points[i].b = 0;
    }

    //transform point_cloud to sensor_msgs
    sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*colored_cloud, *map_msg_ptr);

    while(ros::ok())
    {
        if(cloud_filtered->points.size()>1000000)
        {
            std::cout << "size too large: "<<cloud_filtered->points.size()<<std::endl;
            break;
        }
        map_msg_ptr->header.frame_id = "map";
        pcd_pub.publish(*map_msg_ptr);
        loop_rate.sleep();
    }
    return (0);
}

