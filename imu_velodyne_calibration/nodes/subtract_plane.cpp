#include <iostream>
#include <ros/ros.h>
#include <sstream>
#include <fstream>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>

using namespace ros;
std::ofstream plane_a;
std::ofstream plane_b;
std::ofstream plane_c;
std::ofstream plane_d;

class planeFinder
{
public:
    planeFinder();
    ~planeFinder();

private:
    NodeHandle nh, private_nh;
    Subscriber velodyne_sub;
    Publisher plane_pub, scan_pub;
    double Y_MIN_, Y_MAX_;
    bool debug_;
    void velodyneCallback(const sensor_msgs::PointCloud2::ConstPtr& input);
    void findPlane(pcl::PointCloud<pcl::PointXYZ> cloud,
                   const sensor_msgs::PointCloud2::ConstPtr &input);

};
planeFinder::planeFinder():private_nh("~")
{
    if(!private_nh.getParam("Y_MIN", Y_MIN_))
        Y_MIN_ = -0.5;
    if(!private_nh.getParam("Y_MAX", Y_MAX_))
        Y_MAX_ = 0.5;
    if(!private_nh.getParam("debug", debug_))
        debug_ = false;
    velodyne_sub = private_nh.subscribe("/velodyne_points", 1, &planeFinder::velodyneCallback, this);
    plane_pub = private_nh.advertise<sensor_msgs::PointCloud2>("/plane", 1000);
    scan_pub = private_nh.advertise<sensor_msgs::PointCloud2>("/scan", 1000);
}

planeFinder::~planeFinder()
{
    plane_a.close();
    plane_b.close();
    plane_c.close();
    plane_d.close();
}


void planeFinder::velodyneCallback(const sensor_msgs::PointCloud2::ConstPtr &input)
{
    ros::Time scan_time = input->header.stamp;
    std::cout<<"Veldodyne time: "<<scan_time<<std::endl;

    pcl::PointCloud<pcl::PointXYZ> scan, tmp;
    pcl::fromROSMsg (*input, tmp);

    pcl::PointXYZ p;
    int cloud_index = 0;
    for (pcl::PointCloud<pcl::PointXYZ>::const_iterator item = tmp.begin(); item != tmp.end(); item++)
    {
        p.x = (double) item->x;
        p.y = (double) item->y;
        p.z = (double) item->z;

        if(p.x > 0 && p.y < Y_MAX_ && p.y > Y_MIN_)
        {
            scan.push_back(p);
        }
        cloud_index++;
    }
    if(debug_)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZ>(scan));
        sensor_msgs::PointCloud2::Ptr scan_msg_ptr(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*scan_ptr, *scan_msg_ptr);
        scan_msg_ptr->header.frame_id = input->header.frame_id;
        scan_pub.publish(scan_msg_ptr);
    }

    findPlane(scan, input);
}

void planeFinder::findPlane(pcl::PointCloud<pcl::PointXYZ> scan,
                            const sensor_msgs::PointCloud2::ConstPtr &input)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>(scan));

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::PointCloud<pcl::PointXYZ> inlier_cloud;
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    } else
    {
        for (size_t i = 0; i < inliers->indices.size (); ++i)
        inlier_cloud.push_back(cloud->points[inliers->indices[i]]);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(inlier_cloud));
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud_ptr);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (0.1);

    // Compute the features
    ne.compute (*cloud_normals);
    std::cerr << *cloud_normals << std::endl;



    std::cerr << "Model coefficients: "
              << coefficients->values[0] << " "
              << coefficients->values[1] << " "
              << coefficients->values[2] << " "
              << coefficients->values[3] << std::endl;

    plane_a<< coefficients->values[0] <<"\n";
    plane_b << coefficients->values[1] <<"\n";
    plane_c << coefficients->values[2] <<"\n";
    plane_d << coefficients->values[3] <<"\n";

    pcl::PointCloud<pcl::PointXYZ>::Ptr inliers_ptr(new pcl::PointCloud<pcl::PointXYZ>(inlier_cloud));
    sensor_msgs::PointCloud2::Ptr plane_ptr(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*inliers_ptr, *plane_ptr);
    plane_ptr->header.frame_id = input->header.frame_id;
    plane_pub.publish(*plane_ptr);

}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "planeFinder");
    planeFinder finder;
    ros::Rate loop_rate(9);

    plane_a.open("plane_a.txt");
    plane_b.open("plane_b.txt");
    plane_c.open("plane_c.txt");
    plane_d.open("plane_d.txt");
    
    while (ok())
    {
        spinOnce();
    }

  return 0;
}
