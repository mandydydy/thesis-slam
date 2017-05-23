//
// Created by mengdi on 2017-04-13.
//

#ifndef OUTDOOR_SLAM_NDT_KEY_MAPPING_HPP
#define OUTDOOR_SLAM_NDT_KEY_MAPPING_HPP

#include <Eigen/Geometry>
#include <algorithm>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <runtime_manager/ConfigNdtMapping.h>
#include <runtime_manager/ConfigNdtMappingOutput.h>

#include <tf/transform_listener.h>

#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>

#include "pose_node.hpp"

class Ndt
{
public:
    Ndt();
    ~Ndt();
    tf::Transform g_transform;
    poseNode posnd;
    poseNode getPoseNode();
    void setPoseNode();
private:
    ros::NodeHandle nh, private_nh;

    Position previous_pos, guess_pos, current_pos, added_pos, first_pos;
    tf::Quaternion q;
    Eigen::Matrix4f t;
    double offset_x, offset_y, offset_z, offset_yaw; // current_pos - previous_pos
    nav_msgs::Odometry current_odom, prev_odom;
    double odom_x, odom_y, prev_odom_x, prev_odom_y, odom_theta, prev_odom_yaw;   //odom_offset
    ros::Time odom_time, Imu_time;     //scan time is velodyne callback time
    pcl::PointCloud<pcl::PointXYZI> map_global;

//---------------------Mengdi---------------------------------///
    bool use_keyframe_;
    bool num_of_frames_, distance_based_;     //switch between strategies to reinitialize local maps
    int keyframes_index = 0;
    int keyframes_length_;
    double DISTANCE, ROT;     //distance thresholod
    double distance_cond, rotation_cond;
    bool align_keyframes = 0;

    bool use_imu_, use_odom_, plus_;
    bool debug_odom_;

    std_msgs::Header header;
    geometry_msgs::Quaternion imu;

    pcl::PointCloud <pcl::PointXYZI> scan;
    pcl::PointCloud <pcl::PointXYZI> scan_selected;
    ros::Time scan_time;

    unsigned int pose_id;
    double pose_node_fitness;
    Eigen::Matrix4f pose_node_transform;
    std::vector<pcl::PointCloud<pcl::PointXYZI>> keyframes;  //for ndt registration
    pcl::PointCloud<pcl::PointXYZI> previous_keyframes;
    std::vector<pcl::PointCloud<pcl::PointXYZI>> target_frames;
    pcl::PointCloud<pcl::PointXYZI> target_cloud;

    Eigen::Matrix4f prev_key_trans;
    pcl::PointCloud<pcl::PointXYZI> local_cloud;
    pcl::PointCloud<pcl::PointXYZI> terrain_cloud;



/////////////////////// Xi  ///////////////////////////////////
    std::vector<pcl::PointCloud<pcl::PointXYZI>> map_local;        //Mengdi XYZI
    std::vector<pcl::PointCloud<pcl::PointXYZI>> map_terrain;   // for global map publish

// Pointshape_Processor ps_processor(360*4);

    int map_local_index = 0;
    int map_local_length_ = 20;  //5
    int map_number = 0;
    int target_frames_index = 0;
    int target_frames_length_ = 20;

    int map_terrain_index = 0;
    int map_terrain_length_ = 50;
    float shift_terrain = 0.05;

    tf::TransformListener* tfListener = NULL;
//////////////////////////////////////////////////////////////////////
    pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
// Default values
    int iter = 30; // Maximum iterations
    float ndt_res_ = 1.0; // Resolution
    double step_size_ = 0.1; // Step size
    double trans_eps = 0.01; // Transformation epsilon

// Leaf size of VoxelGrid filter.
    double voxel_leaf_size = 2.0;


    ros::Publisher ndt_map_pub;
    ros::Publisher current_pose_pub, guess_pose_pub, init_pose_pub;
    ros::Publisher ndt_stat_pub, pub_velodyne_base;
//---------------------Mengdi---------------------------------///
    ros::Publisher map_local_pub, target_frames_pub, whole_map_pub;    //publish local maps
    ros::Subscriber param_sub,private_output_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> private_points_sub;
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub;
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Imu, nav_msgs::Odometry> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<MySyncPolicy> *sync;

//---------------------Mengdi---------------------------------///
    int initial_scan_loaded = 0;

    double RANGE_MIN = 0.0, RANGE_MAX = 100.0;
    double SHIFT = 0.0, SHIFT_MIN;


    // functions
    void clear_pos();
    void accumulatePoseNode();  //merge transforms and fitness
    void param_callback(const runtime_manager::ConfigNdtMapping::ConstPtr& input);
    void output_callback(const runtime_manager::ConfigNdtMappingOutput::ConstPtr& input);
    //void points_callback(const sensor_msgs::PointCloud2::ConstPtr& input);
    void ImuOffset();
    void odomOffset();
    void callback(const sensor_msgs::PointCloud2::ConstPtr& input_pt,
                  const sensor_msgs::Imu::ConstPtr &input_imu,
                  const nav_msgs::Odometry::ConstPtr &input_odom);
    void getTransform(pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_source,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_target,
                      Eigen::Matrix4f initial);

    void addKeyframe(pcl::PointCloud<pcl::PointXYZI>::Ptr ptr);
    void addLocalMap(pcl::PointCloud<pcl::PointXYZI>::Ptr ptr);
    void addScissoredMap(pcl::PointCloud<pcl::PointXYZI>::Ptr ptr);
    void addTargetMap(pcl::PointCloud<pcl::PointXYZI>::Ptr ptr);

    void updateMaps(pcl::PointCloud<pcl::PointXYZI>::Ptr i_ptr,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr rgb_ptr);
    void updatePos();
    unsigned int getId();
    pcl::PointCloud<pcl::PointXYZI> get_local_map(std::vector<pcl::PointCloud<pcl::PointXYZI>> map_cloud);
//    pcl::PointCloud<pcl::PointXYZI> get_local_map(std::vector<pcl::PointCloud<pcl::PointXYZI>> map_cloud);
/*
 * void align_again(pcl::PointCloud<pcl::PointXYZI>::Ptr keyframes_ptr,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr prev_frames_ptr,
                      Eigen::Matrix4f init, unsigned int key_index);
*/
    void publishPosData(ros::Time time);


    void ndtKeyframe(pcl::PointCloud<pcl::PointXYZI>::Ptr i_ptr,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr rgb_ptr);
    void ndtIncre(pcl::PointCloud<pcl::PointXYZI>::Ptr i_ptr,
                  pcl::PointCloud<pcl::PointXYZI>::Ptr rgb_ptr);

};

#endif //OUTDOOR_SLAM_NDT_KEY_MAPPING_HPP

