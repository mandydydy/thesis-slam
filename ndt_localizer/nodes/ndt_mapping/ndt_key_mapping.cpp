/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
 Localization and mapping program using Normal Distributions Transform
 
 Yuki KITSUKAWA
 */

#define OUTPUT // If you want to output "position_log.txt", "#define OUTPUT".

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <cmath>
#include <limits>

#include <std_msgs/Bool.h>
//#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include "pcl_ros/impl/transforms.hpp"

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>

#include "ndt_key_mapping.hpp"

// #include <pointshape_processor.h>

// global variables
//output files
/*std::ofstream currentPos_x;
std::ofstream currentPos_y;
std::ofstream currentPos_z;
std::ofstream currentPos_roll;
std::ofstream currentPos_pitch;
std::ofstream currentPos_yaw;
std::ofstream fitness;
*/
Ndt::Ndt() : private_nh("~")
{
    clear_pos();
    // setting parameters
    private_nh.getParam("ndt_res", ndt_res_);
    private_nh.getParam("step_size", step_size_);
    private_nh.getParam("range_max", RANGE_MAX);
    private_nh.getParam("range_min", RANGE_MIN);
    std::cout << "RANGE_MAX: " << RANGE_MAX << std::endl;
    std::cout << "RANGE_MIN: " << RANGE_MIN << std::endl;
    private_nh.getParam("shift", SHIFT);
    std::cout << "SHIFT: " << SHIFT << std::endl;
    private_nh.getParam("shift_min", SHIFT_MIN);
    std::cout << "SHIFT_MIN: " << SHIFT_MIN << std::endl;
///////////////////////////////////// Xi ////////////////////////
    //-------------------------Mengdi start------------------------------//
    if(!private_nh.getParam("debug_odom", debug_odom_))
        debug_odom_= false;
    if(!private_nh.getParam("use_imu", use_imu_))
        use_imu_ = false;
    std::cout << "use_imu: " << use_imu_ << std::endl;
    if(!private_nh.getParam("use_odom", use_odom_))
        use_odom_ = false;
    std::cout << "use_odom: " << use_odom_ << std::endl;
    if(!private_nh.getParam("plus", plus_))
        plus_ = false;

    private_nh.getParam("map_local_length", map_local_length_);
    std::cout << "map_local_length: " << map_local_length_ << std::endl;

    if(!private_nh.getParam("use_keyframe",use_keyframe_))
        use_keyframe_ = false;
    if(use_keyframe_)
    {
        std::cout<<"use_keyframe: "<<use_keyframe_<<std::endl;
        private_nh.getParam("keyframes_length", keyframes_length_);
        std::cout << "keyframes_length: " << keyframes_length_ << std::endl;

        private_nh.getParam("num_of_frames", num_of_frames_);
        if(num_of_frames_)
        {
            std::cout << "num_of_frames: "<< num_of_frames_ << std::endl;
        }
        private_nh.getParam("distance_based", distance_based_);
        if(distance_based_)
        {
            std::cout << "distance_based: " << distance_based_<< std::endl;
            private_nh.getParam("distance", DISTANCE);
            std::cout << "DISTANCE: " << DISTANCE << std::endl;
            private_nh.getParam("rot", ROT);
            std::cout << "ROT: " << ROT << std::endl;
        }
        if(num_of_frames_==distance_based_)
        {
            ROS_ERROR("only one criterion can be chosen");
        }
    } else{
        private_nh.getParam("target_frames_length", target_frames_length_);
        std::cout<<"target_frames_length: "<< target_frames_length_ <<std::endl;
    }
    //only for show map in rviz
    private_nh.getParam("map_terrain_length", map_terrain_length_);
    std::cout << "map_terrain_length: " << map_terrain_length_ << std::endl;
    private_nh.getParam("shift_terrain", shift_terrain);
    std::cout << "shift_terrain: " << shift_terrain << std::endl;
//////////////////////////////////////////////////////////////////////
    //--------------------------Mengdi end-------------------------------//
    tfListener = new (tf::TransformListener);
    //map_terrain.header.frame_id = "map";//todo

    ndt_map_pub = private_nh.advertise<sensor_msgs::PointCloud2>("/scissored_map", 1000);
    current_pose_pub = private_nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 1000);
    guess_pose_pub = private_nh.advertise<geometry_msgs::PoseStamped>("/guess_pose", 1000);
    init_pose_pub = private_nh.advertise<geometry_msgs::PoseStamped>("/init_pose", 1000);
    pub_velodyne_base = private_nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points_base", 1);
//-------------------------Mengdi start-------------------------------------//
    target_frames_pub = private_nh.advertise<sensor_msgs::PointCloud2>("/target_frames", 1000);
    map_local_pub = private_nh.advertise<sensor_msgs::PointCloud2>("/local_map", 1000);
    whole_map_pub = private_nh.advertise<sensor_msgs::PointCloud2>("/whole_map", 1000);
//-------------------------Mengdi end---------------------------------------//
    param_sub = private_nh.subscribe("/config/ndt_mapping", 10, &Ndt::param_callback, this);
    private_output_sub = private_nh.subscribe("/config/ndt_mapping_output", 10, &Ndt::output_callback, this);

    private_points_sub.subscribe(private_nh, "/velodyne_points", 1);
    imu_sub.subscribe(private_nh, "/imu/data/transform", 10);
    odom_sub.subscribe(private_nh, "/atrv/odom", 10);

    sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(40), private_points_sub, imu_sub, odom_sub);
    sync->registerCallback(boost::bind(&Ndt::callback, this, _1, _2, _3));

}
Ndt::~Ndt()
{
   /* currentPos_x.close();
    currentPos_y.close();
    currentPos_z.close();
    currentPos_roll.close();
    currentPos_pitch.close();
    currentPos_yaw.close();
    fitness.close();*/
}
void Ndt::clear_pos()
{
    previous_pos.x = 0.0;
    previous_pos.y = 0.0;
    previous_pos.z = 0.0;
    previous_pos.roll = 0.0;
    previous_pos.pitch = 0.0;
    previous_pos.yaw = 0.0;

    current_pos.x = 0.0;
    current_pos.y = 0.0;
    current_pos.z = 0.0;
    current_pos.roll = 0.0;
    current_pos.pitch = 0.0;
    current_pos.yaw = 0.0;

    guess_pos.x = 0.0;
    guess_pos.y = 0.0;
    guess_pos.z = 0.0;
    guess_pos.roll = 0.0;
    guess_pos.pitch = 0.0;
    guess_pos.yaw = 0.0;

    added_pos.x = 0.0;
    added_pos.y = 0.0;
    added_pos.z = 0.0;
    added_pos.roll = 0.0;
    added_pos.pitch = 0.0;
    added_pos.yaw = 0.0;

    offset_x = 0.0;
    offset_y = 0.0;
    offset_z = 0.0;
    offset_yaw = 0.0;

    odom_theta = 0.0;
    odom_x = 0.0;
    odom_y = 0.0;
    prev_odom_yaw = 0.0;
    prev_odom_x = 0.0;
    prev_odom_y = 0.0;

    t = Eigen::Matrix4f::Identity();

    pose_id = -1;
    pose_node_transform.setIdentity();
    pose_node_fitness = 1;
}

void Ndt::ImuOffset()
{
    //todo calculate offset
}

void Ndt::odomOffset()
{
    odom_x = current_odom.pose.pose.position.x - prev_odom.pose.pose.position.x;
    odom_y = current_odom.pose.pose.position.y - prev_odom.pose.pose.position.y;

    double w, x, y, z;
    w = current_odom.pose.pose.orientation.w;
    x = current_odom.pose.pose.orientation.x;
    y = current_odom.pose.pose.orientation.y;
    z = current_odom.pose.pose.orientation.z;
    Eigen::Quaterniond odom_quat(w, x, y, z);


    Eigen::Matrix3d odom_mat = odom_quat.matrix();

    tf::Matrix3x3 odom_tf;
    odom_tf.setValue(static_cast<double>(odom_mat(0, 0)), static_cast<double>(odom_mat(0, 1)),
                     static_cast<double>(odom_mat(0, 2)),
                     static_cast<double>(odom_mat(1, 0)), static_cast<double>(odom_mat(1, 1)),
                     static_cast<double>(odom_mat(1, 2)),
                     static_cast<double>(odom_mat(2, 0)), static_cast<double>(odom_mat(2, 1)),
                     static_cast<double>(odom_mat(2, 2)));
    double odom_roll, odom_pitch, odom_yaw;
    odom_tf.getRPY(odom_roll, odom_pitch, odom_yaw, 0);

    odom_theta = odom_yaw - prev_odom_yaw;
    prev_odom_yaw = odom_yaw;

//    ROS_WARN("odom offset: %f, %f, %f", odom_x, odom_y, odom_theta);

}

/////////////////////// Xi  ///////////////////////////////////
//merge map_cloud vector into a pcl::PointCloud<pcl::PointXYZI> type variable
pcl::PointCloud<pcl::PointXYZI> Ndt::get_local_map(std::vector<pcl::PointCloud<pcl::PointXYZI>> map_cloud)
{
    pcl::PointCloud<pcl::PointXYZI> map_points;

    for(unsigned int i = 0; i< map_cloud.size(); i++)
    {
        map_points += map_cloud[i];
    }

    map_points.header = map_global.header;

    if(map_points.points.size() == 0)
    std::cout << "empty !" << std::endl;
    else
    std::cout << map_cloud.size() << "  " << map_points.points.size() << std::endl;

    return map_points;
}

/*pcl::PointCloud<pcl::PointXYZI> Ndt::get_local_map(std::vector<pcl::PointCloud<pcl::PointXYZI>> map_cloud)
{
    pcl::PointCloud<pcl::PointXYZI> map_points;

    for(unsigned int i = 0; i< map_cloud.size(); i++)
    {
        map_points += map_cloud[i];
    }

    map_points.header = map_global.header;

    if(map_points.points.size() == 0)
    std::cout << "empty !" << std::endl;
    else
    std::cout << map_cloud.size() << "  " << map_points.points.size() << std::endl;

    return map_points;
}
*/
void Ndt::addKeyframe(pcl::PointCloud<pcl::PointXYZI>::Ptr ptr)
{
    keyframes.push_back(*ptr);
    keyframes_index ++;
}

void Ndt::addLocalMap(pcl::PointCloud<pcl::PointXYZI>::Ptr ptr)
{
    map_local.push_back(*ptr);
    map_local_index ++;
    if(map_local_index == map_local_length_)
    {
//        std::string filename;
//        filename = "local_map" + std::to_string(map_number) +".pcd";
//        pcl::io::savePCDFileASCII (filename, local_cloud);
//        map_number++;
        map_local.erase(map_local.begin());

    }
}

void Ndt::addScissoredMap(pcl::PointCloud<pcl::PointXYZI>::Ptr ptr)
{
    //update scissored map_terrain
    if(map_terrain_index < map_terrain_length_)
    {
        map_terrain.push_back(*ptr);
        map_terrain_index ++;
    }
    else if(map_terrain_index == map_terrain_length_)
    {
        map_terrain.erase(map_terrain.begin());
        map_terrain.push_back(*ptr);
    }
}

void Ndt::addTargetMap(pcl::PointCloud<pcl::PointXYZI>::Ptr ptr)
{
    //update target frames
    if(target_frames_index < target_frames_length_)
    {
        target_frames.push_back(*ptr);
        target_frames_index ++;
    }
    else if(target_frames_index == target_frames_length_)
    {
        target_frames.erase(target_frames.begin());
        target_frames.push_back(*ptr);
    }
}

//----------------------------------------Mengdi-------------------------------------------//
/*void Ndt::align_again(pcl::PointCloud<pcl::PointXYZI>::Ptr keyframes_ptr,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr prev_frames_ptr,
                    Eigen::Matrix4f init,
                    unsigned int key_index)
{
    std::cout<<"align again"<<std::endl;
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_frames_ptr (new pcl::PointCloud<pcl::PointXYZI>());

    // Apply voxelgrid filter
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
    voxel_grid_filter.setInputCloud(keyframes_ptr);
    voxel_grid_filter.filter(*filtered_frames_ptr);

    ndt.setStepSize(step_size_);
    ndt.setResolution(ndt_res_);
    ndt.setMaximumIterations(iter);

    ndt.setInputSource(filtered_frames_ptr);
    ndt.setInputTarget(prev_frames_ptr);

    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    ndt.align(*output_cloud, init);

    Eigen::Matrix4f t_tmp = ndt.getFinalTransformation();
    //calculate calibrated transformation matrix
    Eigen::Matrix3f rot_cal; rot_cal = prev_key_trans.block<3,3>(0,0) * t_tmp.block<3,3>(0,0);
    Eigen::Vector3f tr_cal; tr_cal = t_tmp.block<3,1>(0,3) + prev_key_trans.block<3,1>(0,3);
    Eigen::Matrix4f t_cal;
    t_cal << rot_cal, tr_cal,
            Eigen::RowVector3f::Zero(),1;

    pcl::transformPointCloud(*keyframes_ptr, *keyframes_ptr, t_cal);
    pcl::transformPointCloud(local_cloud, local_cloud, t_cal);

    //transform scissored map
    pcl::PointCloud<pcl::PointXYZI> terrain_clean;
    pcl::PointCloud<pcl::PointXYZI> terrain_points;
    for(unsigned int i = map_terrain.size()-key_index;i < map_terrain.size(); i++)
    {
        terrain_points += map_terrain[i];
        map_terrain.pop_back();
    }
    pcl::transformPointCloud(terrain_points, terrain_points, t_tmp);
    terrain_cloud = get_local_map(map_terrain) + terrain_points;

    tf::Matrix3x3 tf3d;

    tf3d.setValue(static_cast<double>(t_cal(0, 0)), static_cast<double>(t_cal(0, 1)), static_cast<double>(t_cal(0, 2)),
          static_cast<double>(t_cal(1, 0)), static_cast<double>(t_cal(1, 1)), static_cast<double>(t_cal(1, 2)),
          static_cast<double>(t_cal(2, 0)), static_cast<double>(t_cal(2, 1)), static_cast<double>(t_cal(2, 2)));

    // Update calibrated pos.
    Position pos;
    current_pos.x = t_cal(0, 3);
    current_pos.y = t_cal(1, 3);
    current_pos.z = t_cal(2, 3);
    tf3d.getRPY(current_pos.roll, pos.pitch, pos.yaw, 1);

    // Calculate the offset (current_pos - previous_pos)
    double x = current_pos.x - init(0,3);
    double y = current_pos.y - init(1,3);
    double z = current_pos.z - init(2,3);

    double translation = sqrt(pow(x,2.0)+pow(y,2.0)+pow(z,2.0));
    ROS_WARN("Adjusted %f on current keyframes",translation);

}
*/
void Ndt::getTransform(pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_source,
                           pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_target,
                           Eigen::Matrix4f initial)
{
    std::cout<<"get transform"<<std::endl;
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_source_ptr (new pcl::PointCloud<pcl::PointXYZI>());
    // Apply voxelgrid filter
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
    voxel_grid_filter.setInputCloud(ptr_source);
    voxel_grid_filter.filter(*filtered_source_ptr);     //original: filtered_scan_ptr

    //std::cout << "source: " << ptr_source->points.size() <<" points." <<std::endl;
    //std::cout << "filtered source: " << filtered_source_ptr->points.size() <<" points." <<std::endl;
    //std::cout << "target: " << ptr_target->points.size() <<" points." <<std::endl;
    ndt.setTransformationEpsilon(trans_eps);
    ndt.setStepSize(step_size_);
    ndt.setResolution(ndt_res_);
    ndt.setMaximumIterations(iter);
    ndt.setInputSource(filtered_source_ptr);      //make ndt faster
//----------------------Mengdi------------------------///
//  ndt.setInputTarget(map_ptr);                //previous code: map current scan to the whole map
    ndt.setInputTarget(ptr_target);

    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    ndt.align(*output_cloud, initial);

    t = ndt.getFinalTransformation();

    std::cout << "NDT has converged: " << ndt.hasConverged() << std::endl;
    if (ndt.getTransformationProbability() < 5)
        std::cout << "Transform Probability: " << ndt.getTransformationProbability() << std::endl;
    else
        ROS_WARN("Transform Probability: %f", ndt.getTransformationProbability());
   // fitness<<ndt.getTransformationProbability()<<"\n";
    std::cout << "Number of iteration: " << ndt.getFinalNumIteration() << std::endl;
//    std::cout << "Transformation Matrix:" << std::endl;
//    std::cout << ndt.getFinalTransformation() << std::endl;
}

//---------------------------------------------------------------------------------------------//
//////////////////////////////////////////////////////////////////////
void Ndt::param_callback(const runtime_manager::ConfigNdtMapping::ConstPtr& input)
{
    std::cout << "param call back: "<< std::endl;
    ndt_res_ = input->resolution;
    step_size_ = input->step_size;
    trans_eps = input->trans_eps;
    voxel_leaf_size = input->leaf_size;

    std::cout << "param_callback" << std::endl;
    std::cout << "ndt_res: " << ndt_res_ << std::endl;
    std::cout << "step_size: " << step_size_ << std::endl;
    std::cout << "trans_eps: " << trans_eps << std::endl;
    std::cout << "voxel_leaf_size: " << voxel_leaf_size << std::endl;
}
//what does this function do?
void Ndt::output_callback(const runtime_manager::ConfigNdtMappingOutput::ConstPtr& input)
{
    std::cout << "output callback" << std::endl;
    double filter_res = input->filter_res;
    std::string filename = input->filename;
    std::cout << "output_callback" << std::endl;
    std::cout << "filter_res: " << filter_res << std::endl;
    std::cout << "filename: " << filename << std::endl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(map_global));
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_filtered(new pcl::PointCloud<pcl::PointXYZI>());
    map_ptr->header.frame_id = "map";
    map_filtered->header.frame_id = "map";
    sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);

    // Apply voxelgrid filter
    if(filter_res == 0.0)
    {
        std::cout << "Original: " << map_ptr->points.size() << " points." << std::endl;
        pcl::toROSMsg(*map_ptr, *map_msg_ptr);
    }
    else
    {
        pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
        voxel_grid_filter.setLeafSize(filter_res, filter_res, filter_res);
        voxel_grid_filter.setInputCloud(map_ptr);
        voxel_grid_filter.filter(*map_filtered);
        std::cout << "Original: " << map_ptr->points.size() << " points." << std::endl;
        std::cout << "Filtered: " << map_filtered->points.size() << " points." << std::endl;
        pcl::toROSMsg(*map_filtered, *map_msg_ptr);
    }

    std::cout << "publishing map" << std::endl;
    ndt_map_pub.publish(*map_msg_ptr);

    // Writing Point Cloud data to PCD file
    if(voxel_leaf_size == 0.0){
    pcl::io::savePCDFileASCII(filename, *map_ptr);
    std::cout << "Saved " << map_ptr->points.size() << " data points to " << filename << "." << std::endl;
    }else{
    pcl::io::savePCDFileASCII(filename, *map_filtered);
    std::cout << "Saved " << map_filtered->points.size() << " data points to " << filename << "." << std::endl;
    }
}
void Ndt::updateMaps(pcl::PointCloud<pcl::PointXYZI>::Ptr i_ptr,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr rgb_ptr)
{
    std::cout << "updateMaps" << std::endl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr trans_i_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr trans_rgb_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    ////////////////////////////////// xi /////////////////////////////////////////

    pcl::transformPointCloud(*i_ptr, *trans_i_ptr, t);      //transforn orignial scan
    pcl::transformPointCloud(*rgb_ptr, *trans_rgb_ptr, t);   //transform scan within a range
    ///////////////////////////////////////////////////////////////////////////////
    //map_global += *transformed_i_ptr;
    if(use_keyframe_)
    {
        ndtKeyframe(trans_i_ptr, trans_rgb_ptr);
    } else
    {
        ndtIncre(trans_i_ptr, trans_rgb_ptr);
    }

    //publish maps
    addScissoredMap(trans_rgb_ptr);
    if(use_keyframe_)
    {
        std::cout<<"KEYFRAME: "<<keyframes_index<<"th frame" << std::endl;
        std::cout<<"LOCAL MAP: "<<map_local_index<<"th frame" << std::endl;
    }

    //publish scissored global map
    terrain_cloud = get_local_map(map_terrain);
    sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(terrain_cloud, *map_msg_ptr);
    map_msg_ptr->header.frame_id = "map";
    ndt_map_pub.publish(*map_msg_ptr);
//------------------------------------Mengdi--start------------------------------------//
    //publsih target frames
    if(use_keyframe_)
    {
        target_cloud = get_local_map(keyframes);     //add the current transformed scan
    } else
    {
        //
        target_cloud = get_local_map(target_frames);
    }
    sensor_msgs::PointCloud2::Ptr target_msg_ptr(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(target_cloud, *target_msg_ptr);
    target_msg_ptr->header.frame_id = "map";
    target_frames_pub.publish(*target_msg_ptr);

    //publish local map
    local_cloud = get_local_map(map_local);
    sensor_msgs::PointCloud2::Ptr local_msg_ptr(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(local_cloud, *local_msg_ptr);
    local_msg_ptr->header.frame_id = "map";
    map_local_pub.publish(*local_msg_ptr);

    std::cout << "target: " << target_frames.size() << " frames." << std::endl;
    std::cout << "local: " << map_local.size() << " frames." << std::endl;
    //publish global map
/*  sensor_msgs::PointCloud2::Ptr whole_map_msg_ptr(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(map_global,*whole_map_msg_ptr);
    whole_map_msg_ptr->header.frame_id = "map";
    whole_map_pub.publish(*whole_map_msg_ptr);
*/
//------------------------------------Mengdi--end------------------------------------//
}
//add frames
void Ndt::ndtKeyframe(pcl::PointCloud<pcl::PointXYZI>::Ptr i_ptr,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr rgb_ptr)
{
    if((num_of_frames_ && (map_local_index < map_local_length_))
       || (distance_based_ && (distance_cond < DISTANCE && rotation_cond < ROT)))

    {
        addLocalMap(rgb_ptr);
        if(keyframes_index < keyframes_length_)
        {
            addKeyframe(i_ptr);
        }
        else if(map_local_index == keyframes_length_ && keyframes_index == keyframes_length_)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr keyframes_ptr(new pcl::PointCloud<pcl::PointXYZI>(get_local_map(keyframes)));
            if(align_keyframes)
            {
                //calculate transformation difference between current scan and last pre_key_trans
                Eigen::Matrix3f prev_rot = prev_key_trans.block<3,3>(0,0);
                Eigen::Matrix3f rot = t.block<3,3>(0,0);
                Eigen::Matrix3f rot_diff = prev_rot.inverse()*rot;
                Eigen::Vector3f translate = t.block<3,1>(0,3);
                Eigen::Vector3f prev_translate = prev_key_trans.block<3,1>(0,3);
                Eigen::Vector3f translate_diff = translate - prev_translate;
                Eigen::Matrix4f init_diff;
                init_diff << rot_diff, translate_diff,
                        Eigen::RowVector3f::Zero(), 1;
                //align keyframes again

                pcl::PointCloud<pcl::PointXYZI>::Ptr prev_frames_ptr(new pcl::PointCloud<pcl::PointXYZI>(previous_keyframes));
                //align_again(keyframes_ptr, prev_frames_ptr, init_diff, keyframes_index);
                previous_keyframes = *keyframes_ptr;

            }
            else
            {
                previous_keyframes = *keyframes_ptr;
                prev_key_trans = t;
                align_keyframes = 1;
            }

        }
    }
    else
    {
        ROS_WARN("initialize local map");

        keyframes.clear();
        keyframes_index = 0;
        addKeyframe(i_ptr);
        prev_key_trans = t;

        map_local.clear();
        map_local_index = 0;
        addLocalMap(rgb_ptr);

        distance_cond = 0;
        rotation_cond = 0;
    }
}
void Ndt::ndtIncre(pcl::PointCloud<pcl::PointXYZI>::Ptr i_ptr,
                   pcl::PointCloud<pcl::PointXYZI>::Ptr rgb_ptr)
{
    addTargetMap(i_ptr);
    if(map_local_index < map_local_length_)
    {
        addLocalMap(rgb_ptr);
    } else
    {
        //save local map
        map_local.clear();
        map_local_index = 0;
        addLocalMap(rgb_ptr);
    }
}

void Ndt::updatePos()
{
//    std::cout << "update pos" << std::endl;
    tf::Transform transform;
    tf::Matrix3x3 tf3d;
    tf3d.setValue(static_cast<double>(t(0, 0)), static_cast<double>(t(0, 1)), static_cast<double>(t(0, 2)),
                  static_cast<double>(t(1, 0)), static_cast<double>(t(1, 1)), static_cast<double>(t(1, 2)),
                  static_cast<double>(t(2, 0)), static_cast<double>(t(2, 1)), static_cast<double>(t(2, 2)));

    // Update current_pos.
    current_pos.x = t(0, 3);
    current_pos.y = t(1, 3);
    current_pos.z = t(2, 3);
    tf3d.getRPY(current_pos.roll, current_pos.pitch, current_pos.yaw, 1);
    //std::cout<<"roll, pitch, yaw: "<<current_pos.roll<" "<<current_pos.pitch<<" "<<current_pos.yaw<<" "<<std::endl;
    transform.setOrigin(tf::Vector3(current_pos.x, current_pos.y, current_pos.z));
    q.setRPY(current_pos.roll, current_pos.pitch, current_pos.yaw);
    transform.setRotation(q);

    g_transform = transform;

    //the first transform after last keyframe
    if(keyframes_index == keyframes_length_)
    {
        if(map_local_index == keyframes_index)
        {
            std::cout<<"first pos"<<std::endl;
            first_pos.x = current_pos.x;
            first_pos.y = current_pos.y;
            first_pos.z = current_pos.z;
            first_pos.yaw = current_pos.yaw;
        }

        distance_cond = sqrt(pow(current_pos.x - first_pos.x,2.0)
                             +pow(current_pos.y - first_pos.y,2.0)
                             +pow(current_pos.z - first_pos.z,2.0));
        rotation_cond = abs(current_pos.yaw - first_pos.yaw);
        ROS_WARN("Distance: %f relative to keyframes", distance_cond);
        ROS_WARN("Rotation: %f relative to keyframes", rotation_cond);
    }
    // Calculate the offset (curren_pos - previous_pos)
    offset_x = current_pos.x - previous_pos.x;
    offset_y = current_pos.y - previous_pos.y;
    offset_z = current_pos.z - previous_pos.z;
    offset_yaw = current_pos.yaw - previous_pos.yaw;
    //ROS_WARN("offsets: %f, %f, %f", offset_x, offset_y, offset_yaw);

    // Calculate the shift between added_pos and current_pos
//    double shift = sqrt(pow(current_pos.x-added_pos.x, 2.0) + pow(current_pos.y-added_pos.y, 2.0));

//-----------------------------------------Mengdi start-------------------------------------------///
//    if(shift >= SHIFT)
//    {
//      //map_global += *transformed_scan_ptr;
//      added_pos.x = current_pos.x;
//      added_pos.y = current_pos.y;
//      added_pos.z = current_pos.z;
//      added_pos.roll = current_pos.roll;
//      added_pos.pitch = current_pos.pitch;
//      added_pos.yaw = current_pos.yaw;



///////////////////////// Xi /////////////////////////
//      map_local_index ++;
//      if(map_local_index == map_local_length_)
//            map_local_index = 0;
//      std::cout << map_local_index << std::endl;
//      map_local[map_local_index] = *transformed_scan_ptr;
/////////////////////////////////////////////////////

//-----------------------------------------Mengdi-end--------------------------------------------///

    //q.setRPY(current_pos.roll, current_pos.pitch, current_pos.yaw);

    publishPosData(scan_time);
}
void Ndt::publishPosData(ros::Time time)
{
//    std::cout<<"Publish pos" <<std::endl;
    // Update position and posture. current_pos -> previous_pos
    previous_pos.x = current_pos.x;
    previous_pos.y = current_pos.y;
    previous_pos.z = current_pos.z;
    previous_pos.roll = current_pos.roll;
    previous_pos.pitch = current_pos.pitch;
    previous_pos.yaw = current_pos.yaw;
    //publish pose
    geometry_msgs::PoseStamped current_pose_msg;

    current_pose_msg.header.frame_id = "map";
    current_pose_msg.header.stamp = time;
    current_pose_msg.pose.position.x = current_pos.x;
    current_pose_msg.pose.position.y = current_pos.y;
    current_pose_msg.pose.position.z = current_pos.z;
    current_pose_msg.pose.orientation.x = q.x();
    current_pose_msg.pose.orientation.y = q.y();
    current_pose_msg.pose.orientation.z = q.z();
    current_pose_msg.pose.orientation.w = q.w();

    current_pose_pub.publish(current_pose_msg);

    /*currentPos_x<<current_pos.x<<"\n";
    currentPos_y<<current_pos.y<<"\n";
    currentPos_z<<current_pos.z<<"\n";
    currentPos_roll<<current_pos.roll<<"\n";
    currentPos_pitch<<current_pos.pitch<<"\n";
    currentPos_yaw<<current_pos.yaw<<"\n";
    */
    std::cout << "(x,y,z,roll,pitch,yaw):" << std::endl;
    std::cout << "(" << current_pos.x << ", " << current_pos.y << ", " << current_pos.z << ", " << current_pos.roll << ", " << current_pos.pitch << ", " << current_pos.yaw << ")" << std::endl;


 }
void Ndt::callback(const sensor_msgs::PointCloud2::ConstPtr& input_pt,
                   const sensor_msgs::Imu::ConstPtr &input_imu,
                   const nav_msgs::Odometry::ConstPtr &input_odom)
{
    std::cout<<"-----------------callback-----------------"<<std::endl;
    std::cout << "Sequence number: " << input_pt->header.seq << std::endl;
    std::cout<< "point cloud: "<<input_pt->header.stamp<<std::endl;
    std::cout<< "imu        : "<<input_imu->header.stamp<<std::endl;
    std::cout<< "odom       : "<<input_odom->header.stamp<<std::endl;

    //get current odom
    prev_odom = current_odom;
    current_odom.pose= input_odom->pose;
    odomOffset();
    //get current imu orientation
    imu = input_imu->orientation;

    header = input_pt->header;
    //pcl::PointCloud <pcl::PointXYZI> filtered_single_scan;
    //   filtered_single_scan = ps_processor.process_velodyne(input, tfListener);
    //   filtered_single_scan.header.frame_id = "base_link";

    double r;   //distance to velodyne
    pcl::PointXYZI p;
    pcl::PointCloud <pcl::PointXYZI> tmp, tmp_rgb;

    scan_time = input_pt->header.stamp;

    tf::StampedTransform velodyne_to_map;
    tfListener->waitForTransform("/base_link", input_pt->header.frame_id, ros::Time(0), ros::Duration(0.5));
    tfListener->lookupTransform("/base_link", input_pt->header.frame_id, ros::Time(0), velodyne_to_map);

    sensor_msgs::PointCloud2 cloud_map;
    Eigen::Matrix4f eigen_transform;
    pcl_ros::transformAsMatrix(velodyne_to_map, eigen_transform);
    pcl_ros::transformPointCloud(eigen_transform, *input_pt, cloud_map);

    // cloud_map.header.frame_id = "base_link";
    // pub_velodyne_base.publish(cloud_map);

    cloud_map.header.frame_id = "map";

////////////////////////////////// xi /////////////////////////////////////////
    pcl::fromROSMsg(cloud_map, tmp);
    pcl::fromROSMsg(cloud_map, tmp_rgb);
///////////////////////////////////////////////////////////////////////////////

    int cloud_index = 0;
    scan.clear();
    scan_selected.clear();
    for (pcl::PointCloud<pcl::PointXYZI>::const_iterator item = tmp.begin(); item != tmp.end(); item++) {
        p.x = (double) item->x;
        p.y = (double) item->y;
        p.z = (double) item->z;
        p.intensity = (double) item->intensity;

        r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
        if ( r > RANGE_MIN )
        {
            scan.push_back(p);
        }

        // select points for other process
        if (r < RANGE_MAX && r > RANGE_MIN && p.z < 2)
        {
            scan_selected.push_back(tmp_rgb.points[cloud_index]);
        }
        cloud_index++;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));
    pcl::PointCloud<pcl::PointXYZI>::Ptr scan_selected_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan_selected));
    // Add initial point cloud to velodyne_map
    if(initial_scan_loaded == 0)
    {
        ROS_INFO("initialize the map");
        initial_scan_loaded = 1;
///////////////////////////////////////////////
//-----------------------------------Mengdi-start------------------------------------------///
       // map_global += *scan_ptr;
        if(use_keyframe_)
        {
            addKeyframe(scan_ptr);
        }
        else
        {
            addTargetMap(scan_ptr);
        }
        addLocalMap(scan_selected_ptr);
        addScissoredMap(scan_selected_ptr);
//-----------------------------------Mengdi-end------------------------------------------///
    }
    //------------------------------------------Mengdi--start-----------------------------------------------------///
    //if this scan is not the first, get target_cloud
    if(use_keyframe_)
    {
        target_cloud = get_local_map(keyframes);
    }
    else
    {
        std::cout << "get target frames: " << std::endl;
        target_cloud = get_local_map(target_frames);
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr target_ptr(new pcl::PointCloud<pcl::PointXYZI>(target_cloud));

    /*
     * calculate initial guess
     * if use imu, get roll pitch yaw from imu
     */

    if(!use_imu_)
    {   //use previous motion as current one
        if(!use_odom_)
        {
            guess_pos.x = previous_pos.x + offset_x;
            guess_pos.y = previous_pos.y + offset_y;

            guess_pos.yaw = previous_pos.yaw + offset_yaw;
            std::cout << "offset guess: " << guess_pos.x << " " << guess_pos.y << " " << guess_pos.yaw << std::endl;

        }
        else
        {
            if(plus_)
            {
                guess_pos.x = previous_pos.x + odom_x;
                guess_pos.y = previous_pos.y + odom_y;
            } else
            {
                guess_pos.x = previous_pos.x - odom_x;
                guess_pos.y = previous_pos.y - odom_y;
            }

            guess_pos.yaw = previous_pos.yaw + odom_theta;
            std::cout << "odom guess: " << guess_pos.x << " " << guess_pos.y << " " << guess_pos.yaw << std::endl;
        }

        guess_pos.z = previous_pos.z + offset_z;
        guess_pos.roll = previous_pos.roll;
        guess_pos.pitch = previous_pos.pitch;

    }
    else {
        if (!use_odom_) {
            guess_pos.x = previous_pos.x + offset_x;
            guess_pos.y = previous_pos.y + offset_y;
            guess_pos.yaw = previous_pos.yaw + offset_yaw;
            std::cout << "offset guess: " << guess_pos.x << " " << guess_pos.y << " " << guess_pos.yaw << std::endl;

        } else {
            //ndt_localizer::getOdom srv;
            //getOdom(srv);
            if(plus_)
            {
                guess_pos.x = previous_pos.x + odom_x;
                guess_pos.y = previous_pos.y + odom_y;
            } else
            {
                guess_pos.x = previous_pos.x - odom_x;
                guess_pos.y = previous_pos.y - odom_y;
            }
            guess_pos.yaw = previous_pos.yaw + odom_theta;
            std::cout << "odom guess: " << guess_pos.x << " " << guess_pos.y << " " << guess_pos.yaw << std::endl;
        }
        guess_pos.z = previous_pos.z;

        tf::Matrix3x3 imu_tf;
        Eigen::Matrix3d imu_mat;
        Eigen::Quaterniond imu_b(imu.w, imu.x, imu.y, imu.z);
        imu_mat = imu_b.matrix();
        imu_tf.setValue(static_cast<double>(imu_mat(0, 0)), static_cast<double>(imu_mat(0, 1)),
                        static_cast<double>(imu_mat(0, 2)),
                        static_cast<double>(imu_mat(1, 0)), static_cast<double>(imu_mat(1, 1)),
                        static_cast<double>(imu_mat(1, 2)),
                        static_cast<double>(imu_mat(2, 0)), static_cast<double>(imu_mat(2, 1)),
                        static_cast<double>(imu_mat(2, 2)));
        imu_tf.getRPY(guess_pos.roll, guess_pos.pitch, guess_pos.yaw, 0);
        std::cout << "imu guess: " << guess_pos.roll << " " << guess_pos.pitch << " " << guess_pos.yaw << std::endl;
       }

    Eigen::AngleAxisf init_rotation_x(guess_pos.roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf init_rotation_y(guess_pos.pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf init_rotation_z(guess_pos.yaw, Eigen::Vector3f::UnitZ());

    Eigen::Translation3f init_translation(guess_pos.x, guess_pos.y, guess_pos.z);
    Eigen::Matrix3f init_rot_mat = (init_rotation_z * init_rotation_y * init_rotation_x).matrix();
    Eigen::Quaternionf init_rotation(init_rot_mat);
    //init_rotation = init_rotation_z * init_rotation_y * init_rotation_x;
    Eigen::Matrix4f init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();
    //std::cout<<"init guess: "<<init_guess<<std::endl;
    //publish guess pos
    if(debug_odom_)
    {
        geometry_msgs::PoseStamped guess_pose_msg, init_pose_msg;

        guess_pose_msg.header.frame_id = "map";
        guess_pose_msg.header.stamp = scan_time;

        init_pose_msg.header.frame_id = "map";
        init_pose_msg.header.stamp = scan_time;
        if(plus_)
        {
            guess_pose_msg.pose.position.x = prev_odom_x + odom_x;
            prev_odom_x = prev_odom_x + odom_x;
            guess_pose_msg.pose.position.y = prev_odom_y + odom_y;
            prev_odom_y = prev_odom_y + odom_y;

        }
        else{
            guess_pose_msg.pose.position.x = prev_odom_x - odom_x;
            prev_odom_x = prev_odom_x - odom_x;
            guess_pose_msg.pose.position.y = prev_odom_y - odom_y;
            prev_odom_y = prev_odom_y - odom_y;
        }
        guess_pose_msg.pose.position.z = 0;
        guess_pose_msg.pose.orientation.x = init_rotation.x();
        guess_pose_msg.pose.orientation.y = init_rotation.y();
        guess_pose_msg.pose.orientation.z = init_rotation.z();
        guess_pose_msg.pose.orientation.w = init_rotation.w();
        guess_pose_pub.publish(guess_pose_msg);

        init_pose_msg.pose.position.x = guess_pos.x;
        init_pose_msg.pose.position.y = guess_pos.y;
        init_pose_msg.pose.position.z = 0;
        init_pose_msg.pose.orientation.x = init_rotation.x();
        init_pose_msg.pose.orientation.y = init_rotation.y();
        init_pose_msg.pose.orientation.z = init_rotation.z();
        init_pose_msg.pose.orientation.w = init_rotation.w();
        init_pose_pub.publish(init_pose_msg);
    }

    //get t
    getTransform(scan_ptr, target_ptr, init_guess);

    // if shift is outside the range, ignore this scan
//---------------------------------------------------------------------//
    //update current pose and publish pose
    updatePos();
    double shift = sqrt(pow(current_pos.x-added_pos.x, 2.0)
                        + pow(current_pos.y-added_pos.y, 2.0));
//    std::cout << "shift: " << shift << std::endl;

    accumulatePoseNode();
    if(shift >= SHIFT_MIN)
    {
        added_pos.x = current_pos.x;
        added_pos.y = current_pos.y;
        added_pos.z = current_pos.z;
        added_pos.roll = current_pos.roll;
        added_pos.pitch = current_pos.pitch;
        added_pos.yaw = current_pos.yaw;

        updateMaps(scan_ptr, scan_selected_ptr);
        setPoseNode();
        pose_node_fitness = 1;
        pose_node_transform.setIdentity();
    }


//    std::cout << "-----------------------------------------------------------------" << std::endl;
//    std::cout << "Sequence number: " << input->header.seq << std::endl;
//    std::cout << "Number of scan points: " << scan_ptr->size() << " points." << std::endl;
//    std::cout << "Number of filtered scan points: " << filtered_scan_ptr->size() << " points." << std::endl;
//    std::cout << "transformed_scan_ptr: " << transformed_scan_ptr->points.size() << " points." << std::endl;
//    std::cout << "filtered global map: " << filtered_whole_map_ptr->points.size() << " points." << std::endl;
//    std::cout << "Number of iteration: " << ndt.getFinalNumIteration() << std::endl;

 //   std::cout << "-----------------------------------------------------------------" << std::endl;

}

void Ndt::accumulatePoseNode()
{
    if(ndt.getTransformationProbability()!=0)
        pose_node_fitness += ndt.getTransformationProbability();
    pose_node_transform = t * pose_node_transform;
}

void Ndt::setPoseNode()
{
    //poseNode posnd;
    posnd.setHeader(header);
    posnd.setPose(current_pos);
    posnd.setTransform(pose_node_transform);
    //set odom
    double w, x, y, z;
    w = current_odom.pose.pose.orientation.w;
    x = current_odom.pose.pose.orientation.x;
    y = current_odom.pose.pose.orientation.y;
    z = current_odom.pose.pose.orientation.z;
    Eigen::Quaterniond odom_quat(w, x, y, z);

    Eigen::Matrix3d odom_mat = odom_quat.matrix();

    tf::Matrix3x3 odom_tf;
    odom_tf.setValue(static_cast<double>(odom_mat(0, 0)), static_cast<double>(odom_mat(0, 1)),
                     static_cast<double>(odom_mat(0, 2)),
                     static_cast<double>(odom_mat(1, 0)), static_cast<double>(odom_mat(1, 1)),
                     static_cast<double>(odom_mat(1, 2)),
                     static_cast<double>(odom_mat(2, 0)), static_cast<double>(odom_mat(2, 1)),
                     static_cast<double>(odom_mat(2, 2)));
    double odom_roll, odom_pitch, odom_yaw;
    odom_tf.getRPY(odom_roll, odom_pitch, odom_yaw, 0);
    posnd.setOdom(current_odom.pose.pose.position.x,
                  current_odom.pose.pose.position.y,
                  current_odom.pose.pose.position.z,
                    odom_roll,
                    odom_pitch,
                    odom_yaw);
    posnd.setOdomMat(current_odom.pose.pose.position.x,
                     current_odom.pose.pose.position.y,
                     current_odom.pose.pose.position.z,
                     current_odom.pose.pose.orientation.w,
                     current_odom.pose.pose.orientation.x,
                     current_odom.pose.pose.orientation.y,
                     current_odom.pose.pose.orientation.z);
    pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));
    posnd.setScan(scan_ptr);
    posnd.setFitness(std::max(pose_node_fitness, 0.01));
    posnd.setId(getId());
    //std::cout<<"get pose node"<<std::endl;
    //std::cout<<"fitness: " << pose_node_fitness <<std::endl;
    //std::cout <<"tansform: \n" << pose_node_transform <<std::endl;
    //return posnd;
}

poseNode Ndt::getPoseNode()
{
    return posnd;
}

unsigned int Ndt::getId()
{
    pose_id++;
    return pose_id;
}