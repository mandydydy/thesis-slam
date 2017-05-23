//
// Created by mengdi on 2017-04-17.
//

#ifndef OUTDOOR_SLAM_OPTIMIZER_HPP
#define OUTDOOR_SLAM_OPTIMIZER_HPP

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <cmath>
#include <random>

#include <ros/ros.h>

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/Geometry>

#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

#include "pose_node.hpp"

typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> SlamBlockSolver;
typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
typedef g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType> SlamLinearCholmodSolver;

class GraphOptimizer
{
public:
    GraphOptimizer();
    ~GraphOptimizer();
    std::vector<poseNode::Ptr> graph_nodes;
    void addNode();

    bool triggerOptimize(bool opt_on);
    double start_waiting_time;

private:
    ros::NodeHandle nh, private_nh;
    ros::Publisher array_pub, corrected_map_pub, corrected_pose_pub;

    std::map<unsigned int, poseNode::Ptr> graph_nodes_map;      //for searching scan after optimization

    unsigned int prev_node_id, current_node_id;
    //double odom_start_time;     //to calculate time difference

    Position start_odom;

    g2o::SparseOptimizer* optimizer_;

    Eigen::Matrix4d increment_transform;

    double pose_is_close_distance_, pose_is_close_angle_;

    bool first_node;
    bool add_last_edge_;  //add loop closure manually
    bool plus_;
    bool odom_edge_, transform_edge_, add_random_edge_;
    double transform_information_scale_, odom_information_scale_,
            transform_z_noise_,
            odom_z_scale_, odom_angle_scale_,
            odom_linear_noise_, odom_angle_noise_;

    void addOdomEdge();
    bool addOdomEdge(unsigned int from, unsigned int to);
    void addTransformEdge();
    bool addTransformEdge(unsigned int from, unsigned int to);

    bool poseIsClose(const Eigen::Matrix4d& transform);
    //void position2matrix(Position position, Eigen::Matrix4d& mat);
    void matrix2position(Eigen::Matrix4d mat, Position& position);
    void estimateTransCovariance(double fitness, unsigned int to_id, unsigned int from_id, Eigen::Matrix<double, 6,6>& cov);
    void estimateOdomCovariance(unsigned int to_id, unsigned int from_id, Eigen::Matrix<double, 6, 6>& cov);
    g2o::SE3Quat calculateOdomDifference(unsigned int to_id, unsigned int from_id);
    g2o::SE3Quat calculateTransformDifference(unsigned int current_id, unsigned int prev_id);

    geometry_msgs::PoseArray poseArray;
    void publishNodes(poseNode::Ptr node);
    void publishCorrected();
    void saveMap(std::string str);


};


#endif //OUTDOOR_SLAM_OPTIMIZER_HPP
