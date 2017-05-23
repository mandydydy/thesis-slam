//
// Created by mengdi on 2017-04-24.
//

#ifndef OUTDOOR_SLAM_OPTIMIZER2D_HPP
#define OUTDOOR_SLAM_OPTIMIZER2D_HPP
#include <iostream>
#include <fstream>
#include <vector>
#include <unordered_map>
#include <cmath>

#include <ros/ros.h>

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/Geometry>

#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/slam2d/types_slam2d.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

#include "pose_node.hpp"

typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>> SlamBlockSolver;
typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
typedef g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType> SlamLinearCholmodSolver;

typedef std::set<g2o::HyperGraph::Edge*> EdgeSet;

class GraphOptimizer
{
public:
    GraphOptimizer();
    ~GraphOptimizer();
    std::vector<poseNode::Ptr> graph_nodes;
    void addNode();

    bool triggerOptimize();
    double start_waiting_time;

private:
    ros::NodeHandle nh, private_nh;
    ros::Publisher array_pub, corrected_map_pub, corrected_pose_pub;
    unsigned int prev_node_id, current_node_id;
    double odom_start_time;     //to calculate time difference

    Position start_odom;

    g2o::SparseOptimizer* optimizer_;

    Eigen::Matrix4d increment_transform;

    double pose_is_close_distance_, pose_is_close_angle_;

    bool first_node;
    bool add_last_edge_;  //add loop closure manually
    bool plus_;

    bool addOdomEdge(double time);
    bool addTransformEdge();

    bool poseIsClose(const Eigen::Matrix4d& transform);
    //void position2matrix(Position position, Eigen::Matrix4d& mat);
    void matrix2position(Eigen::Matrix4d mat, Position& position);
    void estimateOdomCovariance(double time, Eigen::Matrix3d& cov);
    void estimateTransCovariance(double fitness, Position trans, Eigen::Matrix3d& cov);
    //void accumulateTransformDifference(Eigen::Matrix4d& mat);
    void calculateOdomDifference(unsigned int current_id, unsigned int prev_id, Position& position);

    geometry_msgs::PoseArray poseArray;
    void publishNodes(poseNode::Ptr node);
    void publishCorrected();


};
#endif //OUTDOOR_SLAM_OPTIMIZER2D_HPP
