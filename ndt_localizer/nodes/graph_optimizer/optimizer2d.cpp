#include "optimizer2d.hpp"


GraphOptimizer::GraphOptimizer():private_nh("~")
{
    //get parameters
    if(!private_nh.getParam("add_last_edge", add_last_edge_))
        add_last_edge_ = false;
    if(!private_nh.getParam("plus", plus_))
        plus_ = false;
    graph_nodes.clear();
    pose_is_close_distance_ = 1.0;
    //if(!private_nh.getParam("pose_is_close_angle", pose_is_close_angle_))
    pose_is_close_angle_ = 0.5;
    //set g2o optimizer
    optimizer_ = new g2o::SparseOptimizer();
    optimizer_->setVerbose(false);
    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering(false);
    SlamBlockSolver* solver = new SlamBlockSolver(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solverLevenberg = new g2o::OptimizationAlgorithmLevenberg(solver);

    optimizer_->setAlgorithm(solverLevenberg);

    //initialize variables
    //last_transform.setIdentity();
    increment_transform.setIdentity();
    first_node = true;
    prev_node_id = -1;
    current_node_id = 0;
    //over = false;
    start_waiting_time = ros::Time::now().toSec();

    array_pub = private_nh.advertise<geometry_msgs::PoseArray>("/graph_nodes", 10);
    corrected_map_pub = private_nh.advertise<sensor_msgs::PointCloud2>("/corrected_maps", 10);
    corrected_pose_pub = private_nh.advertise<geometry_msgs::PoseArray>("/corrected_graph_nodes", 10);
}

GraphOptimizer::~GraphOptimizer()
{
    delete optimizer_;

}

void GraphOptimizer::addNode()
{

   if(graph_nodes.size() - 1 != prev_node_id)
   {
       poseNode::Ptr pose_node_ptr = graph_nodes.back();
       prev_node_id = current_node_id;
       current_node_id = pose_node_ptr->id;
       std::cout << "OPTIMIZER: pose node id: " << current_node_id << std::endl;

       if(first_node)
       {
           first_node = false;
           odom_start_time = pose_node_ptr->header.stamp.toSec();

           g2o::SE2 measurement_mean(graph_nodes.back()->pose.x,
                                     graph_nodes.back()->pose.y,
                                     graph_nodes.back()->pose.yaw);

           g2o::VertexSE2* v = new g2o::VertexSE2();
           v->setId(pose_node_ptr->id);
           v->setEstimate(measurement_mean);
           v->setFixed(true);
           optimizer_->addVertex(v);
       } else
       {

           g2o::SE2 measurement_mean(graph_nodes.back()->pose.x,
                                     graph_nodes.back()->pose.y,
                                     graph_nodes.back()->pose.yaw);

           g2o::VertexSE2* v = new g2o::VertexSE2();
           v->setId(pose_node_ptr->id);
           v->setEstimate(measurement_mean);

           optimizer_->addVertex(v);

           double time = pose_node_ptr->header.stamp.toSec();
           //addOdomEdge(time);
           odom_start_time = time;
           addTransformEdge();
       }
       //update waiting time
       publishNodes(pose_node_ptr);
       prev_node_id = pose_node_ptr->id;
       start_waiting_time = ros::Time::now().toSec();
   }

}
void GraphOptimizer::publishNodes(poseNode::Ptr node)
{
    poseArray.header = node->header;
    poseArray.header.frame_id = "map";
    Position pose;
    pose.x = node->pose.x;
    pose.y = node->pose.y;
    pose.z = 0;
    pose.roll = 0;
    pose.pitch = 0;
    pose.yaw = node->pose.yaw;
    poseArray.poses.push_back(pose.geometry_msgs());

    array_pub.publish(poseArray);
}

void GraphOptimizer::matrix2position(Eigen::Matrix4d mat, Position& position)
{
    position.x = mat(0,3);
    position.y = mat(1,3);
    position.z = mat(2,3);

    Eigen::Matrix3d rot_mat;
    rot_mat = mat.block<3,3>(0,0);
    Eigen::Vector3d ea = rot_mat.eulerAngles(1,2,3);
    position.roll = ea[0];
    position.pitch = ea[1];
    position.yaw = ea[2];
}

/*bool GraphOptimizer::poseIsClose(const Eigen::Matrix4d& transform)
{
    //if the transformation between two nodes is within a threshold
    double dist = transform.block<3, 1>(0, 3).norm();
    Eigen::Vector3d angles = transform.block<3, 3>(0, 0).eulerAngles(0,1,2);
    return (dist < pose_is_close_distance_
            && angles[0] < pose_is_close_angle_
            && angles[1] < pose_is_close_angle_
            && angles[2] < pose_is_close_angle_);
}
*/
void GraphOptimizer::estimateTransCovariance(double fitness, Position trans, Eigen::Matrix3d& cov)
{
    std::cout << trans.x <<","
            << trans.y <<","
            << trans.z <<","
            << trans.roll <<","
            << trans.pitch <<","
            << trans.yaw << std::endl;

    cov(0,0) = trans.x;
    cov(1,1) = trans.y;

    if(trans.yaw > M_PI )
    {
        trans.yaw = - M_PI + trans.yaw;
    } else{
        trans.yaw = trans.yaw;
    }

    cov(2,2) = trans.yaw;
    cov = fitness * cov;
    std::cout <<"fitness: " <<fitness<<std::endl;
    std::cout <<"trans covariance:\n" <<cov<<std::endl;
}

void GraphOptimizer::estimateOdomCovariance(double time, Eigen::Matrix3d& cov)
{
    //use velocity and length to decide covariance
    double x = graph_nodes[current_node_id]->odom.x - graph_nodes[prev_node_id]->odom.x;
    double y = graph_nodes[current_node_id]->odom.y - graph_nodes[prev_node_id]->odom.y;
    double yaw = graph_nodes[current_node_id]->odom.yaw - graph_nodes[prev_node_id]->odom.yaw;
    std::cout <<"odom difference: " <<x <<", "<<y<<", "<<yaw<<std::endl;
    std::cout <<"time: "<<time<<std::endl;

    //Eigen::Translation3d translation(100 * fabs(x*x/time), 100 * fabs(y*y/time), 0.1 * time) ;
    cov(0,0) = 100 * fabs(x*x/time);
    cov(1,1) = 100 * fabs(y*y/time);
    cov(2,2) = 100 * fabs(yaw*yaw/time);

    std::cout <<"odom covariance:\n" <<cov<<std::endl;
}


void GraphOptimizer::calculateOdomDifference(unsigned int current_id,
                                             unsigned int prev_id,
                                             Position& position)
{
    if(plus_)
    {
        position.x = graph_nodes[current_id]->odom.x - graph_nodes[prev_id]->odom.x;
        position.y = graph_nodes[current_id]->odom.y - graph_nodes[prev_id]->odom.y;
        position.z = graph_nodes[current_id]->odom.z - graph_nodes[prev_id]->odom.z;
    } else
    {
        position.x = - graph_nodes[current_id]->odom.x + graph_nodes[prev_id]->odom.x;
        position.y = - graph_nodes[current_id]->odom.y + graph_nodes[prev_id]->odom.y;
        position.z = - graph_nodes[current_id]->odom.z + graph_nodes[prev_id]->odom.z;
    }

    position.roll = graph_nodes[current_id]->odom.roll - graph_nodes[prev_id]->odom.roll;
    position.pitch = graph_nodes[current_id]->odom.pitch - graph_nodes[prev_id]->odom.pitch;
    position.yaw = graph_nodes[current_id]->odom.yaw - graph_nodes[prev_id]->odom.yaw;
}

bool GraphOptimizer::addOdomEdge(double time)
{   //todo add more edges?
    std::cout << "add odom edge ..." << std::endl;
    double delta_time = time - odom_start_time;
    Eigen::Matrix3d odom_cov;
    odom_cov.setIdentity();
    estimateOdomCovariance(delta_time, odom_cov);

    Position odom_diff;
    calculateOdomDifference(current_node_id, prev_node_id, odom_diff);

    std::cout << "odom measurement: \n"
                <<odom_diff.x <<","
                <<odom_diff.y <<","
                <<odom_diff.z <<","
                <<odom_diff.roll <<","
                <<odom_diff.pitch <<","
                <<odom_diff.yaw << std::endl;

    g2o::SE2 measurement_mean(odom_diff.x, odom_diff.y, odom_diff.yaw);
    Eigen::Matrix3d measurement_information = odom_cov.inverse();

    g2o::VertexSE2* v1 = dynamic_cast<g2o::VertexSE2*>(optimizer_->vertex(prev_node_id));
    g2o::VertexSE2* v2 = dynamic_cast<g2o::VertexSE2*>(optimizer_->vertex(current_node_id));

    // create edge between new key frame and previous key frame with the estimated transformation
    g2o::EdgeSE2* edge = new g2o::EdgeSE2();
    edge->vertices()[0] = v1;
    edge->vertices()[1] = v2;
    edge->setMeasurement(measurement_mean);
    edge->setInformation(measurement_information);

    return optimizer_->addEdge(edge);
}

bool GraphOptimizer::addTransformEdge()
{
    //todo get hessian from pcl?
    std::cout << "add transform edge ..." << std::endl;

    Eigen::Matrix4d transform;
    transform << graph_nodes.back()->transform;

    Position trans_pos;
    matrix2position(transform, trans_pos);

    Eigen::Matrix3d trans_cov;
    trans_cov.setIdentity();
    estimateTransCovariance(graph_nodes.back()->fitness, trans_pos, trans_cov);

    g2o::SE2 measurement_mean(trans_pos.x, trans_pos.y, trans_pos.yaw);
    Eigen::Matrix3d measurement_information = trans_cov.inverse();

    g2o::VertexSE2* v1 = dynamic_cast<g2o::VertexSE2*>(optimizer_->vertex(prev_node_id));
    g2o::VertexSE2* v2 = dynamic_cast<g2o::VertexSE2*>(optimizer_->vertex(current_node_id));

    // create edge between new key frame and previous key frame with the estimated transformation
    g2o::EdgeSE2* edge = new g2o::EdgeSE2();
    edge->vertices()[0] = v1;
    edge->vertices()[1] = v2;
    edge->setMeasurement(measurement_mean);
    edge->setInformation(measurement_information);

    return optimizer_->addEdge(edge);
}

bool GraphOptimizer::triggerOptimize()
{
    if(add_last_edge_)
    {
        std::cout<< "adding loop closure edge ..." << std::endl;
        std::cout << "add odom edge ..." << std::endl;
        //estimateOdomCovariance(delta_time, odom_cov);
        double x = graph_nodes[current_node_id]->odom.x - graph_nodes[0]->odom.x;
        double y = graph_nodes[current_node_id]->odom.y - graph_nodes[0]->odom.y;
        double yaw = graph_nodes[current_node_id]->odom.yaw - graph_nodes[prev_node_id]->odom.yaw;

        Eigen::Matrix3d odom_cov;
        odom_cov.setIdentity();
        odom_cov(0,0) = fabs(x * 50);
        odom_cov(1,1) = fabs(y * 50);
        odom_cov(2,2) = fabs(yaw*100);
        std::cout <<"odom covariance:\n" <<odom_cov<<std::endl;
        //matrix difference between last and first odom
        Position odom_diff;
        calculateOdomDifference(0, current_node_id, odom_diff);

        g2o::SE2 measurement_mean(odom_diff.x, odom_diff.y, odom_diff.yaw);
        Eigen::Matrix3d measurement_information = odom_cov.inverse();

        g2o::VertexSE2* v1 = dynamic_cast<g2o::VertexSE2*>(optimizer_->vertex(current_node_id));
        g2o::VertexSE2* v2 = dynamic_cast<g2o::VertexSE2*>(optimizer_->vertex(0));

        // create edge between new key frame and previous key frame with the estimated transformation
        g2o::EdgeSE2* edge_odom = new g2o::EdgeSE2();
        edge_odom->vertices()[0] = v1;
        edge_odom->vertices()[1] = v2;
        edge_odom->setMeasurement(measurement_mean);
        edge_odom->setInformation(measurement_information);

        std::cout << "add close loop edge ..." << std::endl;

        Eigen::Matrix3d trans_cov;
        trans_cov.setIdentity();
        trans_cov = 0.01 * Eigen::Matrix3d::Identity();
        std::cout <<"trans covariance:\n" << trans_cov <<std::endl;

        //matrix difference between the last and the first is zero;
        g2o::SE2 trans_measurement_mean(0,0,0);
        Eigen::Matrix3d trans_measurement_information = trans_cov.inverse();

        // create edge between new key frame and previous key frame with the estimated transformation
        g2o::EdgeSE2* edge_trans = new g2o::EdgeSE2();
        edge_trans->vertices()[0] = v1;
        edge_trans->vertices()[1] = v2;
        edge_trans->setMeasurement(trans_measurement_mean);
        edge_trans->setInformation(trans_measurement_information);

        optimizer_->addEdge(edge_odom);
        optimizer_->addEdge(edge_trans);
    }

    std::cout << "optimizing ..."<< std::endl;
    optimizer_->initializeOptimization();
    optimizer_->setVerbose(true);
    optimizer_->optimize(10);
    publishCorrected();
    return true;

}

void GraphOptimizer::publishCorrected()
{
    std::ofstream graph_before;
    std::ofstream graph_after;
    graph_before.open("graph2d_before.txt");
    graph_after.open("graph2d_after.txt");
    //todo save a pcd file, publish corrected poses and maps
    geometry_msgs::PoseArray corrected_array;
    for (unsigned int id = 0; id < graph_nodes.size(); id ++)
    {
        graph_before << graph_nodes[id]->pose.x << ","
                     << graph_nodes[id]->pose.y << ","
                     << graph_nodes[id]->pose.yaw << ","
                     << "\n";

        g2o::VertexSE2* vertex = dynamic_cast<g2o::VertexSE2*>(optimizer_->vertex(graph_nodes[id]->id));
        Eigen::Vector3d v_pose_mat = vertex->estimate().toVector();

        Position v_pose;
        v_pose.x = v_pose_mat[0];
        v_pose.y = v_pose_mat[1];
        v_pose.yaw = v_pose_mat[2];
        v_pose.z = 0;
        v_pose.roll = 0;
        v_pose.pitch = 0;
        corrected_array.poses.push_back(v_pose.geometry_msgs());
        graph_after << v_pose.x << ","
                    << v_pose.y << ","
                    << v_pose.yaw << ","
                    << "\n";
    }
    graph_before.close();
    graph_after.close();
    corrected_array.header = graph_nodes.back()->header;
    corrected_array.header.frame_id = "map";
    corrected_pose_pub.publish(corrected_array);
    /*{
        g2o::VertexSE3* v =  dynamic_cast<g2o::VertexSE3*>(optimizer_->vertices(*it));
        if(v == NULL)
            continue;

        double data[7];
        v->getEstimateData( data );
        for (int i=0; i<7; i++)
            std::cout<<data[i]<<" ";
        std::cout << std::endl;
        //push back poses to pose array
        //get transform
        //transform scan
        //publish map
    }*/

}