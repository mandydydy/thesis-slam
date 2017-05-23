#include "optimizer.hpp"


GraphOptimizer::GraphOptimizer():private_nh("~")
{
    //get parameters
    if(!private_nh.getParam("add_last_edge", add_last_edge_))
        add_last_edge_ = false;
    if(!private_nh.getParam("plus", plus_))
        plus_ = false;
    if(!private_nh.getParam("odom_edge", odom_edge_))
        odom_edge_ = false;
    if(!private_nh.getParam("transform_edge", transform_edge_))
        transform_edge_ = true;
    if(!private_nh.getParam("add_random_edge", add_random_edge_))
        add_random_edge_ = true;
    if(!private_nh.getParam("odom_information_scale", odom_information_scale_))
        odom_information_scale_ = 1.0;
    if(!private_nh.getParam("transform_information_scale", transform_information_scale_))
        transform_information_scale_ = 1.0;
    if(!private_nh.getParam("transform_z_noise", transform_z_noise_))
        transform_z_noise_ = 0.1;
    if(!private_nh.getParam("odom_z_scale", odom_z_scale_))
        odom_z_scale_ = 0.02;
    if(!private_nh.getParam("odom_angle_scale", odom_angle_scale_))
        odom_angle_scale_ = 0.005;
    if(!private_nh.getParam("odom_linear_noise", odom_linear_noise_))
        odom_linear_noise_ = 0.1;
    if(!private_nh.getParam("odom_angle_noise", odom_angle_noise_))
        odom_angle_noise_ = 0.02;
    graph_nodes.clear();
    //pose_is_close_distance_ = 1.0;
    //if(!private_nh.getParam("pose_is_close_angle", pose_is_close_angle_))
    //pose_is_close_angle_ = 0.5;
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

           Eigen::Matrix4d transform = graph_nodes.back()->pose.matrix();
           g2o::SE3Quat measurement_mean(Eigen::Quaterniond(transform.block<3, 3>(0, 0)), transform.block<3, 1>(0, 3));

           g2o::VertexSE3* v = new g2o::VertexSE3();
           v->setId(pose_node_ptr->id);
           v->setEstimate(measurement_mean);
           v->setFixed(true);
           optimizer_->addVertex(v);

           graph_nodes_map[pose_node_ptr->id] = pose_node_ptr;
       } else
       {

           Eigen::Matrix4d transform=graph_nodes.back()->pose.matrix();
           g2o::SE3Quat measurement_mean(Eigen::Quaterniond(transform.block<3, 3>(0, 0)), transform.block<3, 1>(0, 3));

           g2o::VertexSE3* v = new g2o::VertexSE3();
           v->setId(pose_node_ptr->id);
           v->setEstimate(measurement_mean);

           optimizer_->addVertex(v);
           graph_nodes_map[pose_node_ptr->id] = pose_node_ptr;

           if((graph_nodes.size() > 1) && odom_edge_)
           {
               addOdomEdge();
           }
           if(transform_edge_)
           {
               addTransformEdge();
           }
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
    poseArray.poses.push_back(node->pose.geometry_msgs());

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
void GraphOptimizer::estimateTransCovariance(double fitness,
                                             unsigned int to_id,
                                             unsigned int from_id,
                                             Eigen::Matrix<double, 6,6>& cov)
{
    double time = graph_nodes_map.find(to_id)->second->header.stamp.toSec()
                  - graph_nodes_map.find(from_id)->second->header.stamp.toSec();
    cov(0,0) = 0.1 * fabs((graph_nodes[to_id]->pose.x - graph_nodes[from_id]->pose.x)/time) + 0.05;
    cov(1,1) = 0.1 * fabs((graph_nodes[to_id]->pose.y - graph_nodes[from_id]->pose.y)/time) + 0.05;
    cov(2,2) = 0.1 * fabs((graph_nodes[to_id]->pose.z - graph_nodes[from_id]->pose.z)/time) + transform_z_noise_;

    cov(3,3) = fabs((graph_nodes[to_id]->pose.roll - graph_nodes[from_id]->pose.roll)/time);
    cov(4,4) = fabs((graph_nodes[to_id]->pose.pitch - graph_nodes[from_id]->pose.pitch)/time) ;
    cov(5,5) = fabs((graph_nodes[to_id]->pose.yaw - graph_nodes[from_id]->pose.yaw)/time);
    //make sure angle is beteween pi and -pi
    Eigen::Vector3d angle_wrap;
    Eigen::Vector3d angles(cov(3,3), cov(4,4), cov(5,5));
    for (int i = 0; i < 3; ++i) {
        if(angles[i] > M_PI )
        {
            angle_wrap[i] = - M_PI + angles[i];
        } else{
            angle_wrap[i] = angles[i];
        }
    }
//    std::cout << angle_wrap.transpose() <<std::endl;
    cov(3,3) = 0.5 * fabs(angle_wrap[0]);
    cov(4,4) = 0.5 * fabs(angle_wrap[1]) + 0.02 * transform_z_noise_;
    cov(5,5) = 0.5 * fabs(angle_wrap[2]) + 0.05 * transform_z_noise_;
//    cov = log10(fitness + 1) * cov * cov;
    cov = fitness * cov;
//    cov = fitness * cov * cov;
    //    cov = cov * exp((fabs((double)to_id - (double)from_id) - 1));
    cov = cov * (fabs((double)to_id - (double)from_id));
    //std::cout <<"fitness: " <<fitness<<std::endl;
    //std::cout <<"trans covariance:\n" <<cov<<std::endl;
}

void GraphOptimizer::estimateOdomCovariance(unsigned int to_id,
                                            unsigned int from_id,
                                            Eigen::Matrix<double, 6, 6>& cov)
{
    double time = graph_nodes_map.find(to_id)->second->header.stamp.toSec() - graph_nodes_map.find(from_id)->second->header.stamp.toSec();
    //use velocity and length to decide covariance
    double x = graph_nodes[to_id]->odom.x - graph_nodes[from_id]->odom.x;
    double y = graph_nodes[to_id]->odom.y - graph_nodes[from_id]->odom.y;
    double yaw = graph_nodes[to_id]->odom.yaw - graph_nodes[from_id]->odom.yaw;
//    std::cout <<"odom difference: " <<x <<", "<<y<<", "<<yaw<<std::endl;
//    std::cout <<"time: "<<time<<std::endl;

    //Eigen::Translation3d translation(100 * fabs(x*x/time), 100 * fabs(y*y/time), 0.1 * time) ;
    //todo squared deviation?
    cov(0,0) = fabs(x/time) + odom_linear_noise_;
    cov(1,1) = fabs(y/time) + odom_linear_noise_;
    cov(2,2) = odom_z_scale_ * time + odom_linear_noise_;
    cov(3,3) = odom_angle_scale_ * time + odom_angle_noise_;
    cov(4,4) = odom_angle_scale_ * time + odom_angle_noise_;
    cov(5,5) = fabs(yaw/time) + odom_angle_noise_;
    //if the interval is not 1, the covariance should decay as the interval increase
//    std::cout <<"odom covariance:\n" <<cov<<std::endl;

//    cov = cov * exp((fabs((double)to_id - (double)from_id) - 1));
    cov = cov * (fabs((double)to_id - (double)from_id));
    std::cout <<"odom covariance:\n" <<cov<<std::endl;
}
g2o::SE3Quat GraphOptimizer::calculateOdomDifference(unsigned int to_id,
                                                    unsigned int from_id)
{

    g2o::SE3Quat current(graph_nodes[to_id]->odom_mat);
    g2o::SE3Quat prev(graph_nodes[from_id]->odom_mat);
    g2o::SE3Quat position_tmp = prev.inverse() * current;

    return position_tmp;

}

g2o::SE3Quat GraphOptimizer::calculateTransformDifference(unsigned int current_id,
                                                      unsigned int prev_id)
{

    Eigen::Matrix4d current_matrix = graph_nodes[current_id]->pose.matrix();
    Eigen::Matrix4d prev_matrix = graph_nodes[prev_id]->pose.matrix();

    g2o::SE3Quat current(current_matrix.block<3,3>(0,0), current_matrix.block<3,1>(0,3));
    g2o::SE3Quat prev(prev_matrix.block<3,3>(0,0), prev_matrix.block<3,1>(0,3));
    g2o::SE3Quat transform = prev.inverse() * current;

    return transform;
}

void GraphOptimizer::addOdomEdge()
{
    std::cout << "add odom edge ..." << std::endl;

    //Position odom_diff;
    unsigned int to_id = current_node_id;
    unsigned int from_id = to_id - 1;
    addOdomEdge(from_id, to_id);

    if(add_random_edge_ && current_node_id>2)
    {
        unsigned int to_id = current_node_id;
        //randomly generate a from id among all previous nodes except the consecutive one
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, current_node_id-2);
        unsigned int from_id = dis(gen);
        addOdomEdge(from_id, to_id);
    }

}

bool GraphOptimizer::addOdomEdge(unsigned int from, unsigned int to)
{
    g2o::SE3Quat measurement_mean = calculateOdomDifference(to, from);
    //display odom difference
//    Eigen::Matrix<double, 6,1> odom_vec = measurement_mean.toMinimalVector();
//    std::cout << "OPTIMIZER: odom difference: "<< odom_vec.transpose() <<std::endl;

    Eigen::Matrix<double, 6, 6> odom_cov;
    odom_cov.setIdentity();
    estimateOdomCovariance(to, from, odom_cov);
    Eigen::Matrix<double, 6, 6> measurement_information = odom_cov.inverse();

    g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(from));
    g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(to));

    // create edge between new key frame and previous key frame with the estimated transformation
    g2o::EdgeSE3* edge = new g2o::EdgeSE3();
    edge->vertices()[0] = v1;
    edge->vertices()[1] = v2;
    edge->setMeasurement(measurement_mean);
    edge->setInformation(odom_information_scale_ * measurement_information);

    return optimizer_->addEdge(edge);
}

void GraphOptimizer::addTransformEdge()
{
    //todo get hessian from pcl?
    std::cout << "add transform edge ..." << std::endl;
    unsigned int to_id = current_node_id;
    unsigned int from_id = to_id - 1;
    addTransformEdge(from_id, to_id);

    if(add_random_edge_ && current_node_id>2)
    {
        unsigned int to_id = current_node_id;
        //randomly generate a from id among all previous nodes except the consecutive one
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, current_node_id-2);
        unsigned int from_id = dis(gen);
        addTransformEdge(from_id, to_id);
    }

}

bool GraphOptimizer::addTransformEdge(unsigned int from, unsigned int to)
{
    g2o::SE3Quat measurement_mean = calculateTransformDifference(to, from);

    Eigen::Matrix<double, 6,1> trans_vec = measurement_mean.toMinimalVector();
    std::cout << "OPTIMIZER: transform difference: "<< trans_vec.transpose() <<std::endl;

    Eigen::Matrix<double, 6, 6> trans_cov;
    trans_cov.setIdentity();
    estimateTransCovariance(graph_nodes.back()->fitness, to, from, trans_cov);

    Eigen::Matrix<double, 6, 6> measurement_information = trans_cov.inverse();
//    Eigen::Matrix<double, 6,6> measurement_information;
//    measurement_information << 10 * Eigen::Matrix3d::Identity(),Eigen::Matrix3d::Zero(),
//                    Eigen::Matrix3d::Zero(), 100 * Eigen::Matrix3d::Identity();

    g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(from));
    g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(to));

    g2o::EdgeSE3* edge = new g2o::EdgeSE3();
    edge->vertices()[0] = v1;
    edge->vertices()[1] = v2;
    edge->setMeasurement(measurement_mean);
    edge->setInformation(transform_information_scale_ * measurement_information);

    return optimizer_->addEdge(edge);
}

bool GraphOptimizer::triggerOptimize(bool opt_on)
{
    if(add_last_edge_)
    {
        std::cout<< "adding loop closure edge ..." << std::endl;
        g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(current_node_id));
        g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(0));
        if(odom_edge_)
        {
            std::cout << "add odom edge ..." << std::endl;

            double x = graph_nodes[current_node_id]->odom.x - graph_nodes[0]->odom.x;
            double y = graph_nodes[current_node_id]->odom.y - graph_nodes[0]->odom.y;
            double yaw = graph_nodes[current_node_id]->odom.yaw - graph_nodes[prev_node_id]->odom.yaw;

            Eigen::Matrix<double, 6, 6> odom_cov;
            odom_cov.setIdentity();
            odom_cov(0,0) = fabs(x);
            odom_cov(1,1) = fabs(y);
            odom_cov(2,2) = 0.1;
            odom_cov(3,3) = 0.01;
            odom_cov(4,4) = 0.01;
            odom_cov(5,5) = fmax(fabs(yaw), 0.01);
            std::cout <<"odom covariance:\n" <<odom_cov<<std::endl;
            //matrix difference between last and first odom
//            Eigen::Matrix4d transform = graph_nodes[current_node_id]->odom.matrix().inverse() * graph_nodes[0]->odom.matrix();;

            //position2matrix(graph_nodes.back()->odom, transform);
//            g2o::SE3Quat measurement_mean(Eigen::Quaterniond(transform.block<3, 3>(0, 0)), transform.block<3, 1>(0, 3));
            g2o::SE3Quat difference = calculateOdomDifference(0, current_node_id);
            g2o::SE3Quat measurement_mean(difference.rotation(), Eigen::Vector3d::Zero());
            Eigen::Matrix<double, 6, 6> measurement_information = odom_cov.inverse();

            // create edge between new key frame and previous key frame with the estimated transformation
            g2o::EdgeSE3* edge_odom = new g2o::EdgeSE3();
            edge_odom->vertices()[0] = v1;
            edge_odom->vertices()[1] = v2;
            edge_odom->setMeasurement(measurement_mean);
            edge_odom->setInformation(measurement_information);

            optimizer_->addEdge(edge_odom);
        }

        if(transform_edge_)
        {
            std::cout << "add close loop edge ..." << std::endl;

            Eigen::Matrix<double, 6, 6> trans_cov;
            trans_cov.setIdentity();
            trans_cov = 0.0001 * Eigen::Matrix<double, 6, 6>::Identity();
            std::cout <<"trans covariance:\n" << trans_cov <<std::endl;

            //matrix difference between the last and the first is zero;
            Eigen::Matrix4d transform = Eigen::Matrix4d::Zero();

            g2o::SE3Quat trans_measurement_mean(Eigen::Quaterniond(transform.block<3, 3>(0, 0)), transform.block<3, 1>(0, 3));
            Eigen::Matrix<double, 6, 6> trans_measurement_information = trans_cov.inverse();

            // create edge between new key frame and previous key frame with the estimated transformation
            g2o::EdgeSE3* edge_trans = new g2o::EdgeSE3();
            edge_trans->vertices()[0] = v1;
            edge_trans->vertices()[1] = v2;
            edge_trans->setMeasurement(trans_measurement_mean);
            edge_trans->setInformation(trans_measurement_information);

            optimizer_->addEdge(edge_trans);
            
        }

    }
    std::cout << "optimizing ..."<< std::endl;
    optimizer_->initializeOptimization();
    optimizer_->save("before3d.g2o");
//    saveMap("original_map.pcd");
    if(opt_on)
    {
        optimizer_->setVerbose(true);
        optimizer_->optimize(10);
        optimizer_->save("after3d.g2o");
        publishCorrected();
//        saveMap("optimized_map.pcd");
    }
    return true;

}

void GraphOptimizer::publishCorrected()
{
    geometry_msgs::PoseArray corrected_array;
    for (unsigned int id = 0; id < graph_nodes.size(); id ++)
    {
        g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(graph_nodes[id]->id));
        Eigen::Matrix4d v_pose_mat = vertex->estimate().matrix();
        Eigen::Quaterniond v_pose_quat(v_pose_mat.block<3,3>(0,0));
        geometry_msgs::Pose pose_msg;
        pose_msg.position.x = v_pose_mat(0,3);
        pose_msg.position.y = v_pose_mat(1,3);
        pose_msg.position.z = v_pose_mat(2,3);
        pose_msg.orientation.w = v_pose_quat.w();
        pose_msg.orientation.x = v_pose_quat.x();
        pose_msg.orientation.y = v_pose_quat.y();
        pose_msg.orientation.z = v_pose_quat.z();
        corrected_array.poses.push_back(pose_msg);
    }

    corrected_array.header = graph_nodes.back()->header;
    corrected_array.header.frame_id = "map";
    corrected_pose_pub.publish(corrected_array);

}

void GraphOptimizer::saveMap(std::string str)
{
    pcl::PointCloud<pcl::PointXYZI> map;

    g2o::VertexSE3* initial_vertex = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(0));
    for(unsigned int it = 0; it < graph_nodes.size(); ++it)
    {
        g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(it));
        unsigned int id = vertex->id();
        Eigen::Matrix4d diff_transform = (initial_vertex->estimate().inverse() * vertex->estimate()).matrix();

        //std::cout << diff_transform << std::endl;
        pcl::PointCloud<pcl::PointXYZI>::Ptr trans_ptr(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::transformPointCloud(*(graph_nodes_map.find(id)->second->scan), *trans_ptr, diff_transform);      //transforn orignial scan
        map += *trans_ptr;
    }

    //downsample the point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(map));
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(0.1f, 0.1f, 0.1f);
    voxel_grid_filter.setInputCloud(map_ptr);
    voxel_grid_filter.filter(*filtered_ptr);

    pcl::io::savePCDFileASCII(str, *filtered_ptr);
}