//
// Created by mengdi on 2017-04-13.
//
#include "ndt_key_mapping.hpp"
#include "optimizer.hpp"
#include "pose_node.hpp"
//#include "graph_node.hpp"
#include <tf/transform_broadcaster.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ndt_mapping");
    Ndt mapper;
    //GraphNode::Ptr graph;
    GraphOptimizer opt;
    ros::Rate loop_rate(9);

    tf::TransformBroadcaster br;
    /*currentPos_x.open("currentPos_x.txt");
    currentPos_y.open("currentPos_y.txt");
    currentPos_z.open("currentPos_z.txt");
    currentPos_roll.open("currentPos_roll.txt");
    currentPos_pitch.open("currentPos_pitch.txt");
    currentPos_yaw.open("currentPos_yaw.txt");
    fitness.open("fitness.txt");
    */
    double waiting_time;
    unsigned int prev_id = -1;
    while (ros::ok())
    {
        poseNode pn = mapper.getPoseNode();
        //std::cout << pn.fitness << std::endl;
        if(pn.fitness!=-1)  //if this is not an empty node
        {
            if(pn.id != prev_id) { //if this is a new node
                prev_id = pn.id;
                poseNode::Ptr pn_ptr(new poseNode(pn));
                opt.graph_nodes.push_back(pn_ptr);
                opt.addNode();
            }
        }

        waiting_time = ros::Time::now().toSec() - opt.start_waiting_time;
        if(waiting_time > 10)
        {
            std::cout<< "No nodes coming in within : " << waiting_time << "seconds"<< std::endl;
            if(!opt.graph_nodes.empty())
            {
                opt.triggerOptimize(true);
                break;
            }
        }
        br.sendTransform(tf::StampedTransform(mapper.g_transform.inverse(), ros::Time::now(), "base_link", "map"));
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}