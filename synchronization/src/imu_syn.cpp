#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

#include <ros/ros.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Header.h>
std::ofstream Imu_roll;
std::ofstream Imu_pitch;
std::ofstream Imu_yaw;
std::ofstream Imu_time;
class ImuTf
{
public:
    ImuTf();
    ~ImuTf();

private:
    ros::NodeHandle node, private_node;
    std::string imu_pub_topic_;
    ros::Time velodyne_time, pre_velodyne_time;
    double roll_, pitch_, yaw_;
    std::vector<geometry_msgs::QuaternionStamped> imu_vector;
    ros::Subscriber sub, time_sub;
    ros::Publisher pub;
    void ImuCallback(const sensor_msgs::ImuPtr& msg);
    bool new_record_;
    void findOptimal(ros::Time time);
    bool find, initial;
};

ImuTf::ImuTf():private_node("~")
{
    initial = false;
    if(!private_node.getParam("new_record",new_record_))
       new_record_ = true;
    if(!private_node.getParam("imu_pub_topic",imu_pub_topic_))
        imu_pub_topic_ = "/imu/data/transform";

    sub = private_node.subscribe("/imu/data", 40, &ImuTf::ImuCallback, this);
    pub = private_node.advertise<sensor_msgs::Imu>(imu_pub_topic_, 10);
}

ImuTf::~ImuTf()
{
    Imu_roll.close();
    Imu_pitch.close();
    Imu_yaw.close();
    Imu_time.close();
}

void ImuTf::ImuCallback(const sensor_msgs::ImuPtr& msg){
    static tf::TransformBroadcaster br;
    tf::Quaternion imu(msg->orientation.x, msg->orientation.y,msg->orientation.z,msg->orientation.w) ;
    imu.normalize();
    tf::Matrix3x3 imu_mat(imu);
    imu_mat.getRPY(roll_, pitch_, yaw_, 1);
    //std::cout<<"imu data from filter: "<<roll_<<" "<<pitch_<<" "<<yaw_<<std::endl;
    //tf::Quaternion q;
    //q.setRPY(roll_, pitch_, yaw_);

    tf::Quaternion q_trans;
    Imu_roll<<roll_<<"\n";
    Imu_pitch<<pitch_<<"\n";
    Imu_yaw<<yaw_<<"\n";
    Imu_time<<msg->header.stamp.toSec()<<"\n";

    if(!new_record_)
    {
        q_trans.setRPY(roll_ , pitch_ + 0.04*M_PI, M_PI - yaw_ -0.08*M_PI);
    } else{
        q_trans.setRPY(roll_ + M_PI * 0.1, pitch_, M_PI - yaw_ + M_PI/30);
    }

    q_trans.normalize();

    geometry_msgs::QuaternionStamped imu_tmp;
    imu_tmp.quaternion.x = q_trans.x();
    imu_tmp.quaternion.y = q_trans.y();
    imu_tmp.quaternion.z = q_trans.z();
    imu_tmp.quaternion.w = q_trans.w();
    //imu_tmp.normalize();
    imu_tmp.header.stamp = msg->header.stamp;

    sensor_msgs::Imu input;
    input.header = msg->header;
    //input.header.frame_id = "imu_map";
    input.orientation.x = q_trans.x();
    input.orientation.y = q_trans.y();
    input.orientation.z = q_trans.z();
    input.orientation.w = q_trans.w();

    pub.publish(input);


}


int main(int argc, char** argv){
    ros::init(argc, argv, "imu_synchronize");
    ImuTf tf;
    Imu_roll.open("Imu_roll.txt");
    Imu_pitch.open("Imu_pitch.txt");
    Imu_yaw.open("Imu_yaw.txt");
    Imu_time.open("Imu_time.txt");

    ros::spin();
    return 0;
};