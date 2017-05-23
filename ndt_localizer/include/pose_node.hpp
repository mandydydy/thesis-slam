//
// Created by mengdi on 2017-04-17.
//

#ifndef OUTDOOR_SLAM_POSENODE_HPP
#define OUTDOOR_SLAM_POSENODE_HPP
#include <iostream>
#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <geometry_msgs/Pose.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <std_msgs/Header.h>

class Position {
public:
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;

    geometry_msgs::Pose geometry_msgs()
    {
        geometry_msgs::Pose pose_msg;
        pose_msg.position.x = x;
        pose_msg.position.y = y;
        pose_msg.position.z = z;

        Eigen::AngleAxisd rot_roll(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd rot_pitch(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd rot_yaw(yaw, Eigen::Vector3d::UnitZ());

        Eigen::Quaterniond q((rot_yaw*rot_pitch*rot_roll).matrix());
        pose_msg.orientation.w = q.w();
        pose_msg.orientation.x = q.x();
        pose_msg.orientation.y = q.y();
        pose_msg.orientation.z = q.z();
        return pose_msg;
    }

    Eigen::Matrix4d matrix()
    {
        Eigen::Translation3d translation(x, y, z);

        Eigen::Quaterniond rot = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                                 * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                                 * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

        return (translation*rot).matrix();
    }
    //friend Position operator-(const Position &p1, const Position &p2);

    //friend std::ostream& operator<< (std::ostream &os, const Position& pos);
};

/*Position operator-(const Position &p1, const Position &p2)
{
    Position result;
    result.x = p1.x - p2.x;
    result.y = p1.y - p2.y;
    result.z = p1.z - p2.z;
    result.roll = p1.roll - p2.roll;
    result.pitch = p1.pitch - p2.pitch;
    result.yaw = p1.yaw - p2.yaw;
    return result;
}*/
/*std::ostream& operator<< (std::ostream &os, const Position& pos)
{
    os << pos.x <<","
        << pos.y <<","
        << pos.z <<","
        << pos.roll <<","
        << pos.pitch <<","
        << pos.yaw ;
    return os;
}
 */
class poseNode
{
public:
    //todo remove transform
    unsigned int id;
    typedef boost::shared_ptr<poseNode> Ptr;
    std_msgs::Header header;
    Position pose;
    Position odom;
    g2o::SE3Quat odom_mat;
    Eigen::Matrix4d transform;
    double fitness;
    pcl::PointCloud<pcl::PointXYZI>::Ptr scan;

    poseNode()
    {
        fitness = -1;
    }
    ~poseNode()
    {

    }
    //setters
    void setHeader(const std_msgs::Header h)
    {this->header = h;}
    void setPose(const Position p)
    {this->pose = p;}
    void setOdom(const double x,
                 const double y,
                 const double z,
                 const double roll,
                 const double pitch,
                 const double yaw)
    {
        odom.x = x;
        odom.y = y;
        odom.z = z;
        odom.roll = roll;
        odom.pitch = pitch;
        odom.yaw = yaw;

    }
    void setOdomMat(const double x,
                    const double y,
                    const double z,
                    const double w,
                    const double r,
                    const double p,
                    const double ya)
    {
        Eigen::Quaterniond quat(w,r,p,ya);
        odom_mat.setRotation(quat);

        Eigen::Vector3d trans(x,y,z);
        odom_mat.setTranslation(trans);
    }
    void setTransform(const Eigen::Matrix4f trans)
    {this->transform = trans.cast<double>();}
    void setScan(pcl::PointCloud<pcl::PointXYZI>::Ptr sc)
    {this->scan = sc;}
    void setFitness(const double fit)
    {this->fitness = fit;}
    void setId(const unsigned int i)
    {this->id = i;}

    //getters
    std_msgs::Header getHeader()
    { return header;}
    Position getPose()
    { return pose;}
    Position getOdom()
    { return odom;}
    Eigen::Matrix4d getTransform()
    {return transform;}
    pcl::PointCloud<pcl::PointXYZI>::Ptr getScan()
    { return scan;}
    double getFitness()
    { return fitness;}

};
#endif //OUTDOOR_SLAM_POSENODE_HPP
