#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
using namespace std;
ros::Publisher ekf_pub, vicon_pub;
nav_msgs::Path ekf_path, vicon_path;

void ekf_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    

    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.header=msg->header;
    this_pose_stamped.header.frame_id="world";
    this_pose_stamped.pose = msg->pose.pose;

    ekf_path.header=msg->header;     
    ekf_path.header.frame_id="world";
    ekf_path.poses.push_back(this_pose_stamped);
    ekf_pub.publish(ekf_path);
}



void vicon_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.header=msg->header;
    this_pose_stamped.header.frame_id="world";
    this_pose_stamped.pose = msg->pose.pose;
    
    vicon_path.header=msg->header;
    vicon_path.header.frame_id="world";
    vicon_path.poses.push_back(this_pose_stamped);
    vicon_pub.publish(vicon_path);
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "showpath");
    ros::NodeHandle n("~");

    ros::Subscriber s2 = n.subscribe("/ekf/ekf_odom", 1000, ekf_callback,ros::TransportHints().tcpNoDelay());
    ros::Subscriber s3 = n.subscribe("/uwb_vicon_odom", 1000, vicon_callback,ros::TransportHints().tcpNoDelay());

    ekf_pub = n.advertise<nav_msgs::Path>("ekf_path", 100);
    vicon_pub = n.advertise<nav_msgs::Path>("vicon_path", 100);

    ros::spin();
}