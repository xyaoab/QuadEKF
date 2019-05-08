#include <ros/ros.h>
#include <ros/transport_hints.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Image.h>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include "visualization_msgs/Marker.h"
#include <iomanip>
#include "visualization_msgs/Marker.h"
#include "LowPassFilter.hpp"
#include <cmath>
using namespace std;
double height=0; 
double last_height=0;
double height_time=0;
double last_height_time=0;
double vicon_time, last_vicon_time, image_time, last_image_time;
Eigen::Vector3d velocity, position, last_position, velocity_gt;
Eigen::Quaterniond q;
ros::Publisher pub_vel, pub_vel_gt;
cv::Mat tmp, image,prev_image;
double fx, fy, cx, cy;
cv::Mat cameraMatrix, distCoeffs;
cv::Size imageSize;
cv::Mat map1, map2;
vector<cv::Point2f> points,prev_points(12*8),good_pts, good_prev_pts;
vector<uchar> status;
vector<float> err;
cv::TermCriteria termcrit(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,30,0.03);
cv::Size winSize(31,31);
cv::Mat mask;
//double RMS_vx, RMS_vy;
//int frame_count;
//double RMS_vx_sum=0.;
//double RMS_vy_sum=0.;
ros::Publisher odom_pub;
ros::Publisher pub_text;
bool print_flag=true;
bool vicon_rec=true;
bool height_flag=true;
bool undistort_flag=true;
//LowPassFilter lpfz(30);
//LowPassFilter lpfx(30);
//LowPassFilter lpfy(30);

void visualizeText(Eigen::Vector3d position, std::string yaw, ros::Publisher pub_text) {

    visualization_msgs::Marker m;
    m.header.frame_id = "world";
    m.id = 1;
    m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    m.action = visualization_msgs::Marker::ADD;
    m.scale.z = 0.5;
    m.pose.position.x = position.x();
    m.pose.position.y = position.y();
    m.pose.position.z = position.z();
    m.pose.orientation.w = 1;
    m.pose.orientation.x = 0;
    m.pose.orientation.y = 0;
    m.pose.orientation.z = 0;
    m.color.a = 1.0;
    m.color.r = 255;
    m.color.g = 255;
    m.color.b = 0;
    m.text = yaw;

    pub_text.publish(m);
}

void visualizeVelocity(Eigen::Vector3d position, Eigen::Vector3d velocity,
                       int id, Eigen::Vector3d color, ros::Publisher pub_vel) {
    double scale = 10;
    visualization_msgs::Marker m;
    m.header.frame_id = "world";
    m.id = id;
    m.type = visualization_msgs::Marker::ARROW;
    m.action = visualization_msgs::Marker::MODIFY;
    m.scale.x = 0.2;
    m.scale.y = 0.5;
    m.scale.z = 0;
    m.pose.position.x = 0;
    m.pose.position.y = 0;
    m.pose.position.z = 0;
    m.pose.orientation.w = 1;
    m.pose.orientation.x = 0;
    m.pose.orientation.y = 0;
    m.pose.orientation.z = 0;
    m.color.a = 1.0;
    m.color.r = color.x();
    m.color.g = color.y();
    m.color.b = color.z();
    m.points.clear();
    geometry_msgs::Point point;
    point.x = position.x();
    point.y = position.y();
    point.z = position.z();
    m.points.push_back(point);
    point.x = position.x() + velocity.x() * scale;
    point.y = position.y() + velocity.y() * scale;
    point.z = position.z() + velocity.z() * scale;
    m.points.push_back(point);
    pub_vel.publish(m);
}


void heightCallback(const sensor_msgs::Range::ConstPtr &height_msg) {
    height = height_msg->range;
    height_time = height_msg->header.stamp.toSec();
    if(last_height_time!=0){
        velocity.z() = (height-last_height)/(height_time-last_height_time);
        //velocity.z() = lpfz.update(vel,(height_time-last_height_time));
     }

    last_height=height;
    last_height_time=height_time;
    if(height_flag){
        cout<<"TFMINI START"<<endl;
        height_flag=false;
    }

}

void viconCallback(const nav_msgs::Odometry::ConstPtr &vicon_msg) {
    position.x() = vicon_msg->pose.pose.position.x;
    position.y() = vicon_msg->pose.pose.position.y;
    position.z() = vicon_msg->pose.pose.position.z;
    q = Eigen::Quaterniond(vicon_msg->pose.pose.orientation.w,
                           vicon_msg->pose.pose.orientation.x,
                           vicon_msg->pose.pose.orientation.y,
                           vicon_msg->pose.pose.orientation.z);
    vicon_time = vicon_msg->header.stamp.toSec();
    velocity_gt.x() = vicon_msg->twist.twist.linear.x;
    velocity_gt.y() = vicon_msg->twist.twist.linear.y;
    velocity_gt.z() = vicon_msg->twist.twist.linear.z;
    //Eigen::Matrix3d R_ref(q);
    //double phi = asin(R_ref(2,1));
    //double theta = atan2(-R_ref(2,0)/cos(phi), R_ref(2,2)/cos(phi));        
    //double psi = atan2(-R_ref(0,1)/cos(phi), R_ref(1,1)/cos(phi));
    
    //visualizeText(Eigen::Vector3d(0,0,-2), "vicon"+ std::to_string(phi) + " , " + std::to_string(theta) +" , "+std::to_string(psi), pub_text);
    if(vicon_rec){
        cout<<"VICON START"<<endl;
        vicon_rec=false;
    }
}
void imageCallback(const sensor_msgs::Image::ConstPtr &image_msg) {
    
    double last_run_time = ros::Time::now().toSec();
    image_time = image_msg->header.stamp.toSec();
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(image_msg, image_msg->encoding);
    cv_ptr->image.copyTo(tmp);
    //cv::undistort(tmp, image, cameraMatrix, distCoeffs);
    if(undistort_flag){
        cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), cameraMatrix, imageSize, CV_32F, map1, map2);
        undistort_flag=false;
    }
    cv::remap(tmp, image, map1, map2, cv::INTER_LINEAR);
    //cv::imshow("prev_test", image);
    //cv::waitKey(10);
    
    good_pts.clear();
    good_prev_pts.clear();
    status.clear();

    if(prev_image.empty())
    {
       cv_ptr->image.copyTo(prev_image);
       last_image_time = image_time-0.0002f;
       //frame_count=-1;

       for (int i=0;i<12;i++)
       {
          for(int j=0;j<8;j++)
          {
              prev_points.push_back(cv::Point2f(56.f+i*56.f,30.f+60.f*j));
          }
       }
    }
    //double t = clock();
    cv::calcOpticalFlowPyrLK(prev_image, image, prev_points, points, status, err, winSize,
                        2, termcrit, 0, 0.003);
    //ROS_INFO("calofplk, time cost: %f\n", (clock() - t) / CLOCKS_PER_SEC);
    //visualizeText(Eigen::Vector3d(0,0,-2), "calofplk"+ to_string((clock() - t) / CLOCKS_PER_SEC), pub_text);
    if (points.empty()) return;

     int numOfSelectedPoint = 0 ;
     float sumX = 0.0 ;
     float sumY = 0.0 ;

    for(unsigned int i = 0; i<points.size(); i++ )
    {
        //cv::circle(prev_image, prev_points[i], 5, cv::Scalar(0,0,255), 1, 8, 0 );
        if (status[i]){
            good_pts.push_back(points[i]);
            good_prev_pts.push_back(prev_points[i]);
            //cv::circle(prev_image, prev_points[i], 5, cv::Scalar(255,0,255), 2, 8, 0 );

        }
    }
    cv::swap(prev_image, image);
    if (good_pts.empty() || good_prev_pts.empty()) return;
    //t = clock();
    cv::Mat H = cv::findHomography(good_prev_pts, good_pts, 8 , 2 ,mask);
    //ROS_INFO("ransac, time cost: %f\n", (clock() - t) / CLOCKS_PER_SEC);
    //visualizeText(Eigen::Vector3d(0,0,-1), "RANSAC"+ to_string((clock() - t) / CLOCKS_PER_SEC), pub_text);
    
    // TODO: Actually, velocity.z() is not that important for this homework

    if(!mask.empty())
    {
       for (unsigned int i=0;i<good_pts.size();i++)
        {   //double yy=pow((good_prev_pts[i].y-good_pts[i].y),2.0);
            //double xx=pow((good_prev_pts[i].x-good_pts[i].x),2.0);
            if(mask.at<double>(0,i) )//&& sqrt(xx+yy)<12.0)
            {
                
                sumX += good_pts[i].x - good_prev_pts[i].x ;
                sumY += good_pts[i].y - good_prev_pts[i].y ;
                numOfSelectedPoint++;
                //cv::arrowedLine(prev_image, cv::Point2f(good_prev_pts[i].x,good_prev_pts[i].y), cv::Point2f(cvRound(good_prev_pts[i].x+(good_pts[i].x - good_prev_pts[i].x)*2.0), cvRound(good_prev_pts[i].y+(good_pts[i].y - good_prev_pts[i].y)*2.0)),cv::Scalar(255,0,255),3);
               }
       }
        if(numOfSelectedPoint>=5)
       {
        velocity.x()=sumX*height/((image_time-last_image_time)*fx*numOfSelectedPoint);
        velocity.y()=sumY*height/((image_time-last_image_time)*fy*numOfSelectedPoint);
       }

     }
    else return;

    //cv::imshow("prev_test", prev_image);
    //cv::waitKey(10);


    //frame_count++;
    if(print_flag){
        cout<<"OPTICAL FLOW START"<<endl;
        print_flag=false;
    }
    //Eigen::Matrix3d Rr = q.toRotationMatrix();

    //visualizeVelocity(position, velocity, 0, Eigen::Vector3d(1, 0, 0), pub_vel);
    visualizeVelocity(position, velocity_gt, 0, Eigen::Vector3d(0, 1, 0), pub_vel_gt);

    last_position = position;
    last_vicon_time = vicon_time;
    last_image_time = image_time;

    // TODO: 2. Analyze the RMS Error here
    // havn't considered time sychornization
    //if(frame_count>0)
    //{
       //cout<<"VICON speed"<<velocity_gt.x()<<" , "<<velocity_gt.y()<<endl;
       //cout<<"computed velocity"<<velocity.x()<<" , "<<velocity.y()<<endl;
       //RMS_vx_sum += pow((velocity.x()-velocity_gt.x()), 2.0);
       //RMS_vy_sum += pow((velocity.y()-velocity_gt.y()), 2.0);
       //RMS_vx = sqrt(RMS_vx_sum)/frame_count;
       //RMS_vy = sqrt(RMS_vy_sum)/frame_count;

       //cout<<"Sum RMS"<<RMS_vx_sum<<", "<<RMS_vy_sum<<endl;
       //cout<<"Averge RMS vx: "<<RMS_vx<<", vy: "<<RMS_vy<<endl;
    //}

    nav_msgs::Odometry optical_flow_odom;
    optical_flow_odom.header.frame_id ="world";
    optical_flow_odom.header.stamp = image_msg->header.stamp;
    optical_flow_odom.twist.twist.linear.x = -velocity.x();
    optical_flow_odom.twist.twist.linear.y = -velocity.y();
    optical_flow_odom.twist.twist.linear.z = velocity.z();

    optical_flow_odom.pose.pose.position.x = 0;
    optical_flow_odom.pose.pose.position.y = 0;
    optical_flow_odom.pose.pose.position.z = height;
    odom_pub.publish(optical_flow_odom);
    double duration =  ros::Time::now().toSec() - last_run_time;
    visualizeText(Eigen::Vector3d(2,2,3), "OPT Time: " + std::to_string(duration), pub_text);

}


int main(int argc, char **argv) {
    ros::init(argc, argv, "opticalflow_node");
    ros::NodeHandle node;

    cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    fx = cameraMatrix.at<double>(0, 0) = 4.8109068132706136e+02;
    fy = cameraMatrix.at<double>(1, 1) = 4.8065150271909084e+02;
    cx = cameraMatrix.at<double>(0, 2) = 3.2092254241147543e+02;
    cy = cameraMatrix.at<double>(1, 2) = 2.3880401426057333e+02;

    distCoeffs = cv::Mat::zeros(4, 1, CV_64F);
    distCoeffs.at<double>(0, 0) = 9.0980346616300698e-03;
    distCoeffs.at<double>(1, 0) = -4.3334269654755697e-03;
    distCoeffs.at<double>(2, 0) =  7.7683816404546435e-04;
    distCoeffs.at<double>(3, 0) = -1.0475292065355505e-03;


    imageSize.height = 480;
    imageSize.width = 752;
    
    ros::Subscriber sub_height = node.subscribe("/tfmini_ros_node/TFmini", 10, heightCallback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_image = node.subscribe("/camera/image_raw", 1, imageCallback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_vicon = node.subscribe("/uwb_vicon_odom", 10, viconCallback, ros::TransportHints().tcpNoDelay());
    pub_vel = node.advertise<visualization_msgs::Marker>("/optical_flow/velocity", 1, true);
    pub_vel_gt = node.advertise<visualization_msgs::Marker>("/optical_flow/velocity_gt", 1, true);
    odom_pub = node.advertise<nav_msgs::Odometry>("/opticalflow_node/opticalflow_odom", 100);
    
    pub_text = node.advertise<visualization_msgs::Marker>("/opticalflow_node/VICON_text", 1, true);
    
    ros::spin();
    return 0;
}
