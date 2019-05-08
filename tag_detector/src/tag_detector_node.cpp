#include <iostream>

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/transport_hints.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <Eigen/SVD>
#include<opencv2/core/eigen.hpp>
#include "visualization_msgs/Marker.h"

//EIgen SVD libnary, may help you solve SVD
//JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);

using namespace cv;
using namespace aruco;
using namespace Eigen;

//global varialbles for aruco detector
aruco::CameraParameters CamParam;
MarkerDetector MDetector;
vector<Marker> Markers;
float MarkerSize = 0.20 / 1.5 * 1.524;
float MarkerWithMargin = MarkerSize * 1.2;
BoardConfiguration TheBoardConfig;
BoardDetector TheBoardDetector;
Board TheBoardDetected;
ros::Publisher pub_odom_yourwork;
ros::Publisher pub_odom_ref;
cv::Mat K, D;
double se_x=0;
double se_y=0;
double se_z=0;
double se_roll=0;
double se_pitch=0;
double se_yaw=0;
double rmse_x, rmse_y, rmse_z, rmse_roll, rmse_pitch, rmse_yaw;
int frame=0;
double reproj_error=0;
ros::Publisher pub_text;
bool print_flag=true;

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
// test function, can be used to verify your estimation
void calculateReprojectionError(const vector<cv::Point3f> &pts_3, const vector<cv::Point2f> &pts_2, const cv::Mat R, const cv::Mat t)
{   
    reproj_error= 0;
    
    vector<cv::Point2f> un_pts_2;
    cv::undistortPoints(pts_2, un_pts_2, K, D);
    for (unsigned int i = 0; i < pts_3.size(); i++)
    {
        cv::Mat p_mat(3, 1, CV_64FC1);
        p_mat.at<double>(0, 0) = pts_3[i].x;
        p_mat.at<double>(1, 0) = pts_3[i].y;
        p_mat.at<double>(2, 0) = pts_3[i].z;
        cv::Mat p = (R * p_mat + t);
        //printf("(%f, %f, %f) -> (%f, %f) and (%f, %f)\n",
         //      pts_3[i].x, pts_3[i].y, pts_3[i].z,
          //     un_pts_2[i].x, un_pts_2[i].y,
          //     p.at<double>(0) / p.at<double>(2), p.at<double>(1) / p.at<double>(2));
       reproj_error += pow((un_pts_2[i].x- p.at<double>(0) / p.at<double>(2)),2) + pow((un_pts_2[i].y- p.at<double>(1) / p.at<double>(2)),2);
    }
    reproj_error = sqrt(reproj_error);
    
}

// the main function you need to work with
// pts_id: id of each point
// pts_3: 3D position (x, y, z) in world frame
// pts_2: 2D position (u, v) in image frame
void process(const vector<int> &pts_id, const vector<cv::Point3f> &pts_3, const vector<cv::Point2f> &pts_2, const ros::Time& frame_time)
{
   
    //version 1, as reference
    cv::Mat r, rvec, t;
    cv::solvePnP(pts_3, pts_2, K, D, rvec, t);
    cv::Rodrigues(rvec, r);
    Matrix3d R_ref;
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            R_ref(i,j) = r.at<double>(i, j);
        }
    }
    if(print_flag){
        cout<<"PNP START"<<endl;
        print_flag=false;
    }
    calculateReprojectionError(pts_3,  pts_2, r, t);

    if (reproj_error > 0.15){
        return;
    }
    
    Quaterniond Q_ref;
    Q_ref = R_ref;
    nav_msgs::Odometry odom_ref;
    odom_ref.header.stamp = frame_time;
    odom_ref.header.frame_id = "world";
    odom_ref.pose.pose.position.x = t.at<double>(0, 0);
    odom_ref.pose.pose.position.y = t.at<double>(1, 0);
    odom_ref.pose.pose.position.z = t.at<double>(2, 0);
    odom_ref.pose.pose.orientation.w = Q_ref.w();
    odom_ref.pose.pose.orientation.x = Q_ref.x();
    odom_ref.pose.pose.orientation.y = Q_ref.y();
    odom_ref.pose.pose.orientation.z = Q_ref.z();
    //pub_odom_ref.publish(odom_ref);
    pub_odom_yourwork.publish(odom_ref);
    // version 2, your work
    //Matrix3d R;
    //Vector3d T;
    //R.setIdentity();
    //T.setZero();

    //frame++;
    //vector<cv::Point2f> un_pts_2;
    //cv::undistortPoints(pts_2, un_pts_2, K, D);
    //MatrixXd A(2*pts_2.size(),9);
    //for (unsigned int i=0;i<pts_2.size();i++)
    //{
    //    A.block<2,9>(2*i,0) << pts_3[i].x, pts_3[i].y, 1, 0,0,0, -un_pts_2[i].x*pts_3[i].x,  -un_pts_2[i].x*pts_3[i].y, -un_pts_2[i].x,
    //             0,0,0, pts_3[i].x, pts_3[i].y, 1, -un_pts_2[i].y*pts_3[i].x,  -un_pts_2[i].y*pts_3[i].y, -un_pts_2[i].y;
    //}

    //JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);
    //VectorXd x = svd.matrixV().rightCols(1);
    //Matrix3d H_hat, H_orthogonal;

    //H_hat << x[0], x[1], x[2], x[3], x[4], x[5], x[6], x[7], x[8];

    //if (H_hat(2,2)<0)
    //    H_hat = -H_hat;
    //H_orthogonal << H_hat.col(0), H_hat.col(1), H_hat.col(0).cross(H_hat.col(1));
   
    //JacobiSVD<MatrixXd> svd_hat(H_orthogonal, ComputeThinU | ComputeThinV);
    //R = svd_hat.matrixU()*(svd_hat.matrixV().transpose());
    //T = H_hat.col(2)/(H_hat.col(0).norm());

    //cv::Mat RR(3, 3, CV_64FC1); 
    //cv::eigen2cv(R,RR);

    //Quaterniond Q_yourwork;
    //Q_yourwork = R;
    //nav_msgs::Odometry odom_yourwork;
    //odom_yourwork.header.stamp = frame_time;
    //odom_yourwork.header.frame_id = "world";
    //odom_yourwork.pose.pose.position.x = T(0);
    //odom_yourwork.pose.pose.position.y = T(1);
    //odom_yourwork.pose.pose.position.z = T(2);
    //odom_yourwork.pose.pose.orientation.w = Q_yourwork.w();
    //odom_yourwork.pose.pose.orientation.x = Q_yourwork.x();
    //odom_yourwork.pose.pose.orientation.y = Q_yourwork.y();
    //odom_yourwork.pose.pose.orientation.z = Q_yourwork.z();
    //pub_odom_yourwork.publish(odom_yourwork);

 
    //RMSE  0.236613, 0.188604, 0.589393, 0.122965, 0.204613, 0.550431
    //se_x += pow(odom_yourwork.pose.pose.position.x - odom_ref.pose.pose.position.x, 2);
    //se_y += pow(odom_yourwork.pose.pose.position.y - odom_ref.pose.pose.position.y, 2);
    //se_z += pow(odom_yourwork.pose.pose.position.z - odom_ref.pose.pose.position.z, 2);

    //Eigen::Vector3d eulerAngle=Q_ref.matrix().eulerAngles(2,1,0);
    //Eigen::Vector3d eulerAngle_yourwork=Q_yourwork.matrix().eulerAngles(2,1,0);
    //cout<<"Q_ref"<<eulerAngle<<endl;
    //cout<<"Q_yourwork"<<eulerAngle_yourwork<<endl;
    //se_roll += pow(eulerAngle(0) - eulerAngle_yourwork(0),2);
    //se_pitch += pow(eulerAngle(1) - eulerAngle_yourwork(1),2);
    //se_yaw += pow(eulerAngle(2) - eulerAngle_yourwork(2),2);
    //rmse_x = sqrt(se_x/frame);
    //rmse_y = sqrt(se_y/frame);
    //rmse_z = sqrt(se_z/frame);
    //rmse_roll = sqrt(se_roll/frame);
    //rmse_pitch = sqrt(se_pitch/frame);
    //rmse_yaw = sqrt(se_yaw/frame);
    //ROS_INFO("RMSE X, Y, Z, roll, pitch, yaw: \n %f, %f, %f, %f, %f, %f",
    //         rmse_x, rmse_y, rmse_z, rmse_roll, rmse_pitch,rmse_yaw);

}

cv::Point3f getPositionFromIndex(int idx, int nth)
{
    int idx_x = idx % 6, idx_y = idx / 6;
    double p_x = idx_x * MarkerWithMargin - (3 + 2.5 * 0.2) * MarkerSize;
    double p_y = idx_y * MarkerWithMargin - (12 + 11.5 * 0.2) * MarkerSize;
    return cv::Point3f(p_x + (nth == 1 || nth == 2) * MarkerSize, p_y + (nth == 2 || nth == 3) * MarkerSize, 0.0);
}

void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    double last_run_time = ros::Time::now().toSec();
    double t = clock();
    cv_bridge::CvImagePtr bridge_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    MDetector.detect(bridge_ptr->image, Markers);
    float probDetect = TheBoardDetector.detect(Markers, TheBoardConfig, TheBoardDetected, CamParam, MarkerSize);
    //ROS_INFO("p: %f, time cost: %f\n", probDetect, (clock() - t) / CLOCKS_PER_SEC);
    //visualizeText(Eigen::Vector3d(0,0,-3), "DETECT"+ to_string((clock() - t) / CLOCKS_PER_SEC), pub_text);
    
    vector<int> pts_id;
    vector<cv::Point3f> pts_3;
    vector<cv::Point2f> pts_2;
    for (unsigned int i = 0; i < Markers.size(); i++)
    {
        int idx = TheBoardConfig.getIndexOfMarkerId(Markers[i].id);

        //char str[100];
        //sprintf(str, "%d", idx);
        //cv::putText(bridge_ptr->image, str, Markers[i].getCenter(), CV_FONT_HERSHEY_COMPLEX, 0.4, cv::Scalar(-1));
        //for (unsigned int j = 0; j < 4; j++)
        //{
        //    sprintf(str, "%d", j);
        //    cv::putText(bridge_ptr->image, str, Markers[i][j], CV_FONT_HERSHEY_COMPLEX, 0.4, cv::Scalar(-1));
        //}

        for (unsigned int j = 0; j < 4; j++)
        {
            pts_id.push_back(Markers[i].id * 4 + j);
            pts_3.push_back(getPositionFromIndex(idx, j));
            pts_2.push_back(Markers[i][j]);
        }
    }

    //begin your function
    t = clock();
    if (pts_id.size() > 5)
        process(pts_id, pts_3, pts_2, img_msg->header.stamp);
    //sROS_INFO("process time cost: %f\n", probDetect, (clock() - t) / CLOCKS_PER_SEC); 
    //visualizeText(Eigen::Vector3d(0,0,-3.5), "process"+ to_string((clock() - t) / CLOCKS_PER_SEC), pub_text);
    
   // cv::imshow("in", bridge_ptr->image);
    //cv::waitKey(10);
    double duration =  ros::Time::now().toSec() - last_run_time;
    visualizeText(Eigen::Vector3d(2,2,4), "PNP Time: " + std::to_string(duration), pub_text);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tag_detector");
    ros::NodeHandle n("~");
    ros::Subscriber sub_img = n.subscribe("image_raw", 1, img_callback,ros::TransportHints().tcpNoDelay());
    pub_odom_yourwork = n.advertise<nav_msgs::Odometry>("odom_yourwork",10);
    //pub_odom_ref = n.advertise<nav_msgs::Odometry>("odom_ref",10);
    //init aruco detector
    string cam_cal, board_config;
    n.getParam("cam_cal_file", cam_cal);
    n.getParam("board_config_file", board_config);
    CamParam.readFromXMLFile(cam_cal);
    TheBoardConfig.readFromFile(board_config);

    //init intrinsic parameters
    cv::FileStorage param_reader(cam_cal, cv::FileStorage::READ);
    param_reader["camera_matrix"] >> K;
    param_reader["distortion_coefficients"] >> D;

    //init window for visualization
    //cv::namedWindow("in", 1);
    pub_text = n.advertise<visualization_msgs::Marker>("/tag_detector/text", 1, true);
    
    ros::spin();
}
