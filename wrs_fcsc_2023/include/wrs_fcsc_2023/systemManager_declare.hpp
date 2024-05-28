// #ifndef systemManager_h__
// #define systemManager_h_
#pragma once

//-----------------------------------------------------
// INCLUDE FILES
//-----------------------------------------------------
// standard libraries
#include <iostream>

// ros
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Int8.h>

// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv4/opencv2/aruco.hpp>
#include "opencv2/aruco/dictionary.hpp"

// tf2 for transformation
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>

// pointcloud
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

// realsense
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

// moveit
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

// rviz
#include <visualization_msgs/Marker.h>

// robotiq msg
#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h>

#include <condition_variable>

// msg
// #include <wrs_fcsc_2023/pub_flag_ur3.h>

//-----------------------------------------------------
// NAMESPACE
//-----------------------------------------------------
using namespace cv;
using namespace std;

//-----------------------------------------------------------------
// DEFINES
//-----------------------------------------------------------------
// publisher
ros::Publisher gripperCommand_pub;

/*--- images ---*/
Mat img_src_color;
Mat img_src_yolact;
Mat img_mask;   // mask image from input image
Mat img_output;
Mat img_black;

// -------- draw ------------ // 
double depth_draw = 0;  // depth data for draw
Point point_aruco;  // aruco position
Point3d point3d_aruco;  // aruco position
bool bool_draw1 = false; // whether draw will success or not
bool bool_draw2 = false; // whether draw will success or not

// -------- disposal ------------ // 
vector<double> depth_disposal;
double depth_disposal_one;
double depthToObj;

vector<Point> point2d_obj_centroid; // centroids info
Point point2d_obj_centroid_one;

vector<Point3d> point3d_obj_centroid;
vector<Point3d> point3d_obj_base;
Point3d traget_pos_rough; 

vector<vector<Point>> contours_obj;     // contours from mask image

int minDistance = 560;
int num_minDistance = 0;
Point imgCenter(320, 240);
double distanceFromCenter;
int obj_total = 0;

double wrist_rotation;
double gripper_width;
vector<Point> contours_hand_contact;

// subscribe
std_msgs::Int8 flag_mobile_robot_system;
std_msgs::Int8 flag_customer_system;

// flag
int flag_ur3_draw = 1;  // flag for draw
int flag_ur3_disposal = 1;  // flag for disposal 
int flag_ur3_disposal_getImg = 0;

int flag_ur3_putback = 1;   // flag for putback
int flag_ur3_system = 1;

int flag_customer_first = 0;
int flag_customer_sound = 0;

int flag_system_continue = 0;   // 0: finish whole task, 1: retry

// for save video
VideoWriter writer_src("/home/ubuntupc/ダウンロード/wrs/src.mov", VideoWriter::fourcc('m', 'p', '4', 'v'), 30, Size(640, 480), true);
VideoWriter writer_out("/home/ubuntupc/ダウンロード/wrs/out.mov", VideoWriter::fourcc('m', 'p', '4', 'v'), 30, Size(640, 480), true);

//-----------------------------------------------------------------
// FUNCTIONS
//-----------------------------------------------------------------
// callback
void imageCb_color(const sensor_msgs::ImageConstPtr& msg);  // subscribe raw color image
void imageCb_yolact(const sensor_msgs::ImageConstPtr& msg); // subscribe yolact image
void imageCb_depth_draw(const sensor_msgs::ImageConstPtr& msg); // depth callback for draw task
void imageCb_depth_disposal(const sensor_msgs::ImageConstPtr& msg); // depth callbackf for disposal
void flagCb_mobile_robot(const std_msgs::Int8::ConstPtr& flag_msg);    // to get flag from mobile robot
void flagCb_customer(const std_msgs::Int8::ConstPtr& flag_msg);    // to get flag from customer

// draw
Point detectAruco(Mat img_src); // detect Aruco
Point3d deprojectPixelToPointDraw(Point point_pixel);   // deproject pixel to point3D
void draw();
void birdsEyePose();
geometry_msgs::PoseStamped draw_pose;

// disposal
void disposal();
void pca(vector<Point> &pts);
void moveitFinalPosition(double double_wrist);

// putback
void putBackPose(geometry_msgs::PoseStamped input_pose, Point3d point3d_input);
void detctArucoPose();

//-----------------------------------------------------------------
// GRIPPER
//-----------------------------------------------------------------
void gripperControler(int num)
{
    robotiq_2f_gripper_control::Robotiq2FGripper_robot_output command;

    if ( 0 <= num && num <= 255)
    {
        command.rACT = 1;
        command.rGTO = 1;
        command.rATR = 0;
        command.rPR = num;
        command.rSP = 255;
        command.rFR = 150;
        ROS_INFO("position : done");
    }
    if ( num == 300)    // reset
    {
        command.rACT = 0;
        ROS_INFO("reset: done");
    }
    else if ( num == 400)   // activate
    {
        command.rACT = 1;
        command.rGTO = 1;
        command.rATR = 0;
        command.rPR = 0;
        command.rSP = 255;
        command.rFR = 150;
        ROS_INFO("activate: done");
    }
    else if ( num == 500 )  // close
    {
        command.rACT = 1;
        command.rGTO = 1;
        command.rATR = 0;
        command.rPR = 255;
        command.rSP = 255;
        command.rFR = 150;
        ROS_INFO("close: done");
    }
    else if ( num == 600)   // open
    {
        command.rPR = 0;
        ROS_INFO("open: done");
    }

    gripperCommand_pub.publish(command);
}
