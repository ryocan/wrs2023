//-----------------------------------------------------
// INCLUDE FILES
//-----------------------------------------------------
// about image processing
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv4/opencv2/aruco.hpp>
#include "opencv2/aruco/dictionary.hpp"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <image_transport/image_transport.h>
#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h>

// standard libraries
#include <iostream>

//-----------------------------------------------------
// DEFINES
//-----------------------------------------------------
using namespace std;

//-----------------------------------------------------
// Function
//-----------------------------------------------------
ros::Publisher gripperCommand_pub;

void chatterCallback(const std_msgs::Int16 fig)
{
    robotiq_2f_gripper_control::Robotiq2FGripper_robot_output command;

    if ( 0 <= fig.data && fig.data <= 255)
    {
        command.rPR = fig.data;
        ROS_INFO("open position : done");
    }
    if ( fig.data == 300)
    {
        command.rACT = 0;
        ROS_INFO("reset: done");
    }
    else if ( fig.data == 400)
    {
        command.rACT = 1;
        command.rGTO = 1;
        command.rATR = 0;
        command.rPR = 0;
        command.rSP = 255;
        command.rFR = 150;
        ROS_INFO("activate: done");
    }
    else if ( fig.data == 500 )
    {
        command.rPR = 255;
        ROS_INFO("close: done");
    }
    else if ( fig.data == 600)
    {
        command.rPR = 0;
        ROS_INFO("open: done");
    }

    
    gripperCommand_pub.publish(command);
}

//-----------------------------------------------------
// MAIN
//-----------------------------------------------------
int main(int argc, char** argv)
{
    ros::init(argc, argv, "gripper_pub");
    ros::NodeHandle nh;

    gripperCommand_pub = nh.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("Robotiq2FGripperRobotOutput", 10);

    ros::Subscriber sub = nh.subscribe("gripperCommandSub", 10, chatterCallback);

    ros::Rate loop_rate(10);
    ros::spin();

    return 0;
}