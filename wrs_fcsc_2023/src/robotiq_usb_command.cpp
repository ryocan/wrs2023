#include <stdlib.h>
#include <ros/ros.h>
#include <ros/package.h>

int main(int argc, char** argv)
{
    // ros
    ros::init(argc, argv, "robotiq_usb_command");
    ros::NodeHandle nh;
   
    // echo 'password' . this is ""BAD""" code. be careful
    // system("echo 'password'");
    system("echo '3B121'");
    system("sudo -S chmod 666 /dev/ttyUSB0");

    return 0;
}