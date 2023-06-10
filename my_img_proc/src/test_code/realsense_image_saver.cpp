// publish pixel_x, pixel_y, depth using realsense d435i
//-----------------------------------------------------
// INCLUDE FILES
//-----------------------------------------------------
// about image processing
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>

// Include tf2 for transformation
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// pointcloud
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

// standard libraries
#include <iostream>

// #include <my_img_proc/real_pub.h>

#include <sys/stat.h>   //mkdir
#include <sys/types.h>
#include <filesystem>
#include <std_msgs/String.h>
#include <ros/package.h>    // ros package
//-----------------------------------------------------
// DEFINES
//-----------------------------------------------------
using namespace std;
using namespace cv;

Mat img_src;
int n = 96;
/*************************************************************
 * @Function
 *    transform from camela_link to wrist3_link
 * @Details
**************************************************************/
bool IsFileExist(const std::string& name) 
{
    struct stat buffer;
    return (stat(name.c_str(), &buffer) == 0 && S_ISREG(buffer.st_mode));
}

void savePicture()
{
    // save video and make directory automatically
    string dir = ros::package::getPath("my_img_proc") + "/img/230609_yolact/230609_"; //230609_yolact/
   
    // rename folder
    if (n < 10)
        dir += "00" + to_string(n);
    else if (n < 100)
        dir += "0" + to_string(n);
    else
        dir += to_string(n);
    
    dir = dir + ".jpg";
    imwrite(dir, img_src);
    n++;
}

ros::Publisher pub;
void imageCb_color(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        img_src = cv_ptr->image.clone();
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // display
    imshow("img_src", img_src);

    // save
    int key = waitKey(1);
    if (key == 'q' || key == 'Q')
        savePicture();
}


//-----------------------------------------------------
// MAIN
//-----------------------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pixelToPoint");
    ros::NodeHandle nh("");
    image_transport::ImageTransport it(nh);

    image_transport::Subscriber image_sub_color = it.subscribe("/camera/color/image_raw", 1, imageCb_color);

    ros::spin();
    return 0;
}