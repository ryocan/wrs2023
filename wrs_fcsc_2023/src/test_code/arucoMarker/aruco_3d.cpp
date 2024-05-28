// publish pixel_x, pixel_y, depth using realsense d435i
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

#include <wrs_fcsc_2023/real_pub.h>
//-----------------------------------------------------
// DEFINES
//-----------------------------------------------------
using namespace std;
using namespace cv;

Mat img_src;
Mat img_output;
int target_x = 320;
int target_y = 100;
/*************************************************************
 * @Function
 *    image processing
**************************************************************/
Point calcTargetPoint(Mat img_src)
{
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    
    cv::aruco::detectMarkers(img_src, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

    img_output = img_src.clone();
    cv::aruco::drawDetectedMarkers(img_output, markerCorners, markerIds);

    Point center;
    
    if(markerIds.size() > 0)
    {
    cout << "markerIds[0]" << markerIds[0] << endl;
    center.x = (markerCorners[0][0].x + markerCorners[0][1].x + markerCorners[0][2].x + markerCorners[0][3].x ) / 4;
    center.y = (markerCorners[0][0].y + markerCorners[0][1].y + markerCorners[0][2].y + markerCorners[0][3].y ) / 4;
    cout << "center :" << center << endl;
    circle(img_output, center, 10, Scalar(255, 255, 255), 8, 8);
    }
    
    cv::imshow("out", img_output);
    return center;
}


/*************************************************************
 * @Function
 *    transform from camela_link to wrist3_link
 * @Details
**************************************************************/
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

    // detect centroid
    Point targetPoint = calcTargetPoint(img_src);
    target_x = targetPoint.x;
    target_y = targetPoint.y;

    cv::waitKey(1);
}

void imageCb_depth(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    double distance = 0.;
    distance = 0.001*cv_ptr->image.at<u_int16_t>(target_x, target_y);
    if (distance > 1.)
        distance -= 1.0;

    wrs_fcsc_2023::real_pub data;
    data.pixel_x = target_x;
    data.pixel_y = target_y;
    data.depth = distance;
    pub.publish(data);
    
}


//-----------------------------------------------------
// MAIN
//-----------------------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "aruco_3d");
    ros::NodeHandle nh("");
    image_transport::ImageTransport it(nh);

    image_transport::Subscriber image_sub_depth = it.subscribe("/camera/aligned_depth_to_color/image_raw", 1, imageCb_depth);
    image_transport::Subscriber image_sub_color = it.subscribe("/camera/color/image_raw", 1, imageCb_color);

    pub = nh.advertise<wrs_fcsc_2023::real_pub>("real_pub", 10);


    ros::spin();
    return 0;
}