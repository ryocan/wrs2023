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

#include <my_img_proc/real_pub.h>
//-----------------------------------------------------
// DEFINES
//-----------------------------------------------------
using namespace std;
using namespace cv;

Mat img_src;
int target_x = 320;
int target_y = 100;
/*************************************************************
 * @Function
 *    image processing
**************************************************************/
Point calcTargetPoint(Mat img_src)
{
    Point targetPoint;

    /*----- extract region ----*/ 
    Mat img_mask = Mat::zeros(img_src.size(), img_src.type());
    Mat img_hsv;
    cvtColor(img_src, img_hsv, COLOR_BGR2HSV);
    Scalar Lower(0, 100, 110);
    Scalar Upper(13, 255, 255);
    inRange(img_hsv, Lower, Upper, img_mask);
    imshow("img_mask", img_mask);
    
    /*----- calc contours ----*/ 
    vector<vector<Point>> contours; 
    vector<Vec4i> hierarchy;
    cv::findContours(img_mask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    vector<vector<Point>> contours_extract; 
    double area = 0.;
    double area_prev = 0.;
    for (int i = 0; i < contours.size(); i++)
    {
        area = contourArea(contours[i]);
        if (area > area_prev)
        {
            contours_extract.clear();
            contours_extract.shrink_to_fit();
            contours_extract.push_back(contours[i]);
            area_prev = area;
        }
    }
    Mat img_output = img_src.clone();
    // for(int i = 0; i < contours_extract.size(); i++)
    //     cv::drawContours(img_output, contours_extract, i, Scalar(255,255,255), 1, 8);

    /*----- calc centroid ----*/
    vector<Moments> mu;
    for (int i = 0; i < contours_extract.size(); i++)
        mu.push_back(moments(contours_extract[i], false));

    // calc mc    
    vector<Point2f> mc(contours_extract.size());
    for (int i = 0; i < mu.size(); i++)
    {
        mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);

        if (isnan(mu[i].m10 / mu[i].m00) != true && isnan(mu[i].m01 / mu[i].m00) != true)
        {
            targetPoint.x = mc[i].x;
            targetPoint.y = mc[i].y;
            
            // display
            circle(img_output, mc[i], 10, Scalar(255, 255, 255), 8, 8);
            circle(img_output, mc[i],  8, Scalar(  0,   0, 255), 8, 8);
            circle(img_output, mc[i],  2, Scalar(255, 255, 255), 8, 8);
        }
    }

    /*----- output ----*/ 
    imshow("img_output", img_output);
    // ROS_INFO("targetPoint.x = %d", targetPoint.x);
    // ROS_INFO("targetPoint.y = %d", targetPoint.y);
    return targetPoint;
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

    my_img_proc::real_pub data;
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
    ros::init(argc, argv, "pixelToPoint");
    ros::NodeHandle nh("");
    image_transport::ImageTransport it(nh);

    image_transport::Subscriber image_sub_depth = it.subscribe("/camera/aligned_depth_to_color/image_raw", 1, imageCb_depth);
    image_transport::Subscriber image_sub_color = it.subscribe("/camera/color/image_raw", 1, imageCb_color);

    pub = nh.advertise<my_img_proc::real_pub>("pixelToPoint", 10);


    ros::spin();
    return 0;
}