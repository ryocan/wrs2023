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

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <my_img_proc/Custom.h>
//-----------------------------------------------------
// DEFINES
//-----------------------------------------------------
using namespace cv;
using namespace std;

// image processing
Mat src_color;
Mat mask_img;
Mat output_img;

// detection point
vector<Point> detect_centroid(Mat src_color);
float centroid_x = 0.;
float centroid_y = 0.;
void move_calc(vector<Point> center_obj);

// Topics
static const std::string IMAGE_TOPIC = "/yolact_ros/visualization";
// static const std::string IMAGE_TOPIC = "/camera/color/image_raw";


int flag = 0;
double move_x = 0.;
double move_y = 0.;
Point center;

 ros::Publisher pub;
/*************************************************************
 * IMAGE PROCESSING
**************************************************************/
void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        src_color = cv_ptr->image.clone();
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    //

    center.x = src_color.cols / 2;
    center.y = src_color.rows / 2;

    // detect centroid
    vector<Point> center_obj = detect_centroid(src_color);
    move_calc(center_obj);

    cv::imshow("centroid", output_img);

    // publish
    my_img_proc::Custom data;
    data.move_x = move_x;
    data.move_y = move_y;
    pub.publish(data);

    
    cv::waitKey(1);
}

// image proc
vector<Point> detect_centroid(Mat src_color)
{
  mask_img = src_color.clone();
  output_img = src_color.clone();

  // extract rgb region
  Scalar lower(0, 0, 0);
  Scalar upper(0, 255, 255);
  cv::inRange(mask_img, lower, upper, mask_img);
  bitwise_not(mask_img, mask_img);

  // extract contour
  vector<vector<Point>> contour_obj;
  vector<Vec4i> hierarchy_obj;
  cv::findContours(mask_img, contour_obj, hierarchy_obj, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
  cv::drawContours(output_img, contour_obj, -1, (0, 0, 255), 3);

  // calc mu and mc
  vector<Moments> mu_obj(contour_obj.size());
  for (int i = 0; i < contour_obj.size(); i++)
    mu_obj[i] = cv::moments( contour_obj[i], false );
  vector<Point2f> mc_obj(contour_obj.size());

  // centroid
  vector<Point> center_obj(contour_obj.size());
  for (int i = 0; i < contour_obj.size(); i++)
  {
    centroid_x = mu_obj[i].m10 / mu_obj[i].m00;
    centroid_y = mu_obj[i].m01 / mu_obj[i].m00;
    center_obj[i] = cv::Point(int(centroid_x), int(centroid_y));

    if (isnan(mu_obj[i].m10 / mu_obj[i].m00) != true && isnan(mu_obj[i].m01 / mu_obj[i].m00) != true)
    {
        cv::circle(output_img, center_obj[i], 10, cv::Scalar(255, 255, 255), 8, 8);
        cv::circle(output_img, center_obj[i],  8, cv::Scalar(0, 0, 255), 8, 8);
        cv::circle(output_img, center_obj[i],  2, cv::Scalar(255, 255, 255), 8, 8);
        cv::putText(output_img, 
                    "[" + to_string(center_obj[i].x) + "," + to_string(center_obj[i].y) + "]"
                    ,Point(int(center_obj[i].x), int(center_obj[i].x) - 10), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255,255,255), 1);
    }
  }
      cv::circle(output_img, center, 10, cv::Scalar(255, 255, 255), 8, 8);
    cv::circle(output_img, center,  8, cv::Scalar(0, 255, 0), 8, 8);
    cv::circle(output_img, center,  2, cv::Scalar(255, 255, 255), 8, 8);

  // cv::imshow("mask", mask_img);
//   cv::imshow("centroid", output_img);

  return center_obj;
}

// move calc
void move_calc(vector<Point> center_obj)
{
    // 移動点
    double move_x_arm = -0.04; //-0.025
    double move_y_arm = 0.050;

    move_x = (- double(center.x) + double(center_obj[0].x)) * 5. / 13600. + move_x_arm; // * 5 / 1100
    move_y = (double(center.y) - double(center_obj[0].y)) * 5. / 13600. + move_y_arm;

}

//-----------------------------------------------------
// MAIN
//-----------------------------------------------------
int main(int argc, char **argv)
{
    // The name of the node
    ros::init(argc, argv, "demo_img_proc");
    ros::NodeHandle nh("");
    image_transport::ImageTransport it(nh);

    pub = nh.advertise<my_img_proc::Custom>("my_img_proc_result", 10);

    image_transport::Subscriber image_sub = it.subscribe(IMAGE_TOPIC, 1, imageCb);

    ros::spin();

  return 0;
}