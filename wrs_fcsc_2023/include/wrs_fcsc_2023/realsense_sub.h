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

// Topics
static const std::string IMAGE_TOPIC = "/yolact_ros/visualization";
static const std::string IMAGE_DEPTH = "/camera/aligned_depth_to_color/image_raw";
static const std::string POINT_CLOUD2_TOPIC = "/camera/depth/color/points";

const std::string from_frame = "wrist_3_link";
const std::string to_frame = "camera_link";

tf2_ros::Buffer tf_buffer;
geometry_msgs::Point box_position_base_frame;

// centroid transfer
geometry_msgs::Point pixel_to_3d_point(const sensor_msgs::PointCloud2 pCloud, const int u, const int v);
geometry_msgs::Point transform_between_frames(geometry_msgs::Point p, const std::string from_frame, const std::string to_frame);

// function
#include "wrs_fcsc_2023/realsense_sub.h"

/*************************************************************
 * @Function
 *    callback
 * @Details
**************************************************************/
void imageCb_depth(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr1;
  try
  {
    cv_ptr1 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
  }
  catch(const std::exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  double distance = 0.;
  distance = 0.001*cv_ptr1->image.at<u_int16_t>(centroid_x, centroid_y);
}

void imageCb_position(const sensor_msgs::ImageConstPtr& msg)
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

  // detect centroid
  vector<Point> center_obj = detect_centroid(src_color);

  cv::waitKey(1);
}

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
                    ,Point(int(centroid_x), int(centroid_y) - 10), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255,255,255), 1);
    }
  }

  cv::imshow("mask", mask_img);
  cv::imshow("centroid", output_img);

  return  center_obj;
}

geometry_msgs::Point pixel_to_3d_point(const sensor_msgs::PointCloud2 pCloud, const int u, const int v)
{
  // get width and height of 2D point cloud data
  int width = pCloud.width;
  int height = pCloud.height;

  // Convert from u (column / width), v (row/height) to position in array
  // where X,Y,Z data starts
  int arrayPosition = v*pCloud.row_step + u*pCloud.point_step;

  // compute position in array where x,y,z data start
  int arrayPosX = arrayPosition + pCloud.fields[0].offset; // X has an offset of 0
  int arrayPosY = arrayPosition + pCloud.fields[1].offset; // Y has an offset of 4
  int arrayPosZ = arrayPosition + pCloud.fields[2].offset; // Z has an offset of 8

  float X = 0.0;
  float Y = 0.0;
  float Z = 0.0;

  memcpy(&X, &pCloud.data[arrayPosX], sizeof(float));
  memcpy(&Y, &pCloud.data[arrayPosY], sizeof(float));
  memcpy(&Z, &pCloud.data[arrayPosZ], sizeof(float));

  geometry_msgs::Point p;
  p.x = X;
  p.y = Y;
  p.z = Z;

  return p;
}

geometry_msgs::Point transform_between_frames(geometry_msgs::Point p, const std::string from_frame, const std::string to_frame) {
    
  geometry_msgs::PoseStamped input_pose_stamped;
  input_pose_stamped.pose.position = p;
  input_pose_stamped.header.frame_id = from_frame;
  input_pose_stamped.header.stamp = ros::Time::now();

  geometry_msgs::PoseStamped output_pose_stamped = tf_buffer.transform(input_pose_stamped, to_frame, ros::Duration(1));

  cout << "x: " << output_pose_stamped.pose.position.x << endl;
  cout << "y: " << output_pose_stamped.pose.position.y << endl;
  cout << "z: " << output_pose_stamped.pose.position.z << endl;

  return output_pose_stamped.pose.position;
}

void point_cloud_cb(const sensor_msgs::PointCloud2 pCloud) {

  geometry_msgs::Point box_position_camera_frame;
  box_position_camera_frame = pixel_to_3d_point(pCloud, centroid_x, centroid_y);

  box_position_base_frame = transform_between_frames(box_position_camera_frame, from_frame, to_frame);
  ROS_INFO_STREAM("3d box position base frame: x " << box_position_base_frame.x << " y " << box_position_base_frame.y << " z " << box_position_base_frame.z);
}
