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
// static const std::string IMAGE_TOPIC = "/yolact_ros/visualization";
static const std::string IMAGE_TOPIC = "/camera/color/image_raw";

// static const std::string IMAGE_DEPTH = "/camera/aligned_depth_to_color/image_raw";
static const std::string POINT_CLOUD2_TOPIC = "/camera/depth/color/points";

const std::string from_frame = "camera_link";
const std::string to_frame = "wrist_3_link";

tf2_ros::Buffer tf_buffer;
geometry_msgs::Point box_position_arm_frame;

// centroid transfer
geometry_msgs::Point pixel_to_3d_point(const sensor_msgs::PointCloud2 pCloud, const int u, const int v);
geometry_msgs::Point transform_between_frames(geometry_msgs::Point p, const std::string from_frame, const std::string to_frame);

/*************************************************************
 * @Function
 *    callback
 * @Details
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

    // detect centroid
    vector<Point> center_obj = detect_centroid(src_color);
    
    cv::waitKey(1);
}
/*************************************************************
 * @Function
 *    callback
 * @Details
**************************************************************/
void intrin()
{
    rs2::frameset current_frameset;
    cout << "1" << endl;
    auto depth = current_frameset.get_depth_frame();
    cout << "2" << endl;
    rs2_intrinsics intr = depth.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
    cout << "3" << endl;
}

static void rs2_deproject_pixel_to_point(const struct rs2_intrinsics * intrin, const float pixel[2], float depth)
{
    float point[3];


    assert(intrin->model != RS2_DISTORTION_MODIFIED_BROWN_CONRADY); // Cannot deproject from a forward-distorted image
    assert(intrin->model != RS2_DISTORTION_FTHETA); // Cannot deproject to an ftheta image
    //assert(intrin->model != RS2_DISTORTION_BROWN_CONRADY); // Cannot deproject to an brown conrady model

    float x = (pixel[0] - intrin->ppx) / intrin->fx;
    float y = (pixel[1] - intrin->ppy) / intrin->fy;
    if(intrin->model == RS2_DISTORTION_INVERSE_BROWN_CONRADY)
    {
        float r2  = x*x + y*y;
        float f = 1 + intrin->coeffs[0]*r2 + intrin->coeffs[1]*r2*r2 + intrin->coeffs[4]*r2*r2*r2;
        float ux = x*f + 2*intrin->coeffs[2]*x*y + intrin->coeffs[3]*(r2 + 2*x*x);
        float uy = y*f + 2*intrin->coeffs[3]*x*y + intrin->coeffs[2]*(r2 + 2*y*y);
        x = ux;
        y = uy;
    }
    point[0] = depth * x;
    point[1] = depth * y;
    point[2] = depth;
}


void imageCb_depth(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr1;
  try
  {
    cv_ptr1 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    cout << "depth" << endl;
  }
  catch(const std::exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  double distance = 0.;
  distance = 0.001*cv_ptr1->image.at<u_int16_t>(50, 50);
  cout << "distance: " << distance << endl;

  intrin();
}
/*************************************************************
 * @Function
 *    detect centroid
 * @Details
**************************************************************/
vector<Point> detect_centroid(Mat src_color)
{
  mask_img = src_color.clone();
  output_img = src_color.clone();

  // extract rgb region
  // Scalar lower(0, 0, 0);
  // Scalar upper(0, 255, 255);
  // cv::inRange(mask_img, lower, upper, mask_img);
  // bitwise_not(mask_img, mask_img);

  // hsv
  cvtColor(mask_img, mask_img, COLOR_BGR2HSV);
  imshow("hsv", mask_img);
  Scalar lower(0, 115, 120);
  Scalar upper(15, 255, 255);
  cv::inRange(mask_img, lower, upper, mask_img);

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
  cout << "contour_obj.size()): " << contour_obj.size() << endl;
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

  // 中心点
  Point center;
  center.x = src_color.cols / 2 ;
  center.y = src_color.rows / 2;
  cv::circle(output_img, center, 10, cv::Scalar(255, 255, 255), 8, 8);
  cv::circle(output_img, center,  8, cv::Scalar(0, 255, 0), 8, 8);
  cv::circle(output_img, center,  2, cv::Scalar(255, 255, 255), 8, 8);

  cout << "center_obj[0].x: " << center_obj[0].x << ": enter_obj[0].y " << center_obj[0].y << endl;
  cout << "center_obj[1].x: " << center_obj[1].x << ": enter_obj[1].y " << center_obj[1].y << endl;


  // 移動点
  cout << center << endl;
  double move_x = (double(center.x) - double(center_obj[0].x)) * 5. / 1100.; // * 5 / 1100
  double move_y = (double(center.y) - double(center_obj[0].y)) * 5. / 1100.;
  // cout << "move_x = " << move_x << endl;
  // cout << "move_y = " << move_y << endl;

  cv::imshow("mask", mask_img);
  cv::imshow("centroid", output_img);

  

  return  center_obj;
}
/*************************************************************
 * @Function
 *    transform pixel to 3d point
 * @Details
**************************************************************/
geometry_msgs::Point pixel_to_3d_point(const sensor_msgs::PointCloud2 pCloud, const int target_u, const int target_v)
{
    int width = pCloud.width;
    int height = pCloud.height;


    // int arrayPosition = target_v * pCloud.row_step + target_u * ;
    int arrayPosition = target_v * pCloud.row_step/width + target_u * pCloud.point_step / height;

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
     cout<<"Pt x"<<p.x<<endl;
     cout<<"Pt "<<p.z<<endl;

    return p;
}
/*************************************************************
 * @Function
 *    transform from camela_link to wrist3_link
 * @Details
**************************************************************/
geometry_msgs::Point transform_between_frames(geometry_msgs::Point p, const std::string from_frame, const std::string to_frame) 
{
  geometry_msgs::PoseStamped input_pose_stamped;
  input_pose_stamped.pose.position = p;
  input_pose_stamped.header.frame_id = from_frame;
  input_pose_stamped.header.stamp = ros::Time::now();

  geometry_msgs::PoseStamped output_pose_stamped = tf_buffer.transform(input_pose_stamped, to_frame, ros::Duration(1));

  return output_pose_stamped.pose.position;
}



/*************************************************************
 * @Function
 *    point_cloud_cb
 * @Details
**************************************************************/
void point_cloud_cb(const sensor_msgs::PointCloud2 pCloud) 
{
    geometry_msgs::Point box_position_camera_frame;
    
    box_position_camera_frame = pixel_to_3d_point(pCloud, centroid_x, centroid_y);
    cout << "wow" << endl;

    // box_position_arm_frame = transform_between_frames(box_position_camera_frame, from_frame, to_frame);
    // ROS_INFO_STREAM("3d box position base frame: x " << box_position_arm_frame.x 
    //     << " y " << box_position_arm_frame.y 
    //     << " z " << box_position_arm_frame.z);
}

//-----------------------------------------------------
// MAIN
//-----------------------------------------------------
int main(int argc, char **argv)
{
    // The name of the node
    ros::init(argc, argv, "realsense_sub");
    ros::NodeHandle nh("");
    image_transport::ImageTransport it(nh);

    tf2_ros::TransformListener listener(tf_buffer);

    image_transport::Subscriber image_sub = it.subscribe(IMAGE_TOPIC, 1, imageCb);
    // image_transport::Subscriber imageD_sub = it.subscribe(IMAGE_DEPTH, 1, imageCb_depth);
    // ros::Subscriber point_cloud_sub = nh.subscribe(POINT_CLOUD2_TOPIC, 1, point_cloud_cb);


    
  

    ros::spin();
    cv::destroyWindow("view");

  return 0;
}