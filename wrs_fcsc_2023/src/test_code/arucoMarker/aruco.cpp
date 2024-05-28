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

// standard libraries
#include <iostream>

//-----------------------------------------------------
// DEFINES
//-----------------------------------------------------
using namespace cv;
using namespace std;

static const std::string IMAGE_RAW = "/camera/color/image_raw";


Mat inputImage;
Mat outputImage;
/*************************************************************
 * @Function
 *    aruco_detector
 * @Details
**************************************************************/
void aruco_id_detector(Mat inputImage)
{
  std::vector<int> markerIds;
  std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
  cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    
  cv::aruco::detectMarkers(inputImage, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

  outputImage = inputImage.clone();
  cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);

  if (markerIds.size() > 0) 
  {
    cout << "markerIds[0]" << markerIds[0] << endl;
    Point center;
    center.x = (markerCorners[0][0].x + markerCorners[0][1].x + markerCorners[0][2].x + markerCorners[0][3].x ) / 4;
    center.y = (markerCorners[0][0].y + markerCorners[0][1].y + markerCorners[0][2].y + markerCorners[0][3].y ) / 4;
    cout << "center :" << center << endl;
    circle(outputImage, center, 10, Scalar(255, 255, 255), 8, 8);  
  }

  cv::imshow("out", outputImage);
}

/*************************************************************
 * @Function
 *    read camera parameters
 * @Details
**************************************************************/
static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}

/*************************************************************
 * @Function
 *    pose esteminator
 * @Details
**************************************************************/
void aruco_pose_estimation(Mat inputImage)
{
  std::vector< int > ids;
  std::vector< std::vector<cv::Point2f> > corners;
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
  cv::Ptr<cv::aruco::DetectorParameters> parameters  = cv::aruco::DetectorParameters::create();
  parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

  std::string cameraParamsFilename = "/wrs_fcsc_2023/config/camera.yaml";
  cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) << 618.1471, 0., 334.86355, 0., 619.8749, 242.1743, 0., 0., 1.); 
  cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << 0.170811, -0.409068, -0.003645, 0.007765, 0.0);

  Mat outputImage = inputImage.clone();
  cv::aruco::detectMarkers(outputImage, dictionary, corners, ids, parameters);

  if (ids.size() > 0) {
    cv::aruco::drawDetectedMarkers(outputImage, corners, ids);
    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(corners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);
    for(int i=0; i<ids.size(); i++)
      cv::drawFrameAxes(outputImage, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
  }

  cv::imshow("out", outputImage);
}

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
    inputImage = cv_ptr->image.clone();
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  aruco_id_detector(inputImage);
  // aruco_pose_estimation(inputImage);

  int keyboard = waitKey(1);
//   if ( keyboard == 'q')
//     return;
}


//-----------------------------------------------------
// MAIN
//-----------------------------------------------------
int main(int argc, char **argv)
{
  // The name of the node
  ros::init(argc, argv, "aruco");

  ros::NodeHandle nh("");
  image_transport::ImageTransport it(nh);

  // Subscribe to the /camera raw image topic
  image_transport::Subscriber image_sub_depth = it.subscribe(IMAGE_RAW, 1, imageCb);
  
  ros::spin();
  cv::destroyWindow("out");

  return 0;
}