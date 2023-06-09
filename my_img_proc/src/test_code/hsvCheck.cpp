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
    

    // debug
    cv::circle(output_img, Point(50, 50),  8, cv::Scalar(0, 0, 255), 8, 8);
    cv::waitKey(1);
}

int main()
{
    // if using photo
    //std::string file_name = "./Resources/HSV/lanchpack.jpg";
    // if using video
    std::string file_name = "./Resources/src.mov";
    VideoCapture capture(file_name);
    // if using camera
    //PylonInitialize();
    Pylon::PylonAutoInitTerm autoInitTerm;
    CInstantCamera camera(CTlFactory::GetInstance().CreateFirstDevice());
    cout << "Using device" << camera.GetDeviceInfo().GetVendorName() << " " << camera.GetDeviceInfo().GetModelName() << endl;
    GenApi::INodeMap& nodemap = camera.GetNodeMap();
    camera.Open();
    GenApi::CIntegerPtr width = nodemap.GetNode("Width");
    GenApi::CIntegerPtr height = nodemap.GetNode("Height");
    camera.MaxNumBuffer = 5;
    CImageFormatConverter formatConverter;
    formatConverter.OutputPixelFormat = PixelType_BGR8packed;
    CPylonImage pylonImage;
    int grabbedImages = 0;
    camera.StartGrabbing(c_countOfImagesToGrab, GrabStrategy_LatestImageOnly);
    CGrabResultPtr ptrGrabResult;
    Mat img, imgHSV, imgMask_hand, imgMask_obj, imgErode, imgDilate, imgCanny;
    int hmin_hand = 0, hmax_hand = 179, smin_hand = 0, smax_hand = 255, vmin_hand = 0, vmax_hand = 255;
    int hmin_obj = 0, hmax_obj = 179, smin_obj = 0, smax_obj = 255, vmin_obj = 0, vmax_obj = 255;
    //画像のサイズを確認する
    //画像のリサイズを行う
    //cout << img.size() << '/n';
    //resize(img, imgResize, Size(), 0.2, 0.2);
    // 閾値設定
    namedWindow("Track Bar: HAND", (640, 200));
    createTrackbar("Hue min", "Track Bar: HAND", &hmin_hand, 179);
    createTrackbar("Hue max", "Track Bar: HAND", &hmax_hand, 179);
    createTrackbar("Sat min", "Track Bar: HAND", &smin_hand, 255);
    createTrackbar("Sat max", "Track Bar: HAND", &smax_hand, 255);
    createTrackbar("Val min", "Track Bar: HAND", &vmin_hand, 255);
    createTrackbar("Val max", "Track Bar: HAND", &vmax_hand, 255);
    namedWindow("Track Bar:OBJ", (640, 200));
    createTrackbar("Hue min", "Track Bar:OBJ", &hmin_obj, 179);
    createTrackbar("Hue max", "Track Bar:OBJ", &hmax_obj, 179);
    createTrackbar("Sat min", "Track Bar:OBJ", &smin_obj, 255);
    createTrackbar("Sat max", "Track Bar:OBJ", &smax_obj, 255);
    createTrackbar("Val min", "Track Bar:OBJ", &vmin_obj, 255);
    createTrackbar("Val max", "Track Bar:OBJ", &vmax_obj, 255);
    // if using photo
    capture >> img;
    cvtColor(img, imgHSV, COLOR_BGR2HSV);
    while (true)
    {
        // if using video
        //capture >> img;
        //if (img.empty())  return false;
        // if using camera
        camera.RetrieveResult(INFINITE, ptrGrabResult, TimeoutHandling_ThrowException);
        formatConverter.Convert(pylonImage, ptrGrabResult);
        img = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t*)pylonImage.GetBuffer());
        // HSVに変換
        cvtColor(img, imgHSV, COLOR_BGR2HSV);
        // 収縮膨張
        Mat element4 = (Mat_<uchar>(3, 3) << 0, 1, 0, 1, 1, 1, 0, 1, 0);
        erode(imgHSV, imgErode, element4, Point(-1, -1), 1);
        dilate(imgErode, imgDilate, element4, Point(-1, -1), 1);
        Scalar lower_obj(hmin_obj, smin_obj, vmin_obj);
        Scalar upper_obj(hmax_obj, smax_obj, vmax_obj);
        inRange(imgHSV, lower_obj, upper_obj, imgMask_obj);
        Scalar lower_hand(hmin_hand, smin_hand, vmin_hand);
        Scalar upper_hand(hmax_hand, smax_hand, vmax_hand);
        inRange(imgHSV, lower_hand, upper_hand, imgMask_hand);
        imshow("img", img);
        imshow("HSV", imgHSV);
        imshow("mask obj", imgMask_obj);
        imshow("mask hand", imgMask_hand);
        int keyboard = waitKey(1);
        if (keyboard == 'q')
            return false;
    }
    return 0;
}