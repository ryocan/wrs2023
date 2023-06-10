// calculate object pca and calclate object pose
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
vector<vector<Point>> contours; 
//--------------------------------------------------------------------------------------------//
Mat extractRegion(int H_MIN, int H_MAX, int S_MIN, int S_MAX, int V_MIN, int V_MAX)
{
    Mat img_mask = Mat::zeros(img_src.size(), img_src.type());

    // convert to HSV image
    Mat img_hsv;
    cvtColor(img_src, img_hsv, COLOR_BGR2HSV);

    // extract region based on HSV parameter
    Scalar Lower(H_MIN, S_MIN, V_MIN);
    Scalar Upper(H_MAX, S_MAX, V_MAX);
    inRange(img_hsv, Lower, Upper, img_mask);

    return img_mask;
}

//--------------------------------------------------------------------------------------------//

vector<vector<Point>> calcContours(Mat img_mask)
{
    vector<vector<Point>> contours_extract; 

    // findContours from img_mask
    vector<vector<Point>> contours; 
    vector<Vec4i> hierarchy;
    cv::findContours(img_mask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // detect based on area info
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

    // debug
    // Mat img_output = img_src.clone();
    // for(int i = 0; i < contours_extract.size(); i++)
    //     cv::drawContours(img_output, contours_extract, i, Scalar(255,255,255), 1, 8);
    // imshow("img_output", img_output);

    return contours_extract;
}

void getOrientation(const std::vector<cv::Point> &pts, Mat img_output)
{
    cv::Mat data_pts = cv::Mat(pts.size(), 2, CV_64F); // [pts.size() x 2] 行列
    for (int i = 0; i < data_pts.rows; i++) {
        data_pts.at<double>(i, 0) = pts[i].x;
        data_pts.at<double>(i, 1) = pts[i].y;
    }

    // do analysis
    cv::PCA pca_analysis(data_pts, cv::Mat(), cv::PCA::DATA_AS_ROW);

    // calc center
    cv::Point cntr = cv::Point(pca_analysis.mean.at<double>(0, 0),
                               pca_analysis.mean.at<double>(0, 1));
    cout << "-----< center point >-----" << endl << pca_analysis.mean << std::endl;

    // eigen 
    std::vector<cv::Point2d> eigen_vecs(2);
    std::vector<double> eigen_val(2);
    for (int i = 0; i < 2; i++) {
        eigen_vecs[i] = cv::Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
                                    pca_analysis.eigenvectors.at<double>(i, 1));
        eigen_val[i] = pca_analysis.eigenvalues.at<double>(i);
    }
    cout << "-----< vectors >-----" << endl << pca_analysis.eigenvectors << std::endl;
    cout << "-----< values >-----" << endl << pca_analysis.eigenvalues << std::endl; 

    // display
    cv::circle(img_output, cntr, 3, cv::Scalar(255, 0, 255), 2);
    cv::Point p1 = cntr + 0.02 * cv::Point(eigen_vecs[0].x * eigen_val[0], eigen_vecs[0].y * eigen_val[0]);
    cv::Point p2 = cntr - 0.02 * cv::Point(eigen_vecs[1].x * eigen_val[1], eigen_vecs[1].y * eigen_val[1]);
    line(img_output, cntr, p1, Scalar(255, 0, 0), 3);
    line(img_output, cntr, p2, Scalar(0, 255, 0), 3);

    // angle
    double angle = atan2(eigen_vecs[0].y, eigen_vecs[0].x);
    ROS_INFO("angle: %lf", angle);

    imshow("img_output", img_output);
}

//-----------------------------------------------------
// calc orientation
//-----------------------------------------------------
void pca()
{
    // extract object's region
    Mat img_mask = extractRegion(0, 13, 100, 255, 110, 255);
    imshow("img_mask", img_mask);

    // calc contours
    contours = calcContours(img_mask);

    // get orientation
    Mat img_output = img_src.clone();
    for (int i = 0; i < contours.size(); i++)
        getOrientation(contours[i], img_output);

}
//-----------------------------------------------------
// CALLBACK
//-----------------------------------------------------
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

    pca();

    cv::waitKey(1);
}

//-----------------------------------------------------
// MAIN
//-----------------------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pca");
    ros::NodeHandle nh("");
    image_transport::ImageTransport it(nh);

    image_transport::Subscriber image_sub_color = it.subscribe("/camera/color/image_raw", 1, imageCb_color);

    ros::spin();
    return 0;
}