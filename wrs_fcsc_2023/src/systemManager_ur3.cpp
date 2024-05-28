#include "wrs_fcsc_2023/systemManager.hpp"
// image 
Mat img_src_yolact;
Mat img_output;

// depth info from RealSense
vector<double> depth_rs;

// point2D
vector<Point> point2d_obj_centroid; // centroids info

//-----------------------------------------------------------------
// CALLBACK
//-----------------------------------------------------------------
/*******************************************************
 * Callback function for subscribing 2D image
********************************************************/
void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        img_src_yolact = cv_ptr->image.clone();
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

/*******************************************************
 * Callback function for subscribing depth info from RealSense
********************************************************/
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

    // get depth info 
    depth_rs.clear();
    for (int i = 0; i < point2d_obj_centroid.size(); i++)
    {
        double calc =  0.001*cv_ptr->image.at<u_int16_t>(point2d_obj_centroid[i].y, point2d_obj_centroid[i].x);
        depth_rs.push_back(calc);
    }
}

//-----------------------------------------------------------------
// IMAGE PROCESSING
//-----------------------------------------------------------------
/*******************************************************
 * Perform thresholding on the image to extract only 
 * the desired object regions. 
 * Input: image, Output: mask image
********************************************************/
Mat createMaskImg(Mat img_input)
{
    // declre output img
    Mat img_mask = Mat::zeros(img_src_yolact.size(), img_src_yolact.type());

    Mat img_hsv;
    cvtColor(img_input, img_hsv, CV_BGR2HSV, 3);

    // extract object from YOLACT result using RGB info
    // only extract black background and then invert colors
    Scalar lower(  0,   0,   0);
    Scalar upper(  0,   0,   0);
    cv::inRange(img_hsv, lower, upper, img_mask);
    bitwise_not(img_mask, img_mask);

    // show result
    imwrite("/home/umelab/Downloads/wrs_output_debug/createMaskImg.png", img_mask);

    return img_mask;
}

/*******************************************************
 * Get object's contours from mask image
 * Input: mask image, Output: contours
********************************************************/
vector<vector<Point>> extractObjectContours(Mat img_input)
{
    // extract contours from img_mask
    vector<vector<Point>> contour_output;
    vector<Vec4i> hierarchy;
    cv::findContours(img_input, contour_output, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    
    // show result
    cv::drawContours(img_output, contour_output, -1, (0, 0, 255), 3);
    imshow("/home/umelab/Downloads/wrs_output_debug/extractObjectContours.png", img_output);

    return contour_output;
}

/*************************************************************
 * Calculation object's centroid from object's contour info
 * Input: contour info, Output: centroid point2D
**************************************************************/
vector<Point> calcCentroid(vector<vector<Point>> contour_input)
{
    // declare output Point2D
    vector<Point> point_output(contour_input.size());

    // prepare mu and mc for centroid detection
    vector<Moments> mu_obj(contour_input.size());
    for (int i = 0; i < contour_input.size(); i++)
        mu_obj[i] = cv::moments(contour_input[i], false );
    vector<Point2f> mc_obj(contour_input.size());

    // calculation centroid
    for (int i = 0; i < contour_input.size(); i++)
    {
        double centroid_x = mu_obj[i].m10 / mu_obj[i].m00;
        double centroid_y = mu_obj[i].m01 / mu_obj[i].m00;
        point_output[i] = cv::Point(int(centroid_x), int(centroid_y));

        if (isnan(mu_obj[i].m10 / mu_obj[i].m00) != true && isnan(mu_obj[i].m01 / mu_obj[i].m00) != true)
        {
            cv::circle(img_output, point_output[i], 10, cv::Scalar(255, 255, 255), 8, 8);
            cv::circle(img_output, point_output[i],  8, cv::Scalar(0, 0, 255), 8, 8);
            cv::circle(img_output, point_output[i],  2, cv::Scalar(255, 255, 255), 8, 8);
        }
    }

    // show result
    cout << "point_output: " << endl << point_output << endl;
    imwrite("/home/umelab/Downloads/wrs_output_debug/calcCentroid.png", img_output);


    return point_output;
}

//-----------------------------------------------------------------
// GRIPPER
//-----------------------------------------------------------------
/*************************************************************
 * Receives commands and controls the gripper
**************************************************************/
void gripperControler(int num)
{
    robotiq_2f_gripper_control::Robotiq2FGripper_robot_output command;

    if ( 0 <= num && num <= 255)
    {
        command.rACT = 1;
        command.rGTO = 1;
        command.rATR = 0;
        command.rPR = num;
        command.rSP = 255;
        command.rFR = 150;
        ROS_INFO("position : done");
    }
    if ( num == 300)    // reset
    {
        command.rACT = 0;
        ROS_INFO("reset: done");
    }
    else if ( num == 400)   // activate
    {
        command.rACT = 1;
        command.rGTO = 1;
        command.rATR = 0;
        command.rPR = 0;
        command.rSP = 255;
        command.rFR = 150;
        ROS_INFO("activate: done");
    }
    else if ( num == 500 )  // close
    {
        command.rACT = 1;
        command.rGTO = 1;
        command.rATR = 0;
        command.rPR = 255;
        command.rSP = 255;
        command.rFR = 150;
        ROS_INFO("close: done");
    }
    else if ( num == 600)   // open
    {
        command.rPR = 0;
        ROS_INFO("open: done");
    }

    gripperCommand_pub.publish(command);
}


void disposal()
{
    // setup
    img_output = img_src_yolact.clone();

    // create mask image
    Mat img_mask = createMaskImg(img_src_yolact);

    // get all object's contours
    vector<vector<Point>> contours_obj;
    contours_obj = extractObjectContours(img_mask);

    // calclate centroid from contour
    point2d_obj_centroid = calcCentroid(contours_obj);

    // for object size
    for (int i = 0; i < 100; i++)
    {
        ros::NodeHandle nh;
        ros::Subscriber sub;
        sub = nh.subscribe("flag",1,flagCallback);
        cout << "i: " << i << "flag: " << flag << endl;
    }
    cout << "ww" << endl;

    waitKey(1);
}