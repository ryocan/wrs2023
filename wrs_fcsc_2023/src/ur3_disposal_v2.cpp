//-----------------------------------------------------------------
// INCLUDE FILES
//-----------------------------------------------------------------
// about image processing
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>

// tf2 for transformation
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// pointcloud
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

// standard libraries
#include <iostream>

// realsense
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

// moveit
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

// tf
#include <tf/transform_listener.h>

// rviz
#include <visualization_msgs/Marker.h>

// robotiq msg
#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h>

//-----------------------------------------------------------------
// DEFINES
//-----------------------------------------------------------------
/*--- namespace ---*/
using namespace cv;
using namespace std;

/*--- images ---*/
Mat img_src_yolact; // subscribe yolact image
Mat img_mask;   // 
Mat img_black;
Mat img_output;

/*--- contours ---*/
vector<vector<Point>> contours_obj;

/*--- points ---*/
vector<Point> point2d_obj_centroid; // centroids info

/*--- points ---*/
Point3d point3d_obj_centroid;
Point3d point3d_base;
Point3d point3d_wrist;

vector<Point3d> point_obj_3d;
vector<Point> contours_hand_contact;


// depth info from RealSense
vector<double> depth_rs;

// disposal
double wrist_rotation;
double gripper_width;

// hsv 
vector<int> th_obj_hsv{20, 0, 0, 179, 255, 255};

// flag
int flag_state = 0; // used in main function in switch case 
int flag_state_one = 0;
int flag_deprojection = 0;  // used in deprojectPixelToPoint
int flag_if = 0;

//-----------------------------------------------------------------
// FUNCTIONS
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

/*******************************************************
 * Perform thresholding on the image to extract only 
 * the desired object regions. 
 * Input: image, Output: mask image
********************************************************/
Mat createMaskImg(Mat img_input, vector<int> th_hsv)
{
    // declre output img
    Mat img_mask = Mat::zeros(img_src_yolact.size(), img_src_yolact.type());

    Mat img_hsv;
    cvtColor(img_input, img_hsv, CV_BGR2HSV, 3);

    // extract object from YOLACT result using HSV info
    Scalar lower(th_hsv[0], th_hsv[1], th_hsv[2]);
    Scalar upper(th_hsv[3], th_hsv[4], th_hsv[5]);
    cv::inRange(img_hsv, lower, upper, img_mask);

    // show result
    imshow("img_mask", img_mask);
    imwrite("/home/ubuntupc/ダウンロード/wrs/img_mask.png", img_mask);

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
    imshow("RESULT: extractObjectContours", img_output);
    imwrite("/home/ubuntupc/ダウンロード/wrs/img_output.png", img_output);

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
    imshow("RESULT: calcCentroid", img_output);
    imwrite("/home/ubuntupc/ダウンロード/wrs/img_output_centroid.png", img_output);

    return point_output;
}

/*************************************************************
 * Converting 2D coordinates obtained from image processing 
 * to 3D coordinates using RealSense depth information.
 * Input: Point2D, Output: Point3D
**************************************************************/
Point3d deprojectPixelToPoint(Point point2d_input, int i)
{
    // declare output Point3D
    Point3d point3d_output;

    if (!depth_rs.empty())
    {
        // calculation (using parameter from /camera/depth_rs/camera_info)
        double x = 0.;
        double y = 0.;
        x = (point2d_input.x - 321.7812805175781) / 380.3281555175781;
        y = (point2d_input.y - 238.0182342529297) / 380.3281555175781;

        // In the original python code, they use BROWN_CONRADY, but we have to use plum_bob.
        // however, the distortion coefficients is same, we write the same code
        double r2 = x * x + y * y;
        double f = 1 + 0*r2 + 0*r2*r2 + 0*r2*r2*r2;
        double ux = x*f + 2*0*x*y + 0*(r2 + 2*x*x);
        double uy = y*f + 2*0*x*y + 0*(r2 + 2*y*y);

        Point3d temp(depth_rs[i] * ux, (-1) * depth_rs[i] * uy, depth_rs[i]);
        point3d_output = temp;

        // show result
        cout << "RESULT: deprojectPixelToPoint " << endl << point3d_output << endl;
        
        // update flag
        flag_deprojection = 1;
    }

    return point3d_output;
}

/*************************************************************
 * Transform object's point(3D) from wrist_3_link coordinate
 * to base_link coordinate using /tf.
 * Input: Object's Point3D, Output: Object's Point3D in base_link
**************************************************************/
Point3d transformCoordinateToBase(Point3d point3d_input)
{
    //
    Point3d point3d_output;

    // tf listner declare
    tf::TransformListener listener;
    
    // declare target frame
    std::string targetFrame = "base_link";

    // define object point
    geometry_msgs::PointStamped objectPoint;
    objectPoint.header.frame_id = "wrist_3_link";
    objectPoint.point.x = point3d_input.x + 0.035; // camera -> wrist_3_link
    objectPoint.point.y = point3d_input.y - 0.055; // camera -> wrist_3_link
    objectPoint.point.z = point3d_input.z;

    // transform point
    geometry_msgs::PointStamped transformedPoint;
    try
    {
        listener.waitForTransform(targetFrame, objectPoint.header.frame_id, ros::Time(0), ros::Duration(3.0));
        listener.transformPoint(targetFrame, objectPoint, transformedPoint);
    }
    catch (tf::TransformException& ex)
    {
        ROS_ERROR("Failed to transform wrist point: %s", ex.what());
    }

    // output
    double x = transformedPoint.point.x;
    double y = transformedPoint.point.y;
    double z = transformedPoint.point.z;
    point3d_output = Point3d(x - 0.035, y + 0.055, z);   // wrist -> camera

    // show result
    cout << "RESULT: transformCoordinateToBase" << point3d_output << endl;     

    return point3d_output;
}

/*************************************************************
 * To reach the target position, first go to the initial posture.
**************************************************************/
void moveitInitialPose()
{
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    ROS_INFO_NAMED("Initial Pose", "Planning frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("Initial Pose", "End effector link: %s", move_group.getEndEffectorLink().c_str());
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
    
    // ------------------- Collision Setup
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(4);

    collision_objects[0].id = "table";
    collision_objects[0].header.frame_id = move_group.getPlanningFrame();
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 1.0;
    collision_objects[0].primitives[0].dimensions[1] = 1.0;
    collision_objects[0].primitives[0].dimensions[2] = 0.2;

    // define pose
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.0;
    collision_objects[0].primitive_poses[0].position.y = 0.0;
    collision_objects[0].primitive_poses[0].position.z = -0.15;
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;
    planning_scene_interface.applyCollisionObjects(collision_objects);
    ROS_INFO_NAMED("Initial Pose", "Add an object into the world");
    ros::Duration(1.0).sleep();

    // ------------------- Pose Setup
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    
    // set joint radius
    joint_group_positions[1] = -1.27409;
    joint_group_positions[2] = -1.27409;
    joint_group_positions[3] = -2.18166;
    move_group.setJointValueTarget(joint_group_positions);

    // move setup
    move_group.setMaxVelocityScalingFactor(0.5);
    move_group.setMaxAccelerationScalingFactor(0.5);
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("Initial Pose", "Visualizing plan (joint space goal) %s", success ? "" : "FAILED");

    // move
    move_group.move();
}

/*************************************************************
 * Transform object's point(3D) from wrist_3_link coordinate
 * to base_link coordinate using /tf.
 * Input: Object's Point3D, Output: Object's Point3D in base_link
**************************************************************/
Point3d transformCoordinateToWrist(Point3d point3d_input)
{
    // output
    Point3d point3d_output;

    // tf
    tf::TransformListener listener;

    // Specify the source coordinate frame and the target coordinate frame
    std::string sourceFrame = "base_link";
    std::string targetFrame = "wrist_3_link";

    // object position in wrist_3_link
    double x = point3d_input.x + 0.035;
    double y = point3d_input.y - 0.055;
    double z = point3d_input.z;

    // convert object position into tf::StampedTransform
    tf::Stamped<tf::Pose> objectPose;
    objectPose.frame_id_ = sourceFrame;
    objectPose.stamp_ = ros::Time(0);
    objectPose.setOrigin(tf::Vector3(x, y, z));

    // calc
    tf::Stamped<tf::Pose> transformedPose;
    try 
    {
        // get latest tf info
        listener.waitForTransform(targetFrame, sourceFrame, ros::Time(0), ros::Duration(3.0));

        // do transform
        listener.transformPose(targetFrame, objectPose, transformedPose);
    } 
    catch (tf::TransformException& ex) 
    {
        ROS_ERROR("Failed transformation: %s", ex.what());
    }

    // output
    double transformedX = transformedPose.getOrigin().x();;
    double transformedY = transformedPose.getOrigin().y();
    double transformedZ = transformedPose.getOrigin().z();
    double x_input = transformedX - 0.035;
    double y_input = abs(transformedY + 0.055);
    if (y_input >= 0.3)     // error: if y_input >=0.3, robotic arm will collision with shelf
        y_input = 0.3;
    point3d_output = Point3d(x_input, y_input , transformedZ);    // be careful

    // debug
    cout << "point3d_wrist :" << point3d_output << endl;  

    return point3d_output; 
}

/*************************************************************
 * First, UR3 will move to the position of the object 
 * calculated by the first image recognition. 
 * Perform rough positioning.
**************************************************************/
void moveitRoughPosition(Point3d point3d_input)
{
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // moveit plannning setup
    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // display info
    ROS_INFO("Planning frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
    
    // Object setup: make base(like table)
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(4);

    collision_objects[0].id = "table";
    collision_objects[0].header.frame_id = move_group.getPlanningFrame();
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 1.0;
    collision_objects[0].primitives[0].dimensions[1] = 1.0;
    collision_objects[0].primitives[0].dimensions[2] = 0.2;

    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.0;
    collision_objects[0].primitive_poses[0].position.y = 0.0;
    collision_objects[0].primitive_poses[0].position.z = -0.15;
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;
    planning_scene_interface.applyCollisionObjects(collision_objects);
    ROS_INFO("Add an object into the world\n");

    // move to rough target position
    geometry_msgs::PoseStamped target_pose;
    std::vector<geometry_msgs::Pose> waypoints;
    target_pose = move_group.getCurrentPose();
    waypoints.push_back(target_pose.pose);

    target_pose.pose.position.z -= 0.05; 
    waypoints.push_back(target_pose.pose);

    target_pose.pose.position.x += point3d_input.x;
    target_pose.pose.position.y += point3d_input.y; 
    waypoints.push_back(target_pose.pose);

    cout << "target_pose.pose: " << endl << target_pose.pose << endl;

    // move action
    move_group.setMaxVelocityScalingFactor(0.5);
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("Target Pose", "Target Pose (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

    if (fraction != 1.0)    // if nove action failed, lower the threshold
    {
        cout << "--retry--" << endl;
        const double eef_step = 0.02;
        fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        ROS_INFO_NAMED("Target Pose", "Target Pose (Cartesian path) (%.2f%% achieved)", fraction * 100.0);  
    }

    move_group.execute(trajectory);
}

/*************************************************************
 * After rough positioning, position the camera so that 
 * its center is directly above the object. 
 * This allows us to obtain more accurate information about the object.
**************************************************************/
void moveitPrecisionPosition(Point3d point3d_input)
{
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // moveit plannning setup
    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // display info
    ROS_INFO("Planning frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
    
    // Object setup: make base(like table)
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(4);

    collision_objects[0].id = "table";
    collision_objects[0].header.frame_id = move_group.getPlanningFrame();
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 1.0;
    collision_objects[0].primitives[0].dimensions[1] = 1.0;
    collision_objects[0].primitives[0].dimensions[2] = 0.2;

    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.0;
    collision_objects[0].primitive_poses[0].position.y = 0.0;
    collision_objects[0].primitive_poses[0].position.z = -0.15;
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;
    planning_scene_interface.applyCollisionObjects(collision_objects);
    ROS_INFO("Add an object into the world\n");

    // move to precision target position
    geometry_msgs::PoseStamped target_pose;
    std::vector<geometry_msgs::Pose> waypoints;
    target_pose = move_group.getCurrentPose();
    waypoints.push_back(target_pose.pose);

    target_pose.pose.position.z -= 0.05; 
    waypoints.push_back(target_pose.pose);

    target_pose.pose.position.x += (point3d_input.x);
    target_pose.pose.position.y += (point3d_input.y); 
    waypoints.push_back(target_pose.pose);

    cout << "target_pose.pose: " << endl << target_pose.pose << endl;

    // move action
    move_group.setMaxVelocityScalingFactor(0.5);
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO("Target Pose (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

    if (fraction != 1.0)    // if nove action failed, lower the threshold
    {
        cout << "--retry--" << endl;
        const double eef_step = 0.02;
        fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        ROS_INFO("Target Pose (Cartesian path) (%.2f%% achieved)", fraction * 100.0);  
    }

    move_group.execute(trajectory);
}

// /*************************************************************
//  * Move to a position where the gripper center and object center 
//  * are aligned and ready for disposal
// **************************************************************/
void moveitFinalPosition(double double_wrist)
{
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // moveit plannning setup
    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // display info
    ROS_INFO("Planning frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
    
    // Object setup: make base(like table)
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(4);

    collision_objects[0].id = "table";
    collision_objects[0].header.frame_id = move_group.getPlanningFrame();
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 1.0;
    collision_objects[0].primitives[0].dimensions[1] = 1.0;
    collision_objects[0].primitives[0].dimensions[2] = 0.2;

    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.0;
    collision_objects[0].primitive_poses[0].position.y = 0.0;
    collision_objects[0].primitive_poses[0].position.z = -0.15;
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;
    planning_scene_interface.applyCollisionObjects(collision_objects);
    ROS_INFO("Add an object into the world");

    // First, move UR3 so that the center of the gripper and the center of the object are aligned
    geometry_msgs::PoseStamped target_pose;
    std::vector<geometry_msgs::Pose> waypoints;
    target_pose = move_group.getCurrentPose();
    waypoints.push_back(target_pose.pose);

    target_pose.pose.position.x += ( -0.035);
    target_pose.pose.position.y += (  0.048); 
    waypoints.push_back(target_pose.pose);

    // move action
    move_group.setMaxVelocityScalingFactor(0.5);
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("Target Pose", "Target Pose (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

    if (fraction != 1.0)
    {
        cout << "--retry--" << endl;
        const double eef_step = 0.02;
        fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        ROS_INFO_NAMED("Target Pose", "Target Pose (Cartesian path) (%.2f%% achieved)", fraction * 100.0);  
    }

    move_group.execute(trajectory);

    // Rotate the wrist to match the posture of the object
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    
    joint_group_positions[5] += double_wrist;
    move_group.setJointValueTarget(joint_group_positions);

    move_group.setMaxVelocityScalingFactor(0.5);
    move_group.setMaxAccelerationScalingFactor(0.5);
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
    move_group.move();
    ros::Duration(5.0).sleep();

    //--------------------------------------------------------------- z
    // // birdsEye -> Home
    // geometry_msgs::PoseStamped target_pose2;
    // std::vector<geometry_msgs::Pose> waypoints2;
    // target_pose2 = move_group.getCurrentPose();
    // waypoints2.push_back(target_pose2.pose);

    // target_pose2.pose.position.z += -0.2;
    // waypoints2.push_back(target_pose2.pose);

    // // move
    // move_group.setMaxVelocityScalingFactor(0.1);
    // moveit_msgs::RobotTrajectory trajectory2;
    // const double jump_threshold2 = 0.0;
    // const double eef_step2 = 0.01;
    // double fraction2 = move_group.computeCartesianPath(waypoints2, eef_step2, jump_threshold2, trajectory2);
    // ROS_INFO_NAMED("Target Pose", "Target Pose (Cartesian path) (%.2f%% achieved)", fraction2 * 100.0);
    // move_group.execute(trajectory2);
    // ros::Duration(5.0).sleep();
}

// /*************************************************************
//  * Move to a position where the gripper center and object center 
//  * are aligned and ready for disposal
// **************************************************************/
void drawContours(vector<vector<Point>> contours, Mat img_mask, string mode)
{
    // declare for LineIterator
    Point li_start;
    Point li_goal;
    
    for (int i = 0; i < contours.size(); i++)
    {
        for (int j = 0; j < contours[i].size(); j++)
        {
            // specify start and end point for cv::LineIterator
            li_start = contours[i][j];

            if (j == contours[i].size() - 1)
                li_goal = contours[i][0];
            else    
                li_goal = contours[i][j + 1];

            // using cv::LineIterator
            LineIterator LI(img_mask, li_start, li_goal, 8, false);
        
            // get point on the line
            vector<Point> li_point(LI.count);
            for (int l = 0; l < LI.count; l++, ++LI)
                li_point[l] = LI.pos();

            // draw
            for (int k = 0; k < li_point.size(); k++)
            {
                // based on the object's centroid, change the direction to make the line thicker
                if (mode == "obj")
                    img_black.at<Vec3b>(li_point[k].y, li_point[k].x ) = Vec3b(0, 0, 255);
                else if (mode == "hand")
                    img_black.at<Vec3b>(li_point[k].y, li_point[k].x )[0] += 255; //overwrite on Hand's line


                if ((img_black.at<Vec3b>(li_point[k].y, li_point[k].x )) == Vec3b(255, 0, 255))
                {
                    contours_hand_contact.push_back(Point(li_point[k].x, li_point[k].y));
                }
            }
        }
    }
}

/*************************************************************
 * get orientation
**************************************************************/
void getOrientation(const std::vector<cv::Point> &pts, Mat img_output)
{
    cv::Mat data_pts = cv::Mat(pts.size(), 2, CV_64F); // [pts.size() x 2] 行列
    for (int i = 0; i < data_pts.rows; i++) 
    {
        data_pts.at<double>(i, 0) = pts[i].x;
        data_pts.at<double>(i, 1) = pts[i].y;
    }

    // do analysis
    cv::PCA pca_analysis(data_pts, cv::Mat(), cv::PCA::DATA_AS_ROW);

    // calc center
    cv::Point cntr = cv::Point(pca_analysis.mean.at<double>(0, 0),
                               pca_analysis.mean.at<double>(0, 1));

    // eigen 
    std::vector<cv::Point2d> eigen_vecs(2);
    std::vector<double> eigen_val(2);
    for (int i = 0; i < 2; i++) 
    {
        eigen_vecs[i] = cv::Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
                                    pca_analysis.eigenvectors.at<double>(i, 1));
        eigen_val[i] = pca_analysis.eigenvalues.at<double>(i);
    }

    // display
    cv::circle(img_output, cntr, 3, cv::Scalar(255, 0, 255), 2);
    cv::Point p1 = cntr + 0.02 * cv::Point(eigen_vecs[0].x * eigen_val[0], eigen_vecs[0].y * eigen_val[0]);
    cv::Point p2 = cntr - 0.02 * cv::Point(eigen_vecs[1].x * eigen_val[1], eigen_vecs[1].y * eigen_val[1]);
    line(img_output, cntr, p1, Scalar(255, 0, 0), 3);
    line(img_output, cntr, p2, Scalar(0, 255, 0), 3);

    // angle
    double angle = atan2(eigen_vecs[1].y, eigen_vecs[1].x); // pca second principal component
    wrist_rotation = angle;

    // ------------ grasp width
    Mat img_black_gripper = Mat::zeros(img_src_yolact.size(), img_src_yolact.type());
    
    int denominator = (p2.x - cntr.x);
    if (denominator <= 0)   denominator = 1;
    
    Point point1(0, (p2.y - cntr.y) * (0 - cntr.x) / denominator + cntr.y);
    Point point2(img_output.cols, (p2.y - cntr.y) * (img_output.cols - cntr.x) / denominator + cntr.y);
    line(img_black_gripper, point1, point2, Scalar(0, 255, 0), 1);
    line(img_output, point1, point2, Scalar(255, 255, 255), 1);

    vector<int> th_mask_gripper{10, 0, 0, 179, 255, 255};
    Mat img_mask_gripper = createMaskImg(img_black_gripper, th_mask_gripper);
    vector<vector<Point>> contour2;
    vector<Vec4i> hierarchy2;
    cv::findContours(img_mask_gripper, contour2, hierarchy2, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    contours_hand_contact.clear();

    // detect line overlap
    drawContours(contours_obj, img_mask, "obj");
    drawContours(contour2, img_mask_gripper, "hand");

    // display
    for(int i = 0; i < contours_hand_contact.size(); i++)
    {
        circle(img_output, contours_hand_contact[i],  10, Scalar(255, 255, 255), 8, 8);
        circle(img_output, contours_hand_contact[i],   8, Scalar(  0, 255,   0), 8, 8);
        circle(img_output, contours_hand_contact[i],   2, Scalar(255, 255, 255), 8, 8);
    }
    double width = sqrt(pow((contours_hand_contact[0].x - contours_hand_contact[1].x), 2) + pow((contours_hand_contact[0].y - contours_hand_contact[1].y), 2));
    gripper_width = width;

    // display result
    imshow("img_black", img_black);
    imshow("img_output", img_output);
}

/*************************************************************
 * pca for calculate rotation of wrist
**************************************************************/
void pca(vector<int> th_hsv)
{
    // get mask img
    Mat img_mask_pca = createMaskImg(img_src_yolact, th_hsv);
    vector<vector<Point>> contours_pca;
    contours_obj = extractObjectContours(img_mask_pca);
    
    // initialize
    img_output = img_src_yolact.clone();
    getOrientation(contours_obj[0], img_output);
    
}

/*************************************************************
 * Receives commands and controls the gripper
**************************************************************/
ros::Publisher gripperCommand_pub;
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

// /*************************************************************
//  * move to disposal position
// **************************************************************/
void moveitDisposalPosition()
{
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // info
    ROS_INFO("Planning frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
    
    // ------------------- Moveit: Object
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(4);

    collision_objects[0].id = "table";
    collision_objects[0].header.frame_id = move_group.getPlanningFrame();
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 1.0;
    collision_objects[0].primitives[0].dimensions[1] = 1.0;
    collision_objects[0].primitives[0].dimensions[2] = 0.2;

    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.0;
    collision_objects[0].primitive_poses[0].position.y = 0.0;
    collision_objects[0].primitive_poses[0].position.z = -0.15;
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;
    planning_scene_interface.applyCollisionObjects(collision_objects);
    ROS_INFO("Add an object into the world");

    // move
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    
    joint_group_positions[1] = -1.57;  
    joint_group_positions[2] = -1.57;  
    joint_group_positions[3] = -1.57;  

    move_group.setJointValueTarget(joint_group_positions);

    move_group.setMaxVelocityScalingFactor(0.5);
    move_group.setMaxAccelerationScalingFactor(0.5);
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan (joint space goal) %s", success ? "" : "FAILED");

    move_group.move();

    // -------move to container
    current_state = move_group.getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    
    joint_group_positions[0] = -3.14; 
    move_group.setJointValueTarget(joint_group_positions);

    move_group.setMaxVelocityScalingFactor(0.5);
    move_group.setMaxAccelerationScalingFactor(0.5);
    success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

    move_group.move();

    //  // ---------------------------------------z
    // geometry_msgs::PoseStamped target_pose;
    // std::vector<geometry_msgs::Pose> waypoints;
    // target_pose = move_group.getCurrentPose();
    // waypoints.push_back(target_pose.pose);

    // target_pose.pose.position.z += (-0.2) ; //- 0.035
    // waypoints.push_back(target_pose.pose);

    // // move
    // move_group.setMaxVelocityScalingFactor(0.1);
    // moveit_msgs::RobotTrajectory trajectory;
    // const double jump_threshold = 0.0;
    // const double eef_step = 0.01;
    // double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    // ROS_INFO_NAMED("Target Pose", "Target Pose (Cartesian path) (%.2f%% achieved)", fraction * 100.0);
    // move_group.execute(trajectory);
}

//-----------------------------------------------------
// MAIN
//-----------------------------------------------------
int main(int argc, char **argv)
{
    // node declare
    ros::init(argc, argv, "ur3_disposal");
    ros::NodeHandle nh("");
    image_transport::ImageTransport it(nh);

    // subscriber
    image_transport::Subscriber image_sub = it.subscribe("/yolact_ros/visualization", 1, imageCb);
    image_transport::Subscriber image_sub_depth = it.subscribe("/camera/aligned_depth_to_color/image_raw", 1, imageCb_depth);
    gripperCommand_pub = nh.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("Robotiq2FGripperRobotOutput", 10);


    // loop process
	ros::Rate loop_rate(1000);
    while(ros::ok())
    {
        int key = waitKey(1);
        if (key == 'q')
            break;
    
        cout << "--------------flag_state: " << flag_state << endl;
        if (!img_src_yolact.empty())
        {
            // image setup
            img_mask = img_src_yolact.clone();
            img_output = img_src_yolact.clone();
            img_black = Mat::zeros(img_src_yolact.size(), img_src_yolact.type());

            switch (flag_state)
            {
                case 0: // gripper activation
                    gripperControler(300);
                    ros::Duration(1.0).sleep();
                    gripperControler(400);    
                    ros::Duration(1.0).sleep();
                    flag_state = 1;
                    break;

                case 1: // create mask img and get contours
                    img_mask = createMaskImg(img_src_yolact, th_obj_hsv);
                    contours_obj = extractObjectContours(img_mask);
                    flag_state = 2;
                    break;
                
                case 2: // calc object centroids
                    point2d_obj_centroid = calcCentroid(contours_obj);
                    flag_state = 3;
                    break;
                
                case 3: // for each centroids
                    cout << "point2d_obj_centroid.size(): " << point2d_obj_centroid.size() << endl;
                    for(int i = 0; i < point2d_obj_centroid.size(); i++)
                    {
                        cout << "--------------flag_state_one: " << flag_state_one << endl;
                        switch (flag_state_one)
                        {
                            case 0: // transform pixel(2D) to point(3D)
                                cout << "i:" << i << endl;
                                if(flag_deprojection == 1)
                                {
                                    point3d_obj_centroid = deprojectPixelToPoint(point2d_obj_centroid[i], i);
                                    flag_state_one = 1;
                                }
                                break;
                            
                //             case 1: // transform point3d to base coordinate
                //                 point3d_base = transformCoordinateToBase(point3d_obj_centroid);
                //                 flag_state_one = 2;
                //                 break;
                            
                //             case 2:
                //                 moveitInitialPose();
                //                 flag_state_one = 3;
                //                 break;
                            
                //             case 3:
                //                 point3d_wrist = transformCoordinateToWrist(point3d_base);
                //                 flag_state_one = 4;
                //                 break;
                            
                //             case 4:
                //                 moveitRoughPosition(point3d_wrist);
                //                 flag_state_one = 5;
                //                 break;
                            
                //             case 5:
                //                 img_mask = createMaskImg(img_src_yolact, th_obj_hsv);
                //                 contours_obj = extractObjectContours(img_mask);
                //                 if (!contours_obj.empty())
                //                 {
                //                     point3d_obj_centroid = deprojectPixelToPoint(point2d_obj_centroid[0], i);
                //                     if (flag_deprojection == 1)
                //                     {
                //                         moveitPrecisionPosition(point3d_obj_centroid);
                //                         flag_state_one = 6;
                //                         flag_deprojection = 0;
                //                     }
                //                 }
                //                 break;

                //             case 6:
                //                 pca(th_obj_hsv);
                //                 moveitFinalPosition(wrist_rotation);
                //                 flag_state_one = 7;
                //                 break;
                            
                //             case 7:
                //                 gripperControler(134);
                //                 flag_state_one = 8;
                //                 break;
                            
                //             case 8:
                //                 moveitDisposalPosition();
                //                 flag_state_one = 9;
                //                 break;
                            
                //             case 9:
                //                  gripperControler(0);
                //                 flag_state_one = 10;
                //                 break;
                        
                            default:
                                flag_state_one = 0;
                                break;
                        }
                    }
                    flag_state = 4;
                    break;
                
                default:
                    flag_if = 1;
                    break;
            }

            if (flag_if == 1)
                return -1;
        }

        ros::spinOnce();
		loop_rate.sleep();
    }

    return 0;
}