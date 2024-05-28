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

//-----------------------------------------------------
// DEFINES
//-----------------------------------------------------
using namespace cv;
using namespace std;

// img proc
Mat img_src;
Mat img_mask;
Mat img_black;
Mat img_output;

// point info
vector<vector<Point>> contour_obj;
vector<Point> point_obj_2d;
vector<Point3d> point_obj_3d;
vector<Point> contours_hand_contact;
vector<Point3d> point3d_base;
vector<Point3d> point3d_wrist;

// depth info
vector<double> depth;

// disposal
vector<double> wrist_rotation;
vector<double> gripper_width;

// flag
int flag_state = 0;
int flag_deprojection = 0;
int flag_if = 0;
//-----------------------------------------------------
// FUNCTIONS
//-----------------------------------------------------
/*************************************************************
 * @Function
 *    image callback
**************************************************************/
void imageCb(const sensor_msgs::ImageConstPtr& msg)
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
}
/*************************************************************
 * @Function 
 *    image callback depth
**************************************************************/
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

    // calclate distance
    depth.clear();
    for (int i = 0; i < point_obj_2d.size(); i++)
    {
        double calc =  0.001*cv_ptr->image.at<u_int16_t>(point_obj_2d[i].y, point_obj_2d[i].x);
        depth.push_back(calc);
    }
}
/*************************************************************
 * @Function
 *    image callback
**************************************************************/
vector<vector<Point>> extract_region()
{
    // extract object from YOLACT result using RGB info
    Scalar lower(  0,   0,   0);
    Scalar upper(  0, 255, 255);
    cv::inRange(img_mask, lower, upper, img_mask);
    bitwise_not(img_mask, img_mask);
    imshow("img_mask", img_mask);   // debug

    // extract contours from img_mask
    vector<vector<Point>> contour_output;
    vector<Vec4i> hierarchy;
    cv::findContours(img_mask, contour_output, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    cv::drawContours(img_output, contour_output, -1, (0, 0, 255), 3);  // debug
    imshow("img_output", img_output);   // debug

    return contour_output;
}

/*************************************************************
 * @Function
 *    detect centroid
**************************************************************/
vector<Point> detect_centroid(vector<vector<Point>> contour_input)
{
    // prepare mu and mc for centroid detection
    vector<Moments> mu_obj(contour_input.size());
    for (int i = 0; i < contour_input.size(); i++)
        mu_obj[i] = cv::moments(contour_input[i], false );
    vector<Point2f> mc_obj(contour_input.size());

    // calculation centroid
    vector<Point> point_output(contour_input.size());
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

    cout << "point_output: " << endl << point_output << endl;
    imshow("img_mask", img_mask);   // debug
    imshow("img_output", img_output);   // debug
    imwrite("/home/umelab/UR_test/src/wrs_fcsc_2023/img/img_output_centroid.png", img_output);   // debug

    return point_output;
}

/*************************************************************
 * @Function
 *    detect centroid
**************************************************************/
vector<Point3d> deprojectPixelToPoint(vector<Point> point_pixel)
{
    vector<Point3d> point_output;
    if (!depth.empty())
    {
        for (int i = 0; i < point_pixel.size(); i++)
        {
            // calculation (using param from /camera/depth/camera_info)
            double x = 0.;
            double y = 0.;
            x = (point_pixel[i].x - 321.7812805175781) / 380.3281555175781;
            y = (point_pixel[i].y - 238.0182342529297) / 380.3281555175781;

            // in the original code, they use BROWN_CONRADY, but we have to use plum_bob
            // however, the distortion coefficients is same, we write the same code
            double r2 = x * x + y * y;
            double f = 1 + 0*r2 + 0*r2*r2 + 0*r2*r2*r2;
            double ux = x*f + 2*0*x*y + 0*(r2 + 2*x*x);
            double uy = y*f + 2*0*x*y + 0*(r2 + 2*y*y);

            // output
            Point3d calc(depth[i] * ux,(-1)* depth[i] * uy, depth[i]);
            point_output.push_back(calc);
        }

        // update flag
        cout << "deproject pixel to point: " << endl << point_output << endl;
        flag_deprojection = 1;

    }

    return point_output;
}

/*************************************************************
 * @Function
 *    wrist_3_link to base_link
**************************************************************/
void transformCoordinate(vector<Point3d> point3d_input)
{
    // tf
    tf::TransformListener listener;

    point3d_base.clear();
    for (int i = 0; i < point3d_input.size(); i++)
    {
        // declare target frame
        std::string targetFrame = "base_link";

        // define object point
        geometry_msgs::PointStamped objectPoint;
        objectPoint.header.frame_id = "wrist_3_link";
        objectPoint.point.x = point3d_input[i].x + 0.035; // 
        objectPoint.point.y = point3d_input[i].y - 0.055; //
        objectPoint.point.z = point3d_input[i].z;

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
        ///////////////////////////////////////////////////////////
        //point3d_base.push_back(Point3d(x - 0.035, y + 0.055, z));
        point3d_base.push_back(Point3d(x , y , z));
        ///////////////////////////////////////////////////////////
    }

    // debug
    cout << "point3d_base :" << point3d_base << endl;     

    // update flag
}
/*************************************************************
 * @Function
 *    wrist_3_link to base_link
**************************************************************/
void disposalInitPose()
{
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // info
    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
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

    /* Define the pose of the table. */
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.0;
    collision_objects[0].primitive_poses[0].position.y = 0.0;
    collision_objects[0].primitive_poses[0].position.z = -0.15;
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;
    planning_scene_interface.applyCollisionObjects(collision_objects);
    ROS_INFO_NAMED("tutorial", "Add an object into the world");
    ros::Duration(2.0).sleep();

    // birdsEye -> Home
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    
    // joint_group_positions[1] = -1.68;  // -1/6 turn in radians
    // joint_group_positions[2] = -0.959931;  // -1/6 turn in radians
    // joint_group_positions[3] = -2.05949;  // -1/6 turn in radians
    joint_group_positions[1] = -1.27409;  // -1/6 turn in radians
    joint_group_positions[2] = -1.27409;  // -1/6 turn in radians
    joint_group_positions[3] = -2.18166;  // -1/6 turn in radians


    move_group.setJointValueTarget(joint_group_positions);

    move_group.setMaxVelocityScalingFactor(0.4);
    move_group.setMaxAccelerationScalingFactor(0.4);
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

    move_group.move();
    ros::Duration(1.0).sleep();
}

/*************************************************************
 * @Function
 *    base_link to wrist_3_link
**************************************************************/
void transformCoordinateInverse(vector<Point3d> point3d_input)
{
    // tf
    tf::TransformListener listener;

    // Specify the source coordinate frame and the target coordinate frame
    std::string sourceFrame = "base_link";
    std::string targetFrame = "wrist_3_link";

    point3d_wrist.clear();
    for (int i = 0; i < point3d_input.size(); i++)
    {
        // object position in wrist_3_link
        /////////////////////////////////////////////
        //double x = point3d_input[i].x + 0.035;
        //double y = point3d_input[i].y - 0.055;
        /////////////////////////////////////////////
        double x = point3d_input[i].x ;
        double y = point3d_input[i].y ;
        double z = point3d_input[i].z ;

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
            return;
        }

        // output
        double transformedX = transformedPose.getOrigin().x();
        double transformedY = transformedPose.getOrigin().y();
        double transformedZ = transformedPose.getOrigin().z();
        

        //ohashi change
        ////////////////////////////////////////////////////
        // double x_input = transformedX - 0.035;
        // double y_input = abs(transformedY + 0.055);
        // if (y_input >= 0.3)
        //     y_input = 0.3;
        double x_input = transformedX ;
        double y_input = transformedY ;
        // ////////////////////////////////////////////////////
        point3d_wrist.push_back(Point3d(x_input, y_input , transformedZ));    // be careful
    }
    // debug
    cout << "point3d_wrist :" << point3d_wrist << endl;   
}
/*************************************************************
 * @Function
 *    pca
**************************************************************/

void disposalTargetRobustPosition(vector<Point3d> point3d_input)
{
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // info
    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
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

    /* Define the pose of the table. */
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.0;
    collision_objects[0].primitive_poses[0].position.y = 0.0;
    collision_objects[0].primitive_poses[0].position.z = -0.15;
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;
    planning_scene_interface.applyCollisionObjects(collision_objects);
    ROS_INFO_NAMED("tutorial", "Add an object into the world");

    // birdsEye -> Home
    geometry_msgs::PoseStamped target_pose;
    std::vector<geometry_msgs::Pose> waypoints;
    target_pose = move_group.getCurrentPose();
    waypoints.push_back(target_pose.pose);

    target_pose.pose.position.z -= 0.05; 
    waypoints.push_back(target_pose.pose);

    target_pose.pose.position.x += point3d_input[0].x;
    target_pose.pose.position.y += point3d_input[0].y; 
    cout << "target_pose.pose: " << endl << target_pose.pose << endl;
    waypoints.push_back(target_pose.pose);

    // move
    move_group.setMaxVelocityScalingFactor(0.4);
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
}

/*************************************************************
 * @Function
 *    pca
**************************************************************/
void disposalTargetPrecisionImagePosition(vector<Point3d> point3d_input)
{
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // info
    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
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

    /* Define the pose of the table. */
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.0;
    collision_objects[0].primitive_poses[0].position.y = 0.0;
    collision_objects[0].primitive_poses[0].position.z = -0.15;
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;
    planning_scene_interface.applyCollisionObjects(collision_objects);
    ROS_INFO_NAMED("tutorial", "Add an object into the world");

    //--------------------------------------------------------------- position
    // birdsEye -> Home
    geometry_msgs::PoseStamped target_pose;
    std::vector<geometry_msgs::Pose> waypoints;
    target_pose = move_group.getCurrentPose();
    waypoints.push_back(target_pose.pose);

    // input is camera zero point, so we transform into gripper zero point
    target_pose.pose.position.z -= 0.10; 
    waypoints.push_back(target_pose.pose);

    target_pose.pose.position.x += (point3d_input[0].x);
    target_pose.pose.position.y += (point3d_input[0].y); 
    waypoints.push_back(target_pose.pose);

    // move
    move_group.setMaxVelocityScalingFactor(0.4);
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

}

/*************************************************************
 * @Function
 *    pca
**************************************************************/

void disposalTargetPrecisionPosition(vector<Point3d> point3d_input, vector<double> point3d_input_wrist)
{
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // info
    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
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

    /* Define the pose of the table. */
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.0;
    collision_objects[0].primitive_poses[0].position.y = 0.0;
    collision_objects[0].primitive_poses[0].position.z = -0.15;
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;
    planning_scene_interface.applyCollisionObjects(collision_objects);
    ROS_INFO_NAMED("tutorial", "Add an object into the world");
    ros::Duration(2.0).sleep();

    //--------------------------------------------------------------- position
    // birdsEye -> Home
    geometry_msgs::PoseStamped target_pose;
    std::vector<geometry_msgs::Pose> waypoints;
    target_pose = move_group.getCurrentPose();
    waypoints.push_back(target_pose.pose);

    // input is camera zero point, so we transform into gripper zero point
    target_pose.pose.position.x += ( -0.035);
    target_pose.pose.position.y += ( + 0.048); 
    waypoints.push_back(target_pose.pose);

    // move
    move_group.setMaxVelocityScalingFactor(0.4);
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

    //--------------------------------------------------------------- wrist 
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    
    joint_group_positions[5] += point3d_input_wrist[0];
    move_group.setJointValueTarget(joint_group_positions);

    move_group.setMaxVelocityScalingFactor(0.05);
    move_group.setMaxAccelerationScalingFactor(0.05);
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

    // target_pose2.pose.position.z += -0.05;
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


/*************************************************************
 * @Function
 *    pca
**************************************************************/
Mat createMaskImg(Mat img, int H_MIN, int H_MAX, int S_MIN, int S_MAX, int V_MIN, int V_MAX)
{
    Mat img_mask = Mat::zeros(img_src.size(), img_src.type());

    // convert to HSV image
    Mat img_hsv;
    cvtColor(img, img_hsv, COLOR_BGR2HSV);

    // extract region based on HSV parameter
    Scalar Lower(H_MIN, S_MIN, V_MIN);
    Scalar Upper(H_MAX, S_MAX, V_MAX);
    inRange(img_hsv, Lower, Upper, img_mask);

    return img_mask;
}

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

    // eigen 
    std::vector<cv::Point2d> eigen_vecs(2);
    std::vector<double> eigen_val(2);
    for (int i = 0; i < 2; i++) {
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
    // ROS_INFO("angle: %lf", angle);
    wrist_rotation.push_back(angle);

    //-------------------------------------------------------------------------------------
    Mat img_black2 = Mat::zeros(img_src.size(), img_src.type());
    int denominator = (p2.x - cntr.x);
    if (denominator <= 0)
    {
        denominator = 1;
    }
        
    Point point1(0, (p2.y - cntr.y) * (0 - cntr.x) / denominator + cntr.y);
    Point point2(img_output.cols, (p2.y - cntr.y) * (img_output.cols - cntr.x) / denominator + cntr.y);
    line(img_black2, point1, point2, Scalar(0, 255, 0), 1);
    line(img_output, point1, point2, Scalar(255, 255, 255), 1);

    Mat img_mask2 = createMaskImg(img_black2, 10, 179, 0, 255, 0, 255);
    vector<vector<Point>> contour2;
    vector<Vec4i> hierarchy2;
    cv::findContours(img_mask2, contour2, hierarchy2, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    contours_hand_contact.clear();
    drawContours(contour_obj, img_mask, "obj");
    drawContours(contour2, img_mask2, "hand");
    for(int i = 0; i < contours_hand_contact.size(); i++)
    {
        circle(img_output, contours_hand_contact[i],  10, Scalar(255, 255, 255), 8, 8);
        circle(img_output, contours_hand_contact[i],   8, Scalar(  0, 255,   0), 8, 8);
        circle(img_output, contours_hand_contact[i],   2, Scalar(255, 255, 255), 8, 8);
    }
    double width = sqrt(pow((contours_hand_contact[0].x - contours_hand_contact[1].x), 2) + pow((contours_hand_contact[0].y - contours_hand_contact[1].y), 2));
    gripper_width.push_back(width);

    imshow("img_black", img_black);
    imshow("img_output", img_output);
    imwrite("/home/umelab/UR_test/src/wrs_fcsc_2023/img/img_black.png", img_black);
    imwrite("/home/umelab/UR_test/src/wrs_fcsc_2023/img/img_output.png", img_output);
}

void pca()
{
    contour_obj = extract_region();
    imwrite("/home/umelab/UR_test/src/wrs_fcsc_2023/img/img_pca.png", img_output);

    img_output = img_src.clone();
    wrist_rotation.clear();
    gripper_width.clear();

    for (int i = 0; i < contour_obj.size(); i++)
        getOrientation(contour_obj[i], img_output);
    
    for (int i = 0; i < contour_obj.size(); i++)
    {
        cout << "wrist_rotation[" << i << "]: " << endl << wrist_rotation[i] << endl;
        cout << "gripper_width[" << i << "]: " << gripper_width[i] << endl;
    }
    flag_state = 10;
        
}
/*************************************************************
 * @Function
 *    pca
**************************************************************/
ros::Publisher gripperCommand_pub;
void gripperCommand(int fig)
{
    robotiq_2f_gripper_control::Robotiq2FGripper_robot_output co;

    if ( 0 <= fig && fig <= 255)
    {
        co.rACT = 1;
        co.rGTO = 1;
        co.rATR = 0;
        co.rPR = fig;
        co.rSP = 255;
        co.rFR = 150;
        ROS_INFO("position : done");
    }
    if ( fig == 300)
    {
        co.rACT = 0;
        ROS_INFO("reset: done");
    }
    else if ( fig == 400)
    {
        co.rACT = 1;
        co.rGTO = 1;
        co.rATR = 0;
        co.rPR = 0;
        co.rSP = 255;
        co.rFR = 150;
        ROS_INFO("activate: done");
    }
    else if ( fig == 500 )
    {
        co.rACT = 1;
        co.rGTO = 1;
        co.rATR = 0;
        co.rPR = 255;
        co.rSP = 255;
        co.rFR = 150;
        ROS_INFO("close: done");
    }
    else if ( fig == 600)
    {
        co.rPR = 0;
        ROS_INFO("open: done");
    }

    gripperCommand_pub.publish(co);
}

/*************************************************************
 * @Function
 *    p
**************************************************************/
void disposal()
{
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // info
    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
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

    /* Define the pose of the table. */
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.0;
    collision_objects[0].primitive_poses[0].position.y = 0.0;
    collision_objects[0].primitive_poses[0].position.z = -0.15;
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;
    planning_scene_interface.applyCollisionObjects(collision_objects);
    ROS_INFO_NAMED("tutorial", "Add an object into the world");
    ros::Duration(2.0).sleep();

    // move
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    
    joint_group_positions[1] = -1.57;  // -1/6 turn in radians
    joint_group_positions[2] = -1.57;  // -1/6 turn in radians
    joint_group_positions[3] = -1.57;  // -1/6 turn in radians

    move_group.setJointValueTarget(joint_group_positions);

    move_group.setMaxVelocityScalingFactor(0.05);
    move_group.setMaxAccelerationScalingFactor(0.05);
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

    move_group.move();
    ros::Duration(5.0).sleep();

    // ---------------------------------------move to container
    current_state = move_group.getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    
    joint_group_positions[0] = -3.14;  // -1/6 turn in radians
    move_group.setJointValueTarget(joint_group_positions);

    move_group.setMaxVelocityScalingFactor(0.05);
    move_group.setMaxAccelerationScalingFactor(0.05);
    success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

    move_group.move();
    ros::Duration(5.0).sleep();

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
        if (!img_src.empty())
        {
            img_mask = img_src.clone();
            img_output = img_src.clone();
            img_black = Mat::zeros(img_src.size(), img_src.type());

            
            switch (flag_state)
            {
                case 0:
                    gripperCommand(300);
                    ros::Duration(2.0).sleep();
                    gripperCommand(400);    
                    ros::Duration(5.0).sleep();
                    flag_state = 1;

                case 1:
                    contour_obj = extract_region();
                    flag_state = 2;
                    break;

                case 2:
                    point_obj_2d = detect_centroid(contour_obj);
                    flag_state = 3;
                    break;
                
                case 3:
                    point_obj_3d = deprojectPixelToPoint(point_obj_2d);
                    if (flag_deprojection == 1)
                        flag_state = 4;
                    flag_deprojection = 0;
                    break;
                
                case 4:
                    transformCoordinate(point_obj_3d);
                    imwrite("/home/umelab/UR_test/src/wrs_fcsc_2023/img/img_output_init.png", img_output);  

                    flag_state = 5;
                    break;
                
                case 5:
                    disposalInitPose();
                    flag_state = 6;
                    break;

                case 6:
                    transformCoordinateInverse(point3d_base);
                    flag_state = 7;
                    break;

                case 7:
                    disposalTargetRobustPosition(point3d_wrist);
                    flag_state = 8;
                    break;
                
                // case 8:
                //     contour_obj = extract_region();
                //     point_obj_2d = detect_centroid(contour_obj);
                //     if (!point_obj_2d.empty())
                //     {
                //         point_obj_3d = deprojectPixelToPoint(point_obj_2d);
                //         if (flag_deprojection == 1)
                //         {
                //             imwrite("/home/umelab/UR_test/src/wrs_fcsc_2023/img/img_output_precision.png", img_output);  
                //             disposalTargetPrecisionImagePosition(point_obj_3d);
                //             flag_state = 9;
                //             flag_deprojection = 0;
                //         }
                //     }
                //     break;
                
                // case 9:
                //     imwrite("/home/umelab/UR_test/src/wrs_fcsc_2023/img/img_output_precision2.png", img_output);  
                //     pca();
                //     disposalTargetPrecisionPosition(point_obj_3d, wrist_rotation);
                //     flag_state = 10;
                //     break;

                // case 9:
                //     ros::Duration(2.0).sleep();
                //     gripperCommand(134);
                //     flag_state = 10;
                //     break;
                
                // case 10:
                //     disposal();
                //     flag_state = 11;
                //     break;

                // case 11:
                //     gripperCommand(0);
                //     ros::Duration(2.0).sleep();

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