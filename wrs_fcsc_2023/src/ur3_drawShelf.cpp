// publish pixel_x, pixel_y, depth using realsense d435i
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

// Include tf2 for transformation
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// moveit
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

// pointcloud
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

// standard libraries
#include <iostream>

// calc cos
#include<math.h>
//-----------------------------------------------------
// DEFINES
//-----------------------------------------------------
using namespace std;
using namespace cv;

Mat img_src;
Mat img_output;
Point point_target_pixel;
Point3d point_target_3d;
Point3d point_target_3d_new;
double depth = 0.;

int flag_state = 1;

/*************************************************************
 * @Function
 *    detect ArUco marker
**************************************************************/
Point detectAruco(Mat img_src)
{
    // output point
    Point point_aruco;

    if (!img_src.empty())
    {
        // setup
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
        cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
	    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    
        // detect aruco markers
        cv::aruco::detectMarkers(img_src, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

        // draw detected aruco markers
        img_output = img_src.clone();
        cv::aruco::drawDetectedMarkers(img_output, markerCorners, markerIds);

        // calculate aruco centroid position
        if(markerIds.size() > 0 )
        {
            int j = 0;
            for (int i = 0; i < markerIds.size(); i++)
            {
                if ( markerIds[i] == 1)
                {
                    j = i;
                    break;
                }
            }

            point_aruco.x = (int)(markerCorners[j][0].x + markerCorners[j][1].x + markerCorners[j][2].x + markerCorners[j][3].x ) / 4;
            point_aruco.y = (int)(markerCorners[j][0].y + markerCorners[j][1].y + markerCorners[j][2].y + markerCorners[j][3].y ) / 4;
            circle(img_output, point_aruco, 2, Scalar(0, 0, 255), 8, 8);

            // end this process
            if(markerIds[j] == 1)
            {
                cout << "markerIds[j]: " << markerIds[j] << endl;
                cout << "point_aruco: " << point_aruco << endl;
                flag_state = 2;
            }
        }
    
        cv::imshow("out", img_output);
        waitKey(1);
    }
    else
        ROS_WARN("img_src is empty.");

    return point_aruco;
}

/*************************************************************
 * @Function
 *    transform pixel to 3d
**************************************************************/
Point3d deprojectPixelToPoint(Point point_pixel)
{
    // output point3d
    Point3d point_output;

    // calculation (using param from /camera/depth/camera_info)
    double x = 0.;
    double y = 0.;
    x = (point_pixel.x - 321.7812805175781) / 380.3281555175781;
    y = (point_pixel.y - 238.0182342529297) / 380.3281555175781;

    // in the original code, they use BROWN_CONRADY, but we have to use plum_bob
    // however, the distortion coefficients is same, we write the same code
    double r2 = x * x + y * y;
    double f = 1 + 0*r2 + 0*r2*r2 + 0*r2*r2*r2;
    double ux = x*f + 2*0*x*y + 0*(r2 + 2*x*x);
    double uy = y*f + 2*0*x*y + 0*(r2 + 2*y*y);

    // output
    point_output.x = depth * ux;
    point_output.y = depth * uy;
    point_output.z = depth;

    // update flag
    flag_state = 3;

    return point_output;
}

/*************************************************************
 * @Function
 *    transform pixel to 3d
**************************************************************/
Point3d transformTargetPoint(Point3d point_intput)
{
    //output point
    Point3d point_output;

    // calc new y
    // int deg = 45;
    // double rad = deg * 3.1415 / 180;
    // double y_new =  depth * cos(rad);
    // double z = 0.18;

    // point_output.x = point_intput.x;
    // point_output.y = y_new - 0.15 - 0.043; // gripper = 43mm, 
    // point_output.z = z;

    point_output.x = point_intput.x;
    point_output.y = depth - 0.45;  
    point_output.z = point_intput.z;
    cout << "point_output: " << point_output << endl;
    flag_state = 4;
    return point_output;

}

/*************************************************************
 * @Function
 *    transform pixel to 3d
**************************************************************/
void controlArm(Point3d point_input)
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
    joint_group_positions[1] = -1.57;  // -1/6 turn in radians
    move_group.setJointValueTarget(joint_group_positions);

    move_group.setMaxVelocityScalingFactor(0.05);
    move_group.setMaxAccelerationScalingFactor(0.05);
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

    move_group.move();
    ros::Duration(5.0).sleep();

    // get current pose
    geometry_msgs::PoseStamped target_pose;
    std::vector<geometry_msgs::Pose> waypoints;
    target_pose = move_group.getCurrentPose();
    waypoints.push_back(target_pose.pose);

    // add target pose
    cout << "add target pose" << endl;
    target_pose.pose.position.z -= 0.10;
    waypoints.push_back(target_pose.pose);

    ros::Duration(2.0).sleep();
    target_pose.pose.position.y += point_input.y;
    waypoints.push_back(target_pose.pose);

    target_pose.pose.position.z -= 0.03;
    waypoints.push_back(target_pose.pose);

    target_pose.pose.position.y -= 0.26;
    waypoints.push_back(target_pose.pose);

    move_group.setMaxVelocityScalingFactor(0.1);
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("Target Pose", "Target Pose (Cartesian path) (%.2f%% achieved)", fraction * 100.0);
    move_group.execute(trajectory);


    flag_state = 5;

}

void birdsEyePose()
{
    ROS_INFO("birds eye pose in 5 sec.");
    ros::Duration(5.0).sleep();
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
    joint_group_positions[1] = -0.785;  // -1/6 turn in radians
    joint_group_positions[2] = -1.57;  // -1/6 turn in radians
    joint_group_positions[3] = -1.83;  // -1/6 turn in radians
    joint_group_positions[4] = 1.57;  // -1/6 turn in radians
    move_group.setJointValueTarget(joint_group_positions);

    move_group.setMaxVelocityScalingFactor(0.05);
    move_group.setMaxAccelerationScalingFactor(0.05);
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

    move_group.move();
    ros::Duration(5.0).sleep();

    flag_state = 6;
}


/*************************************************************
 * @Function
 *    image callback
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
}


/*************************************************************
 * @Function
 *    depth callback
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
    depth = 0.001*cv_ptr->image.at<u_int16_t>(point_target_pixel.y, point_target_pixel.x);
}


//-----------------------------------------------------
// MAIN
//-----------------------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ur3_drawShelf");
    ros::NodeHandle nh("");
    image_transport::ImageTransport it(nh);

    image_transport::Subscriber image_sub_depth = it.subscribe("/camera/aligned_depth_to_color/image_raw", 1, imageCb_depth);
    image_transport::Subscriber image_sub_color = it.subscribe("/camera/color/image_raw", 1, imageCb_color);

	ros::Rate loop_rate(1000);
	while(ros::ok())
    {
        cout << "flag_state: " << flag_state << endl;
        switch (flag_state)
        {
            case 1:
                // detect ArUco and calc target position
                point_target_pixel = detectAruco(img_src);
                break;

            case 2:
                // transform pixel to 3d position
                point_target_3d = deprojectPixelToPoint(point_target_pixel);
                cout << "point_target_3d: " << point_target_3d << endl;
                break;
            
            case 3:
                // trasform position to new one
                point_target_3d_new = transformTargetPoint(point_target_3d);
                break;

            case 4:
                // move robot arm and draw shelf
                controlArm(point_target_3d_new);
                break;
            
            case 5:
                // move robot arm and draw shelf
                birdsEyePose();
                break;

            default:
                break;
        }
		
        ros::spinOnce();
		loop_rate.sleep();
	}

    return 0;
}