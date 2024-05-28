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

// tf
#include <tf/transform_listener.h>

// robotiq msg
#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h>

//-----------------------------------------------------
// DEFINES
//-----------------------------------------------------
using namespace std;
using namespace cv;

Mat img_src;
Mat img_output;

// 2d
Point point_target_pixel;

// 3d
Point3d point_target_3d;
Point3d point3d_obj;
Point3d point3d_base;
Point3d point3d_wrist;

// double
double depth = 0.;

int flag_state = 0;
int flag_if = 0;
int flag_deprojection = 0;

/*************************************************************
 * @Function
 *    gripper
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
            if(markerIds[j] == 2)
            {
                cout << "markerIds[j]: " << markerIds[j] << endl;
                cout << "point_aruco: " << point_aruco << endl;
                flag_state = 2;
            }
        }
    
        imshow("out", img_output);
        imwrite("/home/umelab/UR_test/src/wrs_fcsc_2023/img/img_output_ar.png", img_output);
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
    cout << "depth" << depth << endl;

    if (depth >= 0.1)
    {
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

        cout << "point_output: " << point_output << endl;
        flag_deprojection = 1;

        // flag_state = 2;
    }

    return point_output;
}

/*************************************************************
 * @Function
 *    transform coordinate
**************************************************************/
void transformCoordinate(Point3d point3d_input)
{
    // tf
    tf::TransformListener listener;

    // declare target frame
    std::string targetFrame = "base_link";

    // define object point
    geometry_msgs::PointStamped objectPoint;
    objectPoint.header.frame_id = "wrist_3_link";
    objectPoint.point.x = point3d_input.x + 0.03; // 
    objectPoint.point.y = point3d_input.y - 0.055; // 
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
    point3d_base = Point3d(x, y, z);

    // debug
    cout << "point3d_base :" << point3d_base << endl;     

    // update flag
}

/*************************************************************
 * @Function
 *    Init pose
**************************************************************/
void InitPose()
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
    joint_group_positions[2] = -1.57;  // -1/6 turn in radians
    joint_group_positions[3] = -1.57;  // -1/6 turn in radians

    move_group.setJointValueTarget(joint_group_positions);

    move_group.setMaxVelocityScalingFactor(0.05);
    move_group.setMaxAccelerationScalingFactor(0.05);
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

    move_group.move();
    ros::Duration(5.0).sleep();
}

/*************************************************************
 * @Function
 *    base_link to wrist_3_link
**************************************************************/
void transformCoordinateInverse(Point3d point3d_input)
{
    // tf
    tf::TransformListener listener;

    // Specify the source coordinate frame and the target coordinate frame
    std::string sourceFrame = "base_link";
    std::string targetFrame = "wrist_3_link";

    // object position in wrist_3_link
    double x = point3d_input.x;
    double y = point3d_input.y;
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
        return;
    }

    // output
    double transformedX = transformedPose.getOrigin().x();
    double transformedY = transformedPose.getOrigin().y();
    double transformedZ = transformedPose.getOrigin().z();
    point3d_wrist = Point3d(transformedX, transformedY, transformedZ);

    // debug
    cout << "point3d_wrist :" << point3d_wrist << endl;   
}

/*************************************************************
 * @Function
 *    image callback
**************************************************************/
void stockTargetRobustPosition(Point3d point3d_input)
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
    geometry_msgs::PoseStamped target_pose;
    std::vector<geometry_msgs::Pose> waypoints;
    target_pose = move_group.getCurrentPose();
    waypoints.push_back(target_pose.pose);

    target_pose.pose.position.x += (point3d_input.x); // camera and wrist ordinate trans : x -0.035
    target_pose.pose.position.y += 0.05; // camera and wrist ordinate trans : x -0.035
    waypoints.push_back(target_pose.pose);

    target_pose.pose.position.z -= 0.10;
    cout << "target_pose.pose: " << endl << target_pose.pose << endl;
    waypoints.push_back(target_pose.pose);

    // move
    move_group.setMaxVelocityScalingFactor(0.1);
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("Target Pose", "Target Pose (Cartesian path) (%.2f%% achieved)", fraction * 100.0);
    move_group.execute(trajectory);
    ros::Duration(5.0).sleep();

}

void test_move()
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
    geometry_msgs::PoseStamped target_pose;
    std::vector<geometry_msgs::Pose> waypoints;
    target_pose = move_group.getCurrentPose();
    waypoints.push_back(target_pose.pose);

    target_pose.pose.position.y -= 0.08;
    cout << "target_pose.pose: " << endl << target_pose.pose << endl;
    waypoints.push_back(target_pose.pose);

    // move
    move_group.setMaxVelocityScalingFactor(0.1);
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("Target Pose", "Target Pose (Cartesian path) (%.2f%% achieved)", fraction * 100.0);
    move_group.execute(trajectory);
    ros::Duration(5.0).sleep();

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
    gripperCommand_pub = nh.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("Robotiq2FGripperRobotOutput", 10);


    ros::Rate loop_rate(1000);
    while(ros::ok())
    {
        int key = waitKey(1);
        if (key == 'q')
            break;
    
        cout << "--------------flag_state: " << flag_state << endl;
        if (!img_src.empty())
        {
            img_output = img_src.clone();
            
            switch (flag_state)
            {
                case 0:
                    gripperCommand(300);
                    ros::Duration(2.0).sleep();
                    gripperCommand(400);    
                    ros::Duration(5.0).sleep();
                    flag_state = 1;
                    break;

                case 1:
                    point_target_pixel = detectAruco(img_src);
                    break;
                
                case 2:
                    point_target_3d = deprojectPixelToPoint(point_target_pixel);
                    if ( flag_deprojection == 1)
                    {   
                        flag_state = 3;
                        flag_deprojection = 0;
                    }
                    break;
                
                case 3:
                    transformCoordinate(point_target_3d);
                    flag_state = 4;
                    break;
                
                case 4:
                    InitPose();
                    flag_state = 5;
                    break;
                
                case 5:
                    transformCoordinateInverse(point3d_base);
                    flag_state = 6;
                    break;
                
                case 6:
                    stockTargetRobustPosition(point3d_wrist);
                    flag_state = 7;
                    break;
                
                case 7:
                    gripperCommand(255);
                    ros::Duration(2.0).sleep();
                    flag_state = 8;
                    break;
                
                case 8:
                    test_move();
                    flag_state = 9;
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