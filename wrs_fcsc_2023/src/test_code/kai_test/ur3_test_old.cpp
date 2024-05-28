//-----------------------------------------------------
// INCLUDE FILES
//-----------------------------------------------------
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <ros/ros.h>
#include <iostream>

#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h>
//-----------------------------------------------------
// DEFINES
//-----------------------------------------------------
using namespace std;

//-----------------------------------------------------
// MAIN
//-----------------------------------------------------
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


//-----------------------------------------------------
// MAIN
//-----------------------------------------------------
int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle n;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    //-----------------------------------------------------
    // GRIPPER SETUP
    //-----------------------------------------------------
    ros::NodeHandle nh;
    gripperCommand_pub = nh.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("Robotiq2FGripperRobotOutput", 10);
    gripperCommand(300);
    sleep(1);
    gripperCommand_pub = nh.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("Robotiq2FGripperRobotOutput", 10);
    gripperCommand(400);
    sleep(5);
    gripperCommand_pub = nh.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("Robotiq2FGripperRobotOutput", 10);
    //-----------------------------------------------------
    // SETUP
    //-----------------------------------------------------
    static const std::string PLANNING_GROUP = "manipulator"; //manipulator
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    //-----------------------------------------------------
    // Getting Basic Information
    //-----------------------------------------------------
    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(), std::ostream_iterator<std::string>
    (std::cout, ", "));

    //-----------------------------------------------------
    // Object
    //-----------------------------------------------------
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

    // /*-------------------------------------------------------------------*/
    // collision_objects[1].id = "side1";
    // collision_objects[1].header.frame_id = move_group.getPlanningFrame();

    // /* Define the primitive and its dimensions. */
    // collision_objects[1].primitives.resize(1);
    // collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;

    // collision_objects[1].primitives[0].dimensions.resize(3);
    // collision_objects[1].primitives[0].dimensions[0] = 0.2;
    // collision_objects[1].primitives[0].dimensions[1] = 1.0;
    // collision_objects[1].primitives[0].dimensions[2] = 1.0;

    // /* Define the pose of the table. */
    // collision_objects[1].primitive_poses.resize(1);
    // collision_objects[1].primitive_poses[0].position.x = 0.3;
    // collision_objects[1].primitive_poses[0].position.y = 0.0;
    // collision_objects[1].primitive_poses[0].position.z = 0.0;
    // collision_objects[1].primitive_poses[0].orientation.w = 1.0;

    // // /*-------------------------------------------------------------------*/
    // collision_objects[2].id = "side2";
    // collision_objects[2].header.frame_id = move_group.getPlanningFrame();

    // /* Define the primitive and its dimensions. */
    // collision_objects[2].primitives.resize(1);
    // collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;

    // collision_objects[2].primitives[0].dimensions.resize(3);
    // collision_objects[2].primitives[0].dimensions[0] = 0.2;
    // collision_objects[2].primitives[0].dimensions[1] = 1.0;
    // collision_objects[2].primitives[0].dimensions[2] = 1.0;

    // /* Define the pose of the table. */
    // collision_objects[2].primitive_poses.resize(1);
    // collision_objects[2].primitive_poses[0].position.x = -0.3;
    // collision_objects[2].primitive_poses[0].position.y = 0.0;
    // collision_objects[2].primitive_poses[0].position.z = 0.0;
    // collision_objects[2].primitive_poses[0].orientation.w = 1.0;

    // // /*-------------------------------------------------------------------*/
    // collision_objects[3].id = "side3";
    // collision_objects[3].header.frame_id = move_group.getPlanningFrame();

    // /* Define the primitive and its dimensions. */
    // collision_objects[3].primitives.resize(1);
    // collision_objects[3].primitives[0].type = collision_objects[1].primitives[0].BOX;

    // collision_objects[3].primitives[0].dimensions.resize(3);
    // collision_objects[3].primitives[0].dimensions[0] = 1.0;
    // collision_objects[3].primitives[0].dimensions[1] = 0.2;
    // collision_objects[3].primitives[0].dimensions[2] = 1.0;

    // /* Define the pose of the table. */
    // collision_objects[3].primitive_poses.resize(1);
    // collision_objects[3].primitive_poses[0].position.x = 0.0;
    // collision_objects[3].primitive_poses[0].position.y = -0.25;
    // collision_objects[3].primitive_poses[0].position.z = 0.0;
    // collision_objects[3].primitive_poses[0].orientation.w = 1.0;


    // set scene and displey status
    planning_scene_interface.applyCollisionObjects(collision_objects);
    ROS_INFO_NAMED("tutorial", "Add an object into the world");
    ros::Duration(2.0).sleep();

    //-----------------------------------------------------
    // Planning to a Pose goal
    //-----------------------------------------------------
    geometry_msgs::PoseStamped current_pose;
    current_pose = move_group.getCurrentPose("wrist_3_link");
    geometry_msgs::Pose target_pose1;

    target_pose1.position.x = 0.141006;
    target_pose1.position.y = 0.382956;
    target_pose1.position.z = 0.3;
    target_pose1.orientation.x = 0.999262;
    target_pose1.orientation.y = -0.00132944;
    target_pose1.orientation.z = -0.015854;   
    target_pose1.orientation.w = -0.034951;
    // move_group.setPoseTarget(target_pose1);

    // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    // move_group.move();

    // pose2
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(target_pose1);
    
    geometry_msgs::Pose target_pose2 = target_pose1;
    target_pose2.position.x -= 0.2;
    // target_pose2.position.z = 0.226627;
    waypoints.push_back(target_pose2);
    move_group.setPoseTarget(target_pose2);

    // success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    // move_group.move();

    move_group.setMaxVelocityScalingFactor(0.1);
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ros::Duration(5.0).sleep();
    ROS_INFO_NAMED("tutorial", "Cartesian path (%.2f%% achieved)", fraction * 100.0);
    move_group.move();
    // move_group.execute(trajectory);

    ROS_INFO("Moveit DONE");
    sleep(5);
    //-----------------------------------------------------
    // gripper
    //-----------------------------------------------------
    ROS_INFO("Gripper closing...");
    sleep(1);
    gripperCommand(75);
    gripperCommand_pub = nh.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("Robotiq2FGripperRobotOutput", 10);

    sleep(10);

    return 0;
}