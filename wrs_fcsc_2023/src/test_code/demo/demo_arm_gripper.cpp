//-----------------------------------------------------
// INCLUDE FILES
//-----------------------------------------------------
#include <ros/ros.h>
#include <std_msgs/String.h>

// standard libraries
#include <iostream>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

// msg
#include <wrs_fcsc_2023/Custom.h>
#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h>

//-----------------------------------------------------
// DEFINES
//-----------------------------------------------------
using namespace std;
static int flag = 0;

double move_x = 0.;
double move_y = 0.;
/*************************************************************
 * Gripper Pub
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
 * IMAGE PROCESSING
**************************************************************/
void ControllerCb(const wrs_fcsc_2023::Custom& data)
{
    move_x = data.move_x;
    move_y = data.move_y;
}

void ArmGripperController()
{
    ros::AsyncSpinner spinner(1);
    spinner.start();
    //-----------------------------------------------------
    // GRIPPER SETUP
    //-----------------------------------------------------
    // reset
    gripperCommand(300);
    ros::Duration(2.0).sleep();
    
    // activate
    gripperCommand(400);
    ros::Duration(2.0).sleep();
    
    //-----------------------------------------------------
    // Moveit: Setup
    //-----------------------------------------------------
    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // info
    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
    //-----------------------------------------------------
    // Moveit: Object
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

    planning_scene_interface.applyCollisionObjects(collision_objects);
    ROS_INFO_NAMED("tutorial", "Add an object into the world");
    ros::Duration(2.0).sleep();

    //-----------------------------------------------------
    // Target Pose = Home Position
    //-----------------------------------------------------
    geometry_msgs::Pose target_pose1;
    target_pose1.position.x = 0.230734;
    target_pose1.position.y = 0.275102;
    target_pose1.position.z = 0.324168;
    target_pose1.orientation.x = 1.0;
    move_group.setPoseTarget(target_pose1);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("Target Pose1", "Pose Goal %s", success ? "" : "FAILED");
    move_group.move(); 
    ros::Duration(3.0).sleep();

    //-----------------------------------------------------
    // Way Point 1
    //-----------------------------------------------------
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(target_pose1);

    geometry_msgs::Pose target_pose2 = target_pose1;
    
    // first way point
    target_pose2.position.x += move_x;
    target_pose2.position.y += move_y; 
    waypoints.push_back(target_pose2);
    ROS_INFO("First waypoint", target_pose2);

    // second way point
    target_pose2.position.z = 0.175;
    waypoints.push_back(target_pose2);
    ROS_INFO("Second waypoint", target_pose2);

    move_group.setMaxVelocityScalingFactor(0.1);
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("Target Pose2", "Target Pose2 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);
    move_group.execute(trajectory);

    ROS_INFO("Moveit DONE");
    ros::Duration(5.0).sleep();

    //-----------------------------------------------------
    // Gripper
    //-----------------------------------------------------
    ROS_INFO("Gripper closing...");
    sleep(1);
    gripperCommand(95); //75

    //-----------------------------------------------------
    // Way Point 2
    //-----------------------------------------------------
    std::vector<geometry_msgs::Pose> waypoints2; 
    waypoints2.push_back(target_pose2);

    geometry_msgs::Pose target_pose3 = target_pose2;

    // first way point
    target_pose3.position.z += 0.30;
    waypoints2.push_back(target_pose3);
    ROS_INFO("First waypoint", target_pose3);

    // second way point
    // target_pose1.position.z += 0.10;
    // waypoints2.push_back(target_pose1);
    // ROS_INFO("Second waypoint", target_pose1);

    // geometry_msgs::Pose target_pose4;
    // target_pose4.position.x = 0.352978;
    // target_pose4.position.y = 0.181667;
    // target_pose4.position.z = 0.340193;
    // target_pose4.orientation.x = 1.0;
    // waypoints2.push_back(target_pose4);
    // ROS_INFO("Second waypoint", target_pose4);

    moveit_msgs::RobotTrajectory trajectory2;
    double fraction2 = move_group.computeCartesianPath(waypoints2, eef_step, jump_threshold, trajectory2);
    ROS_INFO_NAMED("Target Pose3", "Target Pose3 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);
    move_group.execute(trajectory2);

    ROS_INFO("Moveit DONE");
    ros::Duration(3.0).sleep();
    


    moveit::planning_interface::MoveGroupInterface move_group2(PLANNING_GROUP);

    geometry_msgs::Pose target_pose4;
    target_pose4.position.x = 0.352978;
    target_pose4.position.y = 0.181667;
    target_pose4.position.z = 0.340193;
    target_pose4.orientation.x = 1.0;
    move_group2.setPoseTarget(target_pose4);

    // moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    // std::vector<double> joint_group_positions;
    // current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    // joint_group_positions[0] = 0.2;  // radians
    // move_group2.setJointValueTarget(joint_group_positions);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
    success = (move_group2.plan(my_plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    move_group2.move(); 

    ROS_INFO("Moveit DONE");
    ros::Duration(3.0).sleep();
    //-----------------------------------------------------
    // Gripper
    //-----------------------------------------------------
    ROS_INFO("Gripper opening...");
    sleep(1);
    gripperCommand(0); //75

}


//-----------------------------------------------------
// MAIN
//-----------------------------------------------------
int main(int argc, char **argv)
{
    // The name of the node
    ros::init(argc, argv, "demo_arm_gripper");
    ros::NodeHandle nh("");

    gripperCommand_pub = nh.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("Robotiq2FGripperRobotOutput", 10);

    ros::Rate loop_rate(10);
    ros::Subscriber sub = nh.subscribe("wrs_fcsc_2023_result", 10, ControllerCb);
    for (int i = 0; i <= 10; i++)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }    

    ArmGripperController();

    return 0;
}