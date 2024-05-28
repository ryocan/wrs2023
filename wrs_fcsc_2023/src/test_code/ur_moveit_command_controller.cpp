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

//-----------------------------------------------------
// FUNCTION
//-----------------------------------------------------
void  targetPositionController()
{
    //-----------------------------------------------------
    // Moveit: Setup
    //-----------------------------------------------------
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
    geometry_msgs::PoseStamped target_pose;
    std::vector<geometry_msgs::Pose> waypoints;
    
    // input current position
    target_pose = move_group.getCurrentPose();
    waypoints.push_back(target_pose.pose);
    ROS_INFO("First waypoint", target_pose.pose);
    cout << "-----<Current Pose>------" << endl << target_pose << endl;
    
    // input movement of position
    double move_x = 0.;
    double move_y = 0.;
    double move_z = 0.;

    cout << "please input move_x = ";
    cin >> move_x;
    cout << move_x << endl;

    cout << "please input move_y = ";
    cin >> move_y;
    cout << move_y << endl;

    cout << "please input move_z = ";
    cin >> move_z;
    cout << move_z << endl;

    // calib lol
    // move_x -= 0.03;
    // move_y -= 0.050;

    // calc target position
    target_pose.pose.position.x += move_x;
    target_pose.pose.position.y -= move_y;  // because ur wrist y axis and realsense y axis are opposit
    target_pose.pose.position.z -= move_z; 
    // target_pose.pose.orientation.x = 1.0;
    // target_pose.pose.orientation.y = 0.0;
    // target_pose.pose.orientation.z = 0.0;
    // target_pose.pose.orientation.w = 0.0;
    waypoints.push_back(target_pose.pose);
    ROS_INFO("Second waypoint", target_pose.pose);
    cout << "-----<Target Pose>------" << endl << target_pose.pose << endl << endl;

    // let's move using cartesian
    move_group.setMaxVelocityScalingFactor(0.1);
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("Target Pose", "Target Pose (Cartesian path) (%.2f%% achieved)", fraction * 100.0);
    move_group.execute(trajectory);
    
    ROS_INFO("Moveit DONE");
    ros::Duration(1.0).sleep();
}

//-----------------------------------------------------
// MAIN
//-----------------------------------------------------
int main(int argc, char **argv)
{
    // The name of the node
    ros::init(argc, argv, "ur_moveit_command_controller");
    ros::NodeHandle nh("");
    ros::Rate loop_rate(10);

    targetPositionController();

    return 0;
}