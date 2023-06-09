/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  static const std::string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());

  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  // target1=home position
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.x = 1.0;
  //target_pose1.position.x = 0.141006;
  //target_pose1.position.y = 0.382956;
  target_pose1.position.x = 0.15;
  target_pose1.position.y = 0.15;
  target_pose1.position.z = 0.3;
  move_group.setPoseTarget(target_pose1);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  move_group.move(); 
  sleep(3);

  //move_group.clearPathConstraints();

//first step
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(target_pose1);

  geometry_msgs::Pose target_pose2 = target_pose1;

  target_pose2.position.x += 0.1;
  target_pose2.position.y += 0.1;
  waypoints.push_back(target_pose2);
  
  target_pose2.position.z -= 0.20;
  waypoints.push_back(target_pose2);

  move_group.setMaxVelocityScalingFactor(0.1);

  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);
  move_group.execute(trajectory);

  sleep(3);


//second step
  std::vector<geometry_msgs::Pose> waypoints2; 
  waypoints2.push_back(target_pose2);
  geometry_msgs::Pose target_pose3= target_pose2;

  target_pose3.position.z += 0.20;
  waypoints2.push_back(target_pose3);

  waypoints2.push_back(target_pose1);

  geometry_msgs::Pose target_pose4= target_pose1;
  target_pose4.position.z -= 0.20;
  waypoints2.push_back(target_pose4);

  moveit_msgs::RobotTrajectory trajectory2;
  //const double jump_threshold = 0.0;
  //const double eef_step = 0.01;
  double fraction2= move_group.computeCartesianPath(waypoints2, eef_step, jump_threshold, trajectory2);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction2 * 100.0);
  move_group.execute(trajectory2);

  sleep(3);

  return 0;
}
