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

  // info
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));


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

  planning_scene_interface.applyCollisionObjects(collision_objects);
  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  ros::Duration(2.0).sleep();



  // ----------------------- target pose 1 ---------------------------------------//

  geometry_msgs::Pose target_pose1;
  target_pose1.position.x = 0.141006;
  target_pose1.position.y = 0.382956;
  target_pose1.position.z = 0.3;
  target_pose1.orientation.x = 1.0;
  move_group.setPoseTarget(target_pose1);


  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  move_group.move(); 
  sleep(3);
  // move_group.clearPathConstraints();

  // -----------------------  Cartesian Paths ---------------------------------------//
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(target_pose1);

  geometry_msgs::Pose target_pose2 = target_pose1;

  target_pose2.position.z += 0.1;
  waypoints.push_back(target_pose2);  // down

  target_pose2.position.y -= 0.1;
  waypoints.push_back(target_pose2);  // right

  // target_pose3.position.z += 0.2;
  // target_pose3.position.y += 0.2;
  target_pose2.position.x += 0.1;
  waypoints.push_back(target_pose2);  // up and left

  move_group.setMaxVelocityScalingFactor(0.1);
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);
  move_group.execute(trajectory);
    
  return 0;
}
