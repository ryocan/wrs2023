#include "wrs_fcsc_2023/systemManager_declare.hpp"

//-----------------------------------------------------------------
// FUNCTION
//-----------------------------------------------------------------
/*************************************************************
 * Putback shelf
**************************************************************/
void putBackPose(geometry_msgs::PoseStamped input_pose, Point3d point3d_input)
{
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // moveit setup
    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // show info
    ROS_INFO("Planning frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

    // prepare collision object
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

    // First move
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    joint_group_positions[0] = -1.57;
    joint_group_positions[1] = -0.785;
    joint_group_positions[2] = -1.57;
    joint_group_positions[3] = -1.57;
    joint_group_positions[4] = 1.57;
    joint_group_positions[5] = 1.57;
    move_group.setJointValueTarget(joint_group_positions);

    move_group.setMaxVelocityScalingFactor(0.5);
    move_group.setMaxAccelerationScalingFactor(0.5);
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO("First Move (joint space goal) %s", success ? "" : "FAILED");
    move_group.move();  // move

    // Second move
    geometry_msgs::PoseStamped target_pose;
    std::vector<geometry_msgs::Pose> waypoints;

    target_pose = move_group.getCurrentPose();
    waypoints.push_back(target_pose.pose);

    target_pose = input_pose;
    waypoints.push_back(target_pose.pose);

    if((point3d_input.z - 0.45) <= 0.1)
        target_pose.pose.position.y = (point3d_input.z - 0.45) + 0.20;
    else
        target_pose.pose.position.y = (point3d_input.z - 0.45) + 0.27;
    waypoints.push_back(target_pose.pose);

    cout << "target_pose.pose: " << target_pose.pose << endl;
    move_group.setMaxVelocityScalingFactor(0.5);
    moveit_msgs::RobotTrajectory trajectory;

    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
   
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO("Second Move (Cartesian path) (%.2f%% achieved)", fraction * 100.0);
    move_group.execute(trajectory); // move

    // update flag
    flag_ur3_putback = 3;
    cout << "---- SUCCESS: Putback Shelf---- " << endl;
}

/*************************************************************
 * Do pose for ArUco marker detection to get distance from shelf
**************************************************************/
void detctArucoPose2()
{
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // moveit setup
    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // show info
    ROS_INFO("Planning frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

    // prepare collision object
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

    // Pose
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    
    joint_group_positions[1] = 0;  
    joint_group_positions[2] = -1.57;
    joint_group_positions[3] = -1.57;  

    move_group.setJointValueTarget(joint_group_positions);

    move_group.setMaxVelocityScalingFactor(0.5);
    move_group.setMaxAccelerationScalingFactor(0.5);
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO("First Move (joint space goal) %s", success ? "" : "FAILED");
    move_group.move();  // move


    // update flag
    flag_ur3_putback = 4;

}

//-----------------------------------------------------------------
// PUTBACK
//-----------------------------------------------------------------
void putback()
{
    switch (flag_ur3_putback)
    {
        case 1: // close gripper
            gripperControler(300);
            ros::Duration(2.0).sleep();
            gripperControler(400);    
            ros::Duration(2.0).sleep();
            gripperControler(500);    
            ros::Duration(2.0).sleep();
            flag_ur3_putback = 2;
            break;
        
        case 2: // putback
            cout << "draw Pose: " << draw_pose << endl;
            putBackPose(draw_pose, point3d_aruco);
            break;
        
        case 3: // detect aruco pose
            detctArucoPose2();
            break;
        
        case 4:
            gripperControler(300);
            ros::Duration(2.0).sleep();
            gripperControler(400);    
            ros::Duration(2.0).sleep();
            flag_ur3_putback = 5;
            break;

        case 5: // 
            cout << "-------------- FINISH: Putback Task ------------- " << endl << endl;
            flag_ur3_system = 4;
            break;
        
        default:
            break;
    }
    

}