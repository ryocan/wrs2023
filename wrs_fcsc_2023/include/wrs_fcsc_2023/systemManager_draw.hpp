#include "wrs_fcsc_2023/systemManager_declare.hpp"

//-----------------------------------------------------------------
// CALLBACK
//-----------------------------------------------------------------
/*******************************************************
 * Callback function for subscribing 2D raw image
********************************************************/
void imageCb_color(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        img_src_color = cv_ptr->image.clone();
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    writer_src.write(img_src_color);
}

/*******************************************************
 * Callback function for subscribing depth info from RealSense
********************************************************/
void imageCb_depth_draw(const sensor_msgs::ImageConstPtr& msg)
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
    depth_draw = 0.001*cv_ptr->image.at<u_int16_t>(point_aruco.y, point_aruco.x);
}

//-----------------------------------------------------------------
// FUNCTION
//-----------------------------------------------------------------
/*************************************************************
 * detect ArUco marker and calculate centroid of marker
**************************************************************/
Point detectAruco(Mat img_src)
{
    // output point
    Point point_output;

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

        // calc centroid
        point_output.x = (int)(markerCorners[j][0].x + markerCorners[j][1].x + markerCorners[j][2].x + markerCorners[j][3].x ) / 4;
        point_output.y = (int)(markerCorners[j][0].y + markerCorners[j][1].y + markerCorners[j][2].y + markerCorners[j][3].y ) / 4;
        circle(img_output, point_output, 2, Scalar(0, 0, 255), 8, 8);
        imwrite("/home/ubuntupc/ダウンロード/wrs/detectAruco.png", img_output);

        // end this process
        if(markerIds[j] == 1)
        {
            cout << "point_output: " << point_output << endl;
            flag_ur3_draw = 2;  // only if we can get markerIds == 1, we can move to next step
            cout << "---- SUCCESS: Aruco detection ---- " << endl;
        }
    }  

    return point_output;
}

/*************************************************************
 * Converting 2D coordinates obtained from image processing 
 * to 3D coordinates using RealSense depth information.
 * Input: Point2D, Output: Point3D
**************************************************************/
Point3d deprojectPixelToPointDraw(Point point_pixel)
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
    point_output.x = depth_draw * ux;
    point_output.y = depth_draw * uy;
    point_output.z = depth_draw;

    // update flag
    if( depth_draw < 1.0)
    {
        flag_ur3_draw = 3;
        cout << "point3D aruco: " << point_output << endl;
        cout << "---- SUCCESS: deprojectPixelToPointDraw ---- " << endl;
    }


    return point_output;
}

/*************************************************************
 * Draw shelf by moveit.
 * In this function, we use depth info from camera to ArUco marker
**************************************************************/
void drawShelf(Point3d point3d_input)
{
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // debug
    cout << "y move :" << (point3d_input.z - 0.45) << endl;

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
    joint_group_positions[1] = -1.57;
    move_group.setJointValueTarget(joint_group_positions);

    move_group.setMaxVelocityScalingFactor(0.1);
    move_group.setMaxAccelerationScalingFactor(0.1);
    bool_draw1 = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO("First Move (joint space goal) %s", bool_draw1 ? "" : "FAILED");
    move_group.move();  // move

    if(bool_draw1 == true)
    {
        if ((point3d_input.z - 0.45) <= 0.30)
        {
            cout << "---- SUCCESS: Draw Shelf1---- " << endl;

            // Second move
            geometry_msgs::PoseStamped target_pose;
            std::vector<geometry_msgs::Pose> waypoints;

            target_pose = move_group.getCurrentPose();
            waypoints.push_back(target_pose.pose);

            target_pose.pose.position.z -= 0.09;
            waypoints.push_back(target_pose.pose);

            target_pose.pose.position.y += (point3d_input.z - 0.45);
            waypoints.push_back(target_pose.pose);

            target_pose.pose.position.z -= 0.03;
            waypoints.push_back(target_pose.pose);

            if((point3d_input.z - 0.45) <= 0.1)
                target_pose.pose.position.y -= 0.20;
            else
                target_pose.pose.position.y -= 0.22;
            waypoints.push_back(target_pose.pose);

            move_group.setMaxVelocityScalingFactor(0.1);
            move_group.setMaxAccelerationScalingFactor(0.1);
            moveit_msgs::RobotTrajectory trajectory;

            const double jump_threshold = 0.0;
            const double eef_step = 0.01;
        
            double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
            ROS_INFO("Second Move (Cartesian path) (%.2f%% achieved)", fraction * 100.0);
            move_group.execute(trajectory); // move

            // save last pose
            draw_pose = move_group.getCurrentPose();
            cout << "draw Pose: " << draw_pose << endl;

            // update flag
            flag_ur3_draw = 4;
            cout << "---- SUCCESS: Draw Shelf2---- " << endl;
        }
        else
        {
            bool_draw2 = false;
        }
        
    }

}

/*************************************************************
 * Do bird's eye pose for the preparation for disposal task
**************************************************************/
void birdsEyePose()
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

    // Do bird's eye pose
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions[0] = -1.57;
    joint_group_positions[1] = -0.785;
    joint_group_positions[2] = -1.57;
    joint_group_positions[3] = -1.83;
    joint_group_positions[4] = 1.57;
    joint_group_positions[5] = 0;
    move_group.setJointValueTarget(joint_group_positions);

    move_group.setMaxVelocityScalingFactor(0.5);
    move_group.setMaxAccelerationScalingFactor(0.5);
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO("Bird's Eye View (joint space goal) %s", success ? "" : "FAILED");
    move_group.move();  // move

    // update flag
    flag_ur3_draw = 5;
    cout << "---- SUCCESS: Birds Eye Pose---- " << endl;
}

//-----------------------------------------------------------------
// DRAW
//-----------------------------------------------------------------
void draw()
{
    if (!img_src_color.empty())
    {
        // setting
        img_output = img_src_color.clone();

        switch (flag_ur3_draw)
        {
            case 1: // detect ArUco and calculate position
                cout << "-------------- START: Draw Task ------------- " << endl;
                gripperControler(300);
                ros::Duration(2.0).sleep();
                gripperControler(400);    
                ros::Duration(2.0).sleep();
                point_aruco = detectAruco(img_src_color);
                break;
            
            case 2: // deproject pixel to point3D
                if(depth_draw != 0)
                    point3d_aruco = deprojectPixelToPointDraw(point_aruco);
                break;
            
            case 3: // draw shelf
                drawShelf(point3d_aruco);
                break;
            
            case 4: // do bird's eye pose
                birdsEyePose();
                break;
            
            case 5: // finish draw shelf task
                cout << "-------------- FINISH: Draw Task ------------- " << endl << endl;
                if(bool_draw1 == true)
                    flag_ur3_system = 2;
                else
                    flag_ur3_system = 4;
                break;
            
            default:
                break;
        }
    }

}