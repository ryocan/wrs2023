#include "wrs_fcsc_2023/systemManager_declare.hpp"


//-----------------------------------------------------------------
// CALLBACK
//-----------------------------------------------------------------
/*******************************************************
 * Callback function for subscribing 2D image
********************************************************/
void imageCb_yolact(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        img_src_yolact = cv_ptr->image.clone();
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

/*******************************************************
 * Callback function for subscribing depth info from RealSense
********************************************************/
void imageCb_depth_disposal(const sensor_msgs::ImageConstPtr& msg)
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

    // get depth info 
    depth_disposal.clear();
    for(int i = 0; i < point2d_obj_centroid.size(); i++)
    {
        double calc =  0.001*cv_ptr->image.at<u_int16_t>(point2d_obj_centroid[i].y, point2d_obj_centroid[i].x);
        depth_disposal.push_back(calc);
    }

    // for onr object
    depth_disposal_one = 0.001*cv_ptr->image.at<u_int16_t>(point2d_obj_centroid_one.y, point2d_obj_centroid_one.x);


}

//-----------------------------------------------------------------
// IMAGE PROCESSING
//-----------------------------------------------------------------
/*******************************************************
 * Perform thresholding on the image to extract only 
 * the desired object regions. 
 * Input: image, Output: mask image
********************************************************/
Mat createMaskImg(Mat img_input)
{
    // declre output img
    Mat img_mask = Mat::zeros(img_input.size(), img_input.type());

    Mat img_hsv;
    cvtColor(img_input, img_hsv, CV_BGR2HSV, 3);

    // extract object from YOLACT result using RGB info
    // only extract black background and then invert colors
    Scalar lower(  0,   0,   0);
    Scalar upper(  0,   0,   0);
    cv::inRange(img_hsv, lower, upper, img_mask);
    bitwise_not(img_mask, img_mask);

    // show result
    imwrite("/home/ubuntupc/ダウンロード/wrs/createMaskImg.png", img_mask);

    return img_mask;
}

/*******************************************************
 * Get object's contours from mask image
 * Input: mask image, Output: contours
********************************************************/
vector<vector<Point>> extractObjectContours(Mat img_input)
{
    // extract contours from img_mask
    vector<vector<Point>> contour_output;
    vector<Vec4i> hierarchy;
    cv::findContours(img_input, contour_output, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    
    // show result
    cv::drawContours(img_output, contour_output, -1, (0, 0, 255), 3);
    imshow("/home/ubuntupc/ダウンロード/wrs/extractObjectContours.png", img_output);

    return contour_output;
}

/*************************************************************
 * Calculation object's centroid from object's contour info
 * Input: contour info, Output: centroid point2D
**************************************************************/
vector<Point> calcCentroid(vector<vector<Point>> contour_input)
{
    // declare output Point2D
    vector<Point> point_output(contour_input.size());

    // prepare mu and mc for centroid detection
    vector<Moments> mu_obj(contour_input.size());
    for (int i = 0; i < contour_input.size(); i++)
        mu_obj[i] = cv::moments(contour_input[i], false );
    vector<Point2f> mc_obj(contour_input.size());

    // calculation centroid
    for (int i = 0; i < contour_input.size(); i++)
    {
        double centroid_x = mu_obj[i].m10 / mu_obj[i].m00;
        double centroid_y = mu_obj[i].m01 / mu_obj[i].m00;

        if ( -1000 < centroid_x && centroid_x < 1000)
        {
            if ( -1000 < centroid_y && centroid_y < 1000)
            {
                point_output[i] = cv::Point(int(centroid_x), int(centroid_y));
            }
        }


        if (isnan(mu_obj[i].m10 / mu_obj[i].m00) != true && isnan(mu_obj[i].m01 / mu_obj[i].m00) != true)
        {
            cv::circle(img_output, point_output[i], 10, cv::Scalar(255, 255, 255), 8, 8);
            cv::circle(img_output, point_output[i],  8, cv::Scalar(0, 0, 255), 8, 8);
            cv::circle(img_output, point_output[i],  2, cv::Scalar(255, 255, 255), 8, 8);
        }
    }

    // show result
    cout << "point_output: " << endl << point_output << endl;
    imwrite("/home/ubuntupc/ダウンロード/wrs/calcCentroid.png", img_output);
    writer_src.write(img_src_color);

    return point_output;
}

/*************************************************************
 * Calculation object's centroid from object's contour info
 * Input: contour info, Output: centroid point2D
**************************************************************/
Point calcCentroidOne(vector<vector<Point>> contour_input)
{
    // declare output Point2D
    Point point_output;

        // prepare mu and mc for centroid detection
    vector<Moments> mu_obj(contour_input.size());
    for (int i = 0; i < contour_input.size(); i++)
        mu_obj[i] = cv::moments(contour_input[i], false);
    vector<Point2f> mc_obj(contour_input.size());

    // calculation centroid
    for (int i = 0; i < contour_input.size(); i++)
    {
        double centroid_x = mu_obj[i].m10 / mu_obj[i].m00;
        double centroid_y = mu_obj[i].m01 / mu_obj[i].m00;
        point_output = cv::Point(int(centroid_x), int(centroid_y));

        if (isnan(mu_obj[i].m10 / mu_obj[i].m00) != true && isnan(mu_obj[i].m01 / mu_obj[i].m00) != true)
        {
            cv::circle(img_output, point_output, 10, cv::Scalar(255, 255, 255), 8, 8);
            cv::circle(img_output, point_output,  8, cv::Scalar(0, 0, 255), 8, 8);
            cv::circle(img_output, point_output,  2, cv::Scalar(255, 255, 255), 8, 8);
        }
    }

    // show result
    cout << "point_output_one: " << endl << point_output << endl;
    imwrite("/home/ubuntupc/ダウンロード/wrs/calcCentroidOne.png", img_output);
    writer_src.write(img_src_color);

    return point_output;
}

/*************************************************************
 * deproject pixel to point3d
**************************************************************/
vector<Point3d> deprojectPixelToPoint(vector<Point> point_pixel)
{
    vector<Point3d> point_output;
    if (!depth_disposal.empty())
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
            Point3d calc(depth_disposal[i] * ux,(-1)* depth_disposal[i] * uy, depth_disposal[i]);
            point_output.push_back(calc);
        }

        // update flag
        cout << "deproject pixel to point: " << endl << point_output << endl;
        flag_ur3_disposal = 6;

    }

    return point_output;
}

/*************************************************************
 * Transform object's point(3D) from wrist_3_link coordinate
 * to base_link coordinate using /tf.
 * Input: Object's Point3D, Output: Object's Point3D in base_link
**************************************************************/
vector<Point3d> transformCoordinateToBase(vector<Point3d> point3d_input)
{
    // point output
    vector<Point3d> point3d_output;

    // tf
    tf::TransformListener listener;

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
        point3d_output.push_back(Point3d(x , y , z));
        ///////////////////////////////////////////////////////////
    }

    // debug
    cout << "point3d_base :" << point3d_output << endl;
    flag_ur3_disposal = 7;

    return point3d_output;
}

/*************************************************************
 * To reach the target position, first go to the initial posture.
**************************************************************/
void moveitInitialPose()
{
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    ROS_INFO_NAMED("Initial Pose", "Planning frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("Initial Pose", "End effector link: %s", move_group.getEndEffectorLink().c_str());
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
    
    // ------------------- Collision Setup
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

    // define pose
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.0;
    collision_objects[0].primitive_poses[0].position.y = 0.0;
    collision_objects[0].primitive_poses[0].position.z = -0.15;
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;
    planning_scene_interface.applyCollisionObjects(collision_objects);
    ROS_INFO_NAMED("Initial Pose", "Add an object into the world");
    ros::Duration(1.0).sleep();

    // ------------------- Pose Setup
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    
    // set joint radius
    joint_group_positions[0] = -1.57;
    joint_group_positions[1] = -1.27409;
    joint_group_positions[2] = -1.27409;
    joint_group_positions[3] = -2.18166;
    
    move_group.setJointValueTarget(joint_group_positions);

    // move setup
    move_group.setMaxVelocityScalingFactor(0.5);
    move_group.setMaxAccelerationScalingFactor(0.5);
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("Initial Pose", "Visualizing plan (joint space goal) %s", success ? "" : "FAILED");

    // move
    move_group.move();
}

/*************************************************************
 * Transform object's point(3D) from wrist_3_link coordinate
 * to base_link coordinate using /tf.
 * Input: Object's Point3D, Output: Object's Point3D in base_link
**************************************************************/
Point3d transformCoordinateToWrist(Point3d point3d_input)
{
    // output
    Point3d point3d_output;

    // tf
    tf::TransformListener listener;

    // Specify the source coordinate frame and the target coordinate frame
    std::string sourceFrame = "base_link";
    std::string targetFrame = "wrist_3_link";

    // object position in wrist_3_link
    double x = point3d_input.x + 0.035;
    double y = point3d_input.y - 0.055;
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
    }

    // output
    double transformedX = transformedPose.getOrigin().x();;
    double transformedY = transformedPose.getOrigin().y();
    double transformedZ = transformedPose.getOrigin().z();
    double x_input = transformedX - 0.035;
    double y_input = abs(transformedY + 0.055);
    // if (y_input >= 0.3)     // error: if y_input >=0.3, robotic arm will collision with shelf
    //     y_input = 0.3;
    // point3d_output = Point3d(x_input, y_input , transformedZ);    // be careful

    x_input = x_input - 0.08;
    if( x_input > 0.2)
        x_input = x_input - 0.02;
    else if (x_input < -0.2)
        x_input = x_input + 0.02;

    y_input = abs(y_input-0.25);
    if (y_input >= 0.3)     // error: if y_input >=0.3, robotic arm will collision with shelf
        y_input = 0.3;

    point3d_output = Point3d(x_input, y_input, transformedZ);  
    // debug
    cout << "point3d_wrist :" << point3d_output << endl;  

    return point3d_output; 
}

/*************************************************************
 * First, UR3 will move to the position of the object 
 * calculated by the first image recognition. 
 * Perform rough positioning.
**************************************************************/
void moveitRoughPosition(Point3d point3d_input)
{
    ros::AsyncSpinner spinner(1);
    spinner.start();

    if(point3d_input.z < 0.4) //0.3
    {
        // moveit plannning setup
        static const std::string PLANNING_GROUP = "manipulator";
        moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

        // display info
        ROS_INFO("Planning frame: %s", move_group.getPlanningFrame().c_str());
        ROS_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());
        std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
        
        // Object setup: make base(like table)
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
        ROS_INFO("Add an object into the world\n");

        // move to rough target position
        geometry_msgs::PoseStamped target_pose;
        std::vector<geometry_msgs::Pose> waypoints;
        target_pose = move_group.getCurrentPose();
        waypoints.push_back(target_pose.pose);

        target_pose.pose.position.z -= 0.1; 
        waypoints.push_back(target_pose.pose);

        target_pose.pose.position.x += point3d_input.x;
        target_pose.pose.position.y += point3d_input.y; 
        waypoints.push_back(target_pose.pose);

        cout << "target_pose.pose: " << endl << target_pose.pose << endl;

        // move action
        move_group.setMaxVelocityScalingFactor(0.5);
        moveit_msgs::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        ROS_INFO_NAMED("Target Pose", "Target Pose (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

        if (fraction != 1.0)    // if nove action failed, lower the threshold
        {
            cout << "--retry--" << endl;
            const double eef_step = 0.02;
            fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
            ROS_INFO_NAMED("Target Pose", "Target Pose (Cartesian path) (%.2f%% achieved)", fraction * 100.0);  
        }

        move_group.execute(trajectory);
    }
    else
    {
        ROS_WARN("input z is too big.");
    }
    
}

/*************************************************************
 * 
**************************************************************/
void PrecisionPrepare ()
{
    // mask img
    img_mask = createMaskImg(img_src_yolact);

    // get contours
    contours_obj = extractObjectContours(img_mask);

    // get centroid
    point2d_obj_centroid = calcCentroid(contours_obj);

    if(!point2d_obj_centroid.empty())
    {
        flag_ur3_disposal_getImg = 1;

        for (int i = 0; i < point2d_obj_centroid.size(); i++)
        {
            distanceFromCenter = abs(point2d_obj_centroid[i].x - imgCenter.x) + abs(point2d_obj_centroid[i].y - imgCenter.y);
        
            if (distanceFromCenter < num_minDistance)
            {
                num_minDistance = distanceFromCenter;
                num_minDistance = i;
            }
        }

        cout << "num_minDistance: " << num_minDistance << endl;
        // calculation (using param from /camera/depth/camera_info)
        double x = (point2d_obj_centroid[num_minDistance].x - 321.7812805175781) / 380.3281555175781;
        double y = (point2d_obj_centroid[num_minDistance].y - 238.0182342529297) / 380.3281555175781;
        double r2 = x * x + y * y;
        double f = 1 + 0*r2 + 0*r2*r2 + 0*r2*r2*r2;
        double ux = x*f + 2*0*x*y + 0*(r2 + 2*x*x);
        double uy = y*f + 2*0*x*y + 0*(r2 + 2*y*y);
        Point3d targetPrecision(0.2 * ux, 0.2 * uy, 0.2);
        cout << "targetPrecision: " << targetPrecision << endl;

        // moveitPrecisionPosition(targetPrecision);

        pca(contours_obj[num_minDistance]);
    }


}

/*************************************************************
 * After rough positioning, position the camera so that 
 * its center is directly above the object. 
 * This allows us to obtain more accurate information about the object.
**************************************************************/
void moveitPrecisionPosition(Point3d point3d_input)
{
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // moveit plannning setup
    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // display info
    ROS_INFO("Planning frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
    
    // Object setup: make base(like table)
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
    ROS_INFO("Add an object into the world\n");

    // move to precision target position
    geometry_msgs::PoseStamped target_pose;
    std::vector<geometry_msgs::Pose> waypoints;
    target_pose = move_group.getCurrentPose();
    waypoints.push_back(target_pose.pose);

    // target_pose.pose.position.z -= 0.05; 
    // waypoints.push_back(target_pose.pose);

    target_pose.pose.position.x += (point3d_input.x);
    target_pose.pose.position.y += (point3d_input.y); 
    waypoints.push_back(target_pose.pose);

    cout << "target_pose.pose: " << endl << target_pose.pose << endl;

    // move action
    move_group.setMaxVelocityScalingFactor(0.5);
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO("Target Pose (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

    if (fraction != 1.0)    // if nove action failed, lower the threshold
    {
        cout << "--retry--" << endl;
        const double eef_step = 0.02;
        fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        ROS_INFO("Target Pose (Cartesian path) (%.2f%% achieved)", fraction * 100.0);  
    }

    move_group.execute(trajectory);
}

/*************************************************************
 * movet final position
**************************************************************/
void moveitFinalPosition(double double_wrist)
{
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // moveit plannning setup
    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // display info
    ROS_INFO("Planning frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
    
    // Object setup: make base(like table)
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

    // // First, move UR3 so that the center of the gripper and the center of the object are aligned
    // geometry_msgs::PoseStamped target_pose;
    // std::vector<geometry_msgs::Pose> waypoints;
    // target_pose = move_group.getCurrentPose();
    // waypoints.push_back(target_pose.pose);

    // // target_pose.pose.position.x += ( -0.035);
    // target_pose.pose.position.y += (  0.048); 
    // waypoints.push_back(target_pose.pose);

    // // move action
    // move_group.setMaxVelocityScalingFactor(0.5);
    // moveit_msgs::RobotTrajectory trajectory;
    // const double jump_threshold = 0.0;
    // const double eef_step = 0.01;
    // double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    // ROS_INFO_NAMED("Target Pose", "Target Pose (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

    // if (fraction != 1.0)
    // {
    //     cout << "--retry--" << endl;
    //     const double eef_step = 0.02;
    //     fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    //     ROS_INFO_NAMED("Target Pose", "Target Pose (Cartesian path) (%.2f%% achieved)", fraction * 100.0);  
    // }u
    // move_group.execute(trajectory);

    // Rotate the wrist to match the posture of the object
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    
    joint_group_positions[5] += double_wrist;
    move_group.setJointValueTarget(joint_group_positions);

    move_group.setMaxVelocityScalingFactor(0.5);
    move_group.setMaxAccelerationScalingFactor(0.5);
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
    move_group.move();

    // // calc depth
    // Mat img_mask_one = createMaskImg(img_src_yolact);
    // vector<vector<Point>> contours_obj_one;
    // contours_obj_one = extractObjectContours(img_mask_one);
    // point2d_obj_centroid_one = calcCentroidOne(contours_obj_one);
    // cout << "system: point2d_obj_centroid_one: " << point2d_obj_centroid_one << endl;
    // imwrite("/home/ubuntupc/ダウンロード/wrs/calcCentroidOne.png", img_mask);

    // birdsEye -> Home
    geometry_msgs::PoseStamped target_pose2;
    std::vector<geometry_msgs::Pose> waypoints2;
    target_pose2 = move_group.getCurrentPose();
    waypoints2.push_back(target_pose2.pose);

    target_pose2.pose.position.z += -0.1;
    waypoints2.push_back(target_pose2.pose);

    // move
    move_group.setMaxVelocityScalingFactor(0.1);
    moveit_msgs::RobotTrajectory trajectory2;
    const double jump_threshold2 = 0.0;
    const double eef_step2 = 0.01;
    double fraction2 = move_group.computeCartesianPath(waypoints2, eef_step2, jump_threshold2, trajectory2);
    ROS_INFO_NAMED("Target Pose", "Target Pose (Cartesian path) (%.2f%% achieved)", fraction2 * 100.0);
    move_group.execute(trajectory2);
}

void moveitFinalPosition2()
{
    // debug
    cout << "depth_disposal_one: " << depth_disposal_one << endl; 

    //
    double input_z = 0.;
    if (point2d_obj_centroid_one.x == 0 && point2d_obj_centroid_one.y == 0)
        input_z = -0.02;
    else
        input_z = (-1) * (depth_disposal_one - 0.16);
    //--------------------------------------------------------------- z
    // // birdsEye -> Home
    // geometry_msgs::PoseStamped target_pose2;
    // std::vector<geometry_msgs::Pose> waypoints2;
    // target_pose2 = move_group.getCurrentPose();
    // waypoints2.push_back(target_pose2.pose);

    // target_pose2.pose.position.z += -0.1;
    // waypoints2.push_back(target_pose2.pose);

    // // move
    // move_group.setMaxVelocityScalingFactor(0.1);
    // moveit_msgs::RobotTrajectory trajectory2;
    // const double jump_threshold2 = 0.0;
    // const double eef_step2 = 0.01;
    // double fraction2 = move_group.computeCartesianPath(waypoints2, eef_step2, jump_threshold2, trajectory2);
    // ROS_INFO_NAMED("Target Pose", "Target Pose (Cartesian path) (%.2f%% achieved)", fraction2 * 100.0);
    // move_group.execute(trajectory2);        point_output[i] = cv::Point(int(centroid_x), int(centroid_y));
}

/*************************************************************
 * pca
**************************************************************/
Mat maskForPrecison(Mat img)
{
    Mat img_mask = Mat::zeros(img_src_color.size(), img_src_color.type());

    // convert to HSV image
    Mat img_hsv;
    cvtColor(img, img_hsv, COLOR_BGR2HSV);

    // extract region based on HSV parameter
    Scalar Lower( 10,   0,   0);
    Scalar Upper(179, 255, 255);
    inRange(img_hsv, Lower, Upper, img_mask);

    return img_mask;
}

void drawContours(vector<Point> contours, Mat img_mask, string mode)
{
    // declare for LineIterator
    Point li_start;
    Point li_goal;
    

    for (int j = 0; j < contours.size(); j++)
    {
        // specify start and end point for cv::LineIterator
        li_start = contours[j];

        if (j == contours.size() - 1)
            li_goal = contours[0];
        else    
            li_goal = contours[j + 1];

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

void pca(vector<Point> &pts)
{
    cv::Mat data_pts = cv::Mat(pts.size(), 2, CV_64F); // [pts.size() x 2] 行列
    for (int i = 0; i < data_pts.rows; i++) 
    {
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
    for (int i = 0; i < 2; i++) 
    {
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
    wrist_rotation = angle;
    cout << "wrist_rotation: " << wrist_rotation << endl;

    //-------------------------------------------------------------------------------------
    img_black = Mat::zeros(img_src_color.size(), img_src_color.type());
    Mat img_black2 = Mat::zeros(img_src_color.size(), img_src_color.type());
    int denominator = (p2.x - cntr.x);
    if (denominator <= 0)
        denominator = 1;
        
    Point point1(0, (p2.y - cntr.y) * (0 - cntr.x) / denominator + cntr.y);
    Point point2(img_output.cols, (p2.y - cntr.y) * (img_output.cols - cntr.x) / denominator + cntr.y);
    line(img_black2, point1, point2, Scalar(0, 255, 0), 1);
    line(img_output, point1, point2, Scalar(255, 255, 255), 1);

    Mat img_mask2 = maskForPrecison(img_black2);
    vector<vector<Point>> contour2;
    vector<Vec4i> hierarchy2;
    cv::findContours(img_mask2, contour2, hierarchy2, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    contours_hand_contact.clear();
    drawContours(contours_obj[num_minDistance], img_mask, "obj");
    drawContours(contour2[0], img_mask2, "hand");
    for(int i = 0; i < contours_hand_contact.size(); i++)
    {
        circle(img_output, contours_hand_contact[i],  10, Scalar(255, 255, 255), 8, 8);
        circle(img_output, contours_hand_contact[i],   8, Scalar(  0, 255,   0), 8, 8);
        circle(img_output, contours_hand_contact[i],   2, Scalar(255, 255, 255), 8, 8);
    }
    double width = sqrt(pow((contours_hand_contact[0].x - contours_hand_contact[1].x), 2) + pow((contours_hand_contact[0].y - contours_hand_contact[1].y), 2));
    gripper_width = width;
    cout << "gripper width: " << gripper_width << endl;

    imwrite("/home/ubuntupc/ダウンロード/wrs/pca.png", img_output);
    writer_src.write(img_src_color);
}

// /*************************************************************
//  * move to disposal position
// **************************************************************/    ros::Duration(5.0).sleep();
void moveitDisposalPosition()
{
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // info
    ROS_INFO("Planning frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());
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

    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.0;
    collision_objects[0].primitive_poses[0].position.y = 0.0;
    collision_objects[0].primitive_poses[0].position.z = -0.15;
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;
    planning_scene_interface.applyCollisionObjects(collision_objects);
    ROS_INFO("Add an object into the world");

    // move
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    
    joint_group_positions[0] = -1.57;
    joint_group_positions[1] = -1.27409;
    joint_group_positions[2] = -1.27409;
    joint_group_positions[3] = -2.18166;
    joint_group_positions[5] = 0;

    move_group.setJointValueTarget(joint_group_positions);

    move_group.setMaxVelocityScalingFactor(0.5);
    move_group.setMaxAccelerationScalingFactor(0.5);
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan (joint space goal) %s", success ? "" : "FAILED");

    move_group.move();    
    // -------move to container
    current_state = move_group.getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    
    // joint_group_positions[0] = -3.14; 
    joint_group_positions[0] = -3.14;
    joint_group_positions[1] = -1.57;
    joint_group_positions[2] = -1.57;
    joint_group_positions[3] = -1.57;
    joint_group_positions[4] = 1.57;
    joint_group_positions[5] = 0;

    move_group.setJointValueTarget(joint_group_positions);

    move_group.setMaxVelocityScalingFactor(0.5);
    move_group.setMaxAccelerationScalingFactor(0.5);
    success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

    move_group.move();

     // ---------------------------------------z
    geometry_msgs::PoseStamped target_pose;
    std::vector<geometry_msgs::Pose> waypoints;
    target_pose = move_group.getCurrentPose();
    waypoints.push_back(target_pose.pose);

    target_pose.pose.position.z += (-0.1) ; //- 0.035
    waypoints.push_back(target_pose.pose);

    // move
    move_group.setMaxVelocityScalingFactor(0.1);
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("Target Pose", "Target Pose (Cartesian path) (%.2f%% achieved)", fraction * 100.0);
    move_group.execute(trajectory);
}

void customerJudge()
{
    if(flag_customer_system.data != 0)
    {

        cout << "--- Customer Detection --- " << endl;
        // system("cd ~/catkin_ws/src/fcsc/scripts ; python3 kaere.py");
        system("(cd ~/catkin_ws/src/fcsc/scripts ; python3 kaere.py)&");
        cout << "flag_customer : " << flag_customer_system.data << endl;
        flag_ur3_disposal = 15;  //////////////////////////////////////////////////////////////


        flag_customer_sound = 1;
        
    }
}


//-----------------------------------------------------------------
// DISPOSAL
//-----------------------------------------------------------------

int obj_num = 0;    // to identify which object to disposal

void disposal()
{
    if (!img_src_yolact.empty())
    {
        // setting
        img_output = img_src_color.clone();
        writer_src.write(img_output);

        switch (flag_ur3_disposal)
        {
            case 1: // gripper 
                cout << "-------------- START: Disposal Task ------------- " << endl;
                flag_ur3_disposal = 2;
                break;
            
            case 2: // create mask image
                img_mask = createMaskImg(img_src_yolact);
                flag_ur3_disposal = 3;
                break;
            
            case 3: // get all object's contours
                contours_obj = extractObjectContours(img_mask);
                flag_ur3_disposal = 4;
                break;
            
            case 4:  // calclate centroid from contour
                point2d_obj_centroid = calcCentroid(contours_obj);
                flag_ur3_disposal = 5;
                break;
            
            case 5: // pixel to point3d
                point3d_obj_centroid = deprojectPixelToPoint(point2d_obj_centroid);
                break;
            
            case 6: // transform to base coordinate
                point3d_obj_base = transformCoordinateToBase(point3d_obj_centroid);
                obj_total = point2d_obj_centroid.size();
                cout <<  "point2d_obj_centroid.size(): " << point2d_obj_centroid.size() << endl;
                break;

            case 7: // disposal for one object
                cout << "--- Disposal --- " << endl;
                cout << "obj_num: " << obj_num << endl;

                // gripper 
                customerJudge();
                ros::spinOnce();
                gripperControler(300);
                ros::Duration(2.0).sleep();
                gripperControler(400);    
                ros::Duration(2.0).sleep();

                if(flag_customer_sound == 1)
                {
                    flag_ur3_disposal = 15;////////////////last 
                    flag_system_continue = 1;
                    flag_customer_sound = 0;
                }
                else
                    flag_ur3_disposal = 8;
                break;
            
            case 8:
                // move to initial pose
                customerJudge();
                moveitInitialPose();

                if(flag_customer_sound == 1)
                {
                    flag_ur3_disposal = 15;////////////////last 
                    flag_system_continue = 1;
                    flag_customer_sound = 0;
                }
                else
                    flag_ur3_disposal = 9;
                break;

            case 9:
                // calc target rough position
                customerJudge();
                traget_pos_rough = transformCoordinateToWrist(point3d_obj_base[obj_num]);
                if(flag_customer_sound == 1)
                {
                    flag_ur3_disposal = 15;////////////////last 
                    flag_system_continue = 1;
                    flag_customer_sound = 0;
                }
                else
                    flag_ur3_disposal = 10;
                break;

            case 10:
                // move to rough position
                customerJudge();
                moveitRoughPosition(traget_pos_rough);
                
                if(flag_customer_sound == 1)
                {
                    flag_ur3_disposal = 15;////////////////last 
                    flag_system_continue = 1;
                    flag_customer_sound = 0;
                }
                else
                    flag_ur3_disposal = 11;
                break;

            case 11:
                // pricision
                customerJudge();
                img_output = img_src_color.clone();
                PrecisionPrepare();

                if(flag_customer_sound == 1)
                {
                    flag_ur3_disposal = 15;////////////////last 
                    flag_system_continue = 1;
                    flag_customer_sound = 0;
                }
                else
                    flag_ur3_disposal = 12;
                break;

            case 12:
                // final
                customerJudge();
                moveitFinalPosition(wrist_rotation);
                // moveitFinalPosition2();

                if(flag_customer_sound == 1)
                {
                    flag_ur3_disposal = 15;////////////////last 
                    flag_system_continue = 1;
                    flag_customer_sound = 0;
                }
                else
                    flag_ur3_disposal = 13;
                break;

            case 13:
                // gripper
                customerJudge();
                ros::spinOnce();
                gripperControler(140);
                ros::Duration(2.0).sleep();

                if(flag_customer_sound == 1)
                {
                    flag_ur3_disposal = 15;////////////////last 
                    flag_system_continue = 1;
                    flag_customer_sound = 0;
                }
                else
                    flag_ur3_disposal = 14;
                break;

            case 14:
                // moveit disposal
                customerJudge();
                moveitDisposalPosition();

                ros::spinOnce();
                gripperControler(400);    
                ros::Duration(2.0).sleep();

            
                obj_num++;
                if (obj_num >= obj_total)
                {
                    flag_ur3_disposal = 15;  //////////////////////////////
                    cout << "flag_ur3_disposal: " << flag_ur3_disposal << endl;
                } 
                else
                    flag_ur3_disposal = 7;

                flag_system_continue = 0;

                // if customer detected
                if(flag_customer_sound == 1)
                {
                    flag_ur3_disposal = 15;////////////////last 
                    flag_system_continue = 1;
                    flag_customer_sound = 0;
                }
                break;

            case 15: // finish disposal task
                cout << "-------------- FINISH: Disposal Task ------------- " << endl << endl;
                flag_ur3_system = 3;
                break;
            
            default:
                break;
        }
    }

    waitKey(1);
}