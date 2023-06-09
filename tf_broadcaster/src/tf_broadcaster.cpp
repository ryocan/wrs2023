// tf_broadcaster.cpp
#include <ros/ros.h> 
#include <geometry_msgs/TransformStamped.h> 
#include <tf2_ros/static_transform_broadcaster.h> 
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 
 
class TfBroadcaster 
{ 
public: 
    TfBroadcaster(); 
    ~TfBroadcaster(); 
// Broadcast 
    void BroadcastStaticTfFromCameraToUR3(); 
 
private: 
    ros::NodeHandle nh_; 
// TF Broadcaster 
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_; 
// constant 
    double PI_ = 3.1419265; 
}; 
 
TfBroadcaster::TfBroadcaster(){} 
TfBroadcaster::~TfBroadcaster(){} 
 
void TfBroadcaster::BroadcastStaticTfFromCameraToUR3() 
{ 
    geometry_msgs::TransformStamped transformStamped; 
    transformStamped.header.stamp = ros::Time::now(); 
    // transformStamped.header.frame_id = "camera_color_optical_frame";
    // transformStamped.child_frame_id = "tool0";
    // transformStamped.header.frame_id = "camera_color_optical_frame";
    // transformStamped.child_frame_id = "base_link";
    transformStamped.header.frame_id = "wrist_3_link";
    transformStamped.child_frame_id = "camera_link";

    transformStamped.transform.translation.x = -0.0202343 + 0.0001350092643406242; 
    transformStamped.transform.translation.y = -0.0438682 - 0.014751131646335125; 
    transformStamped.transform.translation.z = -0.0164669 + 2.05004198505776e-05; 

    tf2::Quaternion q; 
    q.setEuler(-1.552130627, 1.546712127, 0.021618);   
    transformStamped.transform.rotation.x = q.x(); 
    transformStamped.transform.rotation.y = q.y(); 
    transformStamped.transform.rotation.z = q.z(); 
    transformStamped.transform.rotation.w = q.w(); 

    static_tf_broadcaster_.sendTransform(transformStamped);     
} 
 
int main(int argc, char** argv) 
{ 
    ros::init(argc, argv, "tf_broadcaster"); 
    TfBroadcaster tf_broadcaster; 
    tf_broadcaster.BroadcastStaticTfFromCameraToUR3(); 
 
    ros::Rate loop_rate(10); 
    while(ros::ok()){ 
        ros::spinOnce(); 
        loop_rate.sleep(); 
    } 
    return 0; 
} 