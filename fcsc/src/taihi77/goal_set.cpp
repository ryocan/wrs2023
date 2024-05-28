#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>


#define PI 3.14159265359
#define resolution 0.05;
#define origin_x 30*20
#define origin_y 30*20
//x->width  y->height
#define shelf1_x 650
#define shelf1_y 589
#define back_x 600
#define back_y 600
#define shelf2_x 640
#define shelf2_y 560

nav_msgs::Odometry rover_odom_recieve;
double roll, pitch, yaw;
void odom_receive(const nav_msgs::Odometry::ConstPtr& odom_msg){
  rover_odom_recieve = *odom_msg;

  tf::Quaternion q(rover_odom_recieve.pose.pose.orientation.x, rover_odom_recieve.pose.pose.orientation.y, rover_odom_recieve.pose.pose.orientation.z, rover_odom_recieve.pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
}


  //uint8 PENDING         = 0  
  //uint8 ACTIVE          = 1 
  //uint8 PREEMPTED       = 2
  //uint8 SUCCEEDED       = 3
  //uint8 ABORTED         = 4
  //uint8 REJECTED        = 5
  //uint8 PREEMPTING      = 6
  //uint8 RECALLING       = 7
  //uint8 RECALLED        = 8
  //uint8 LOST            = 9
int status_id = 0;
void status_receive(const actionlib_msgs::GoalStatusArray::ConstPtr& status_msg){
  if(!status_msg->status_list.empty()){
    int n_status = status_msg->status_list.size();
    status_id = status_msg->status_list[n_status - 1].status;
  }
}


ros::Publisher goal_pub;
void flag_receive(const std_msgs::String::ConstPtr& input_flag){
  std_msgs::String input_flagg = *input_flag;
  int flag = atoi(input_flagg.data.c_str());


  geometry_msgs::PoseStamped goal;
    nav_msgs::Odometry rover_odom = rover_odom_recieve;

  goal.header.stamp = ros::Time::now();
  goal.header.frame_id = "odom";


  // goal.header.stamp = ros::Time::now();
  // goal.header.frame_id = rover_odom.header.frame_id;

  double rad;
  // if(pmt_odom.pose.pose.orientation.z >= 0){
  //   rad = 2.0*acos(pmt_odom.pose.pose.orientation.w);
  // }
  // else{
  //   rad = -2.0*acos(pmt_odom.pose.pose.orientation.w);
  // }
  // if(fabs(rad) > PI){
  //   if(rad > 0){
  //     rad -= 2.0*PI;
  //   }
  //   else{
  //     rad += 2.0*PI;
  //   }
  // }

  if(flag == 0 && status_id != 1){
    goal.pose.position.x = (back_x - origin_x)*resolution;
    goal.pose.position.y = (back_y - origin_y)*resolution;
    goal.pose.orientation.z = 0;
    goal.pose.orientation.w = 1;
    // goal.pose.orientation.z = sin(rad/2.0);
    // goal.pose.orientation.w = cos(rad/2.0);
    goal_pub.publish(goal);
  }

  if(flag == 1 && status_id != 1){
    goal.pose.position.x = (shelf1_x - origin_x)*resolution;
    goal.pose.position.y = (shelf1_y - origin_y)*resolution;
    goal.pose.orientation.z = 0;
    goal.pose.orientation.w = 1;
    // goal.pose.orientation.z = sin(rad/2.0);
    // goal.pose.orientation.w = cos(rad/2.0);
    goal_pub.publish(goal);
  }

    if(flag == 2 && status_id != 1){
    goal.pose.position.x = (shelf2_x - origin_x)*resolution;
    goal.pose.position.y = (shelf2_y - origin_y)*resolution;
    goal.pose.orientation.z = 1;
    goal.pose.orientation.w = 0;
    // goal.pose.orientation.z = sin(rad/2.0);
    // goal.pose.orientation.w = cos(rad/2.0);
    goal_pub.publish(goal);
  }
  
}

int main(int argc, char** argv){
  ros::init(argc, argv, "goal_set");
  ros::NodeHandle nh;
  ros::Subscriber flag_sub = nh.subscribe("/flag", 1, flag_receive);
  ros::Subscriber status_sub = nh.subscribe("/move_base/status", 1, status_receive);
  ros::Subscriber odom_sub = nh.subscribe("/odom", 1, odom_receive);
  goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
  ros::spin();
  return 0;
}
