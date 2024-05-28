#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>


#define PI 3.14159265359
// #define resolution 0.05;
// #define origin_x 30*20
// #define origin_y 30*20
//x->width  y->height
#define shelf1_x 650
#define shelf1_y 589
#define shelf1_avo_x 650
#define shelf1_avo_y 589
#define shelf2_x 639
#define shelf2_y 556
#define shelf2_avo_x 639
#define shelf2_avo_y 556
#define back_x 600
#define back_y 600

static int sta = 0; //0->not陳列  1->陳列中


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
ros::Publisher sound_flag_pub;
void flag_receive(const std_msgs::String::ConstPtr& input_flag){
  std_msgs::String input_flagg = *input_flag;
  int flag = atoi(input_flagg.data.c_str());
  std_msgs::Int8  sound_flag;

  geometry_msgs::PoseStamped goal;

  goal.header.stamp = ros::Time::now();
  goal.header.frame_id = "map";

static int vo_flag = 0;//0→音は止まらない　1→音を止めることが許されている
static int vo_flagg;
  double rad;

    if (status_id==3 || status_id==0){//roboが止まっているとき
  sound_flag.data = 0; //音声停止
  sound_flag_pub.publish(sound_flag);
  vo_flag = 0;
 }
 else{//roboが動いているとき
  sound_flag.data = vo_flagg;
  sound_flag_pub.publish(sound_flag);
 }

  if(flag == 0 && status_id != 1){
    goal.pose.position.x = back_x;
    goal.pose.position.y = back_y;
    goal.pose.orientation.z = 0;
    goal.pose.orientation.w = 1;
    vo_flagg =2;
    sound_flag.data = vo_flagg; //音声kaeri
    system("cd ~/catkin_ws/src/fcsc/scripts ; python3 play.py");
    sound_flag_pub.publish(sound_flag);
    goal_pub.publish(goal);
    vo_flag = 1;
  }

  if(flag == 1 && status_id != 1 && sta == 0){
    goal.pose.position.x = shelf1_x;
    goal.pose.position.y = shelf1_y;
    goal.pose.orientation.z = 0;
    goal.pose.orientation.w = 1;
    vo_flagg = 1;
    sound_flag.data = vo_flagg; //音声iki
    system("cd ~/catkin_ws/src/fcsc/scripts ; python3 play.py");
    sound_flag_pub.publish(sound_flag);
    goal_pub.publish(goal);
    vo_flag = 1;
  }
  if(flag == 1 && sta == 1){
    goal.pose.position.x = shelf1_avo_x;
    goal.pose.position.y = shelf1_avo_y;
    goal.pose.orientation.z = 0;
    goal.pose.orientation.w = 1;
    vo_flagg = 1;
    sound_flag.data =vo_flagg; //音声iki
    system("cd ~/catkin_ws/src/fcsc/scripts ; python3 play.py");
    sound_flag_pub.publish(sound_flag);
    goal_pub.publish(goal);
    vo_flag = 1;
  }

    if(flag == 2 && status_id != 1 && sta == 0){
    goal.pose.position.x = shelf2_x;
    goal.pose.position.y = shelf2_y;
    goal.pose.orientation.z = 1;
    goal.pose.orientation.w = 0;
    vo_flagg = 1;
    sound_flag.data = vo_flagg; //音声iki
    system("cd ~/catkin_ws/src/fcsc/scripts ; python3 play.py");
    sound_flag_pub.publish(sound_flag);
    goal_pub.publish(goal);
    vo_flag = 1;
  }
  
  if(flag == 1 && sta == 1){
    goal.pose.position.x = shelf2_avo_x;
    goal.pose.position.y = shelf2_avo_y;
    goal.pose.orientation.z = 0;
    goal.pose.orientation.w = 1;
    vo_flagg = 1;
    sound_flag.data = vo_flagg; //音声iki
    system("cd ~/catkin_ws/src/fcsc/scripts ; python3 play.py");
    sound_flag_pub.publish(sound_flag);
    goal_pub.publish(goal);
    vo_flag = 1;
  }

  
}

int main(int argc, char** argv){
  ros::init(argc, argv, "goal_set");
  ros::NodeHandle nh;
  ros::Subscriber flag_sub = nh.subscribe("/flag", 1, flag_receive);
  ros::Subscriber status_sub = nh.subscribe("/move_base/status", 1, status_receive);
  ros::Subscriber odom_sub = nh.subscribe("/odom", 1, odom_receive);
  goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
  sound_flag_pub = nh.advertise<std_msgs::Int8>("/sound_flag", 1);
  ros::spin();
  return 0;
}
