// 2023_07_08_最終版

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <actionlib_msgs/GoalStatusArray.h>

#include <std_msgs/Int8.h>

#include <unistd.h> //sleep関数用

// input_flag  -1->停止 0->棚前に行く 1->バックヤードに帰る 2->陳列中　3->回避してよい
// avo_flag  0->障害物なし 1->障害物あり
// output_flag 0->作業停止  1->作業可能 //7月８日　回避したい flagを削除
// sta -1->競技開始前 0->goalに着く前　1->goalに着いた 3-> 陳列中 4->陳列終了でスタート位置に戻る 5->回避中 6->回避する場所に到着  7->陳列に行くために中間地点についた 8->スタート位置に戻るために中間地点についた

// #define mid_x 0.8560725331306458
// #define mid_y 0.0
// #define shelf_x 0.8560725331306458
// #define shelf_y 1.4258326292037964

// #define shelf_x 1.2560725331306458
// #define shelf_y 1.9258326292037964
// #define avo_x 1.7380053997039795
// #define avo_y 1.461308240890503
// #define back_x  0.32024985551834106
// #define back_y 0.5682538747787476

// #define mid_x 1.4938474893569946
// #define mid_y 0.7122771739959717

#define shelf_x 1.1
#define shelf_y 2.55
#define avo_x 1.6
#define avo_y 2.55
#define back_x 0
#define back_y 0

#define mid_x 1.1
#define mid_y 0
#define mid2_x 1.45
#define mid2_y 3.3

ros::Publisher flag_pub;
ros::Publisher goal_pub;
ros::Publisher sound_flag_pub;
// out put//0->作業停止  1->作業可能 2->回避したい
static int input_flag = -1; //-1→停止 0->棚前に行く 1->バックヤードに帰る 2->陳列中　3->回避してよい
static int avo_input_flag = 0;
std_msgs::Int8 input_flaggg;
std_msgs::Int8 avo_input_flaggg;
std_msgs::Int8 output_flag;
static int sta = -1;
static int sta2 = 0;

static float goal_x = 0;
static float goal_y = 0;

void flag_receive(const std_msgs::Int8::ConstPtr &input_flagg)
{
  input_flaggg = *input_flagg;
  input_flag = input_flaggg.data;
}

// void avo_flag_receive(const std_msgs::Int8::ConstPtr& avo_input_flagg){
//   avo_input_flaggg = *avo_input_flagg;
//   avo_input_flag = avo_input_flaggg.data;
// }

void avo_flag_receive(const std_msgs::String::ConstPtr &avo_input_flagg)
{
  std_msgs::String avo_input_flaggg = *avo_input_flagg;
     // ROS_INFO("aaaaaaaaaaaaaaaaavo_input_flag: %d",  avo_input_flaggg.data);
  avo_input_flag = atoi(avo_input_flaggg.data.c_str());
    // ROS_INFO("bbbbbbbbbbbavo_input_flag: %d",  avo_input_flag);
}

nav_msgs::OccupancyGrid grid_map;
void map_receive(const nav_msgs::OccupancyGrid::ConstPtr &map)
{
  grid_map = *map;
}

nav_msgs::Odometry odom_msg;
double roll, pitch, yaw;
void odom_receive(const nav_msgs::Odometry::ConstPtr &rover_odom_recieve)
{
  // rover_odom_recieve = *odom_msg;
  odom_msg = *rover_odom_recieve;

  tf::Quaternion q(odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
}

int status_id = 0;
geometry_msgs::PoseStamped goal;

void status_receive(const actionlib_msgs::GoalStatusArray::ConstPtr &status_msg)
{
  if (!status_msg->status_list.empty())
  {
    int n_status = status_msg->status_list.size();
    status_id = status_msg->status_list[n_status - 1].status;
  }
  std_msgs::Int8 sound_flag;
  static int vo_flag = 0;
  static int vo_flagg = 0;
  static int sys_flag = 0;
  goal.header.stamp = ros::Time::now();
  goal.header.frame_id = "map";

  if (status_id == 3 || status_id == 0)
  {                      // roboが止まっているとき
    sound_flag.data = 0; // 音声停止
    sound_flag_pub.publish(sound_flag);
    vo_flag = 0;
    // ROS_INFO("aaaaaaaaaaaaaaaaaaa");
  }
  else
  { // roboが動いているとき
    sound_flag.data = vo_flag;
    sound_flag_pub.publish(sound_flag);
    // ROS_INFO("%d",sound_flag.data );
    sys_flag = 0;
  }

  // if (avo_input_flag != 7)
  // {
  //   avo_input_flag = 0;
  // }
  // else
  // {
  // }

  if (avo_input_flag == 0)
  {

    // start
    if (input_flag == -1 && status_id == 0 && sta == -1)
    {
      output_flag.data = 0;
      sta = 7;
      flag_pub.publish(output_flag);
    }

    // go mid to go shelf
    if (input_flag == 0 && (status_id == 0 || status_id == 3) && sta == 7)
    {
      if (sta2 == 1)
      {
      }
      else
      {
        goal.pose.position.x = mid_x;
        goal.pose.position.y = mid_y;
        goal.pose.orientation.z = float(1 / sqrt((double)2));
        goal.pose.orientation.w = float(1 / sqrt((double)2));

        output_flag.data = 0;
        sta = 7;

        vo_flag = 1;
        sound_flag.data = vo_flag; // 音声kaeri
        flag_pub.publish(output_flag);
        if (sys_flag == 0)
        {
          system("cd ~/catkin_ws/src/fcsc/scripts ; python3 play.py");
          sys_flag = 1;
        }
        sound_flag_pub.publish(sound_flag);
        // ROS_INFO("%d", sound_flag.data);
        goal_pub.publish(goal);
        sta2 = 1;
        usleep(100000);
      }
    }

    // reach mid
    if (input_flag == 0 && status_id == 3 && sta == 7)
    {
      output_flag.data = 0;
      sta = 0;
      flag_pub.publish(output_flag);
    }
    // reach shelf
    else if (input_flag == 0 && status_id == 3 && sta == 0)
    {
      output_flag.data = 1;
      sta = 1;
      flag_pub.publish(output_flag);
      ROS_INFO("REACH SHELF AND PUB MOBILEROBOT FLAG 1\n");
    }

    // go shelf
    if (input_flag == 0 && (status_id == 0 || status_id == 3) && sta == 0)
    {
      if (sta2 == 2)
      {
      }
      else
      {
        goal.pose.position.x = shelf_x;
        goal.pose.position.y = shelf_y;
        goal.pose.orientation.z = float(1 / sqrt((double)2));
        goal.pose.orientation.w = float(1 / sqrt((double)2));
        // goal.pose.orientation.z = 0;
        // goal.pose.orientation.w = 1;

        output_flag.data = 0;
        sta = 0;
        ROS_INFO("go shelf");
        vo_flag = 1;
        sound_flag.data = vo_flag; // 音声
        sound_flag_pub.publish(sound_flag);
        goal_pub.publish(goal);
        flag_pub.publish(output_flag);
        sta2 = 2;
      }
    }

    // chinretsu
    if (input_flag == 2 && status_id == 3 && sta == 1)
    {
      output_flag.data = 1;
      sta = 3;
      flag_pub.publish(output_flag);
    }
  }

  // go shelf during going avo
  if (sta == 6)
  {
    if (sta2 == 3)
    {
    }
    else
    {
      goal.pose.position.x = shelf_x;
      goal.pose.position.y = shelf_y;
      goal.pose.orientation.z = float(1 / sqrt((double)2));
      goal.pose.orientation.w = float(1 / sqrt((double)2));
      // goal.pose.orientation.z = 0;
      // goal.pose.orientation.w = 1;

      output_flag.data = 0;
      sta = 0;
      goal_pub.publish(goal);
      flag_pub.publish(output_flag);
      sta2 = 3;
      vo_flag = 1; // vo_flagwo 1 nisuruto goalsurumadenaru
      sound_flag.data = vo_flag;
    }
  }
  //  ROS_INFO("%d",sound_flag.data );




  if (avo_input_flag == 1 || avo_input_flag == 5)
  {
    ROS_INFO("ffffffffffffffff");
    // kaihi recuest
    // if (sta == 3)
    // {
    //   output_flag.data = 1;
    //   sta = 3;
    //   flag_pub.publish(output_flag);
    // }

    if (input_flag == 3 && sta == 3)
    {
      output_flag.data = 0;
      sta = 5;
      flag_pub.publish(output_flag);
        ROS_INFO("ccccccccccccccccccccccc");
    }



    // reach avo
    if (status_id == 3 && sta == 9)
    {
      output_flag.data = 0;
      sta = 6;
      flag_pub.publish(output_flag);
    }

    // go avo
    if ((status_id == 0 || status_id == 3) && sta == 5)
    {
      if (sta2 == 4)
      {
      }
      else
      {
        goal.pose.position.x = avo_x;
        goal.pose.position.y = avo_y;
        goal.pose.orientation.z = float(1 / sqrt((double)2));
        goal.pose.orientation.w = float(1 / sqrt((double)2));
        // goal.pose.orientation.z = 0;
        // goal.pose.orientation.w = 1;

        output_flag.data = 0;
        sta = 9;
        vo_flag = 1;
        sound_flag.data = vo_flag; // 音声
        vo_flagg = 1;
        if (sys_flag == 0)
        {
          system("cd ~/catkin_ws/src/fcsc/scripts ; python3 play.py");
          sys_flag = 1;
        }
        goal_pub.publish(goal);
        flag_pub.publish(output_flag);
        sta2 = 4;
      }
    }

    // go shelf from avo
    // if((status_id == 0 || status_id == 3) && sta == 6){
    //   if(sta2 == 5){

    //   }
    //   else{
    //     goal.pose.position.x = shelf_x;
    //     goal.pose.position.y = shelf_y;
    //     goal.pose.orientation.z = float(1/sqrt((double)2));
    //     goal.pose.orientation.w = float(1/sqrt((double)2));
    //     // goal.pose.orientation.z = 0;
    //     // goal.pose.orientation.w = 1;

    //     output_flag.data = 0;
    //     sta = 6;
    //     vo_flag = 1;
    //     sound_flag.data = vo_flag; //音声
    //     vo_flagg = 1;
    //     if(sys_flag==0){
    //       system("cd ~/catkin_ws/src/fcsc/scripts ; python3 play.py");
    //       sys_flag = 1;
    //     }
    //     goal_pub.publish(goal);
    //     flag_pub.publish(output_flag);
    //     sta2 = 5;
    //   }
    // }

    //   //reach shelf
    // if(status_id == 3 && sta == 6){
    //   output_flag.data = 1;
    //   sta = 1;
    //   flag_pub.publish(output_flag);
    // }
  }

  else if (avo_input_flag == 2 || avo_input_flag == 3 || avo_input_flag == 6 || avo_input_flag == 7)
  {

    // どいてください音声
    system("cd ~/catkin_ws/src/fcsc/scripts ; python3 play2.py");

    output_flag.data = 0;
    sta = 10;
    flag_pub.publish(output_flag);
  }

  if(avo_input_flag == 0 && sta == 10)
  {
      sta = 6;
  }

  // reach mid
  if (input_flag == 1 && status_id == 3 && sta == 8)
  {
    output_flag.data = 0;
    sta = 4;
    flag_pub.publish(output_flag);
  }

  // go mid to go back
  if (input_flag == 1 && (status_id == 0 || status_id == 3) && sta != 4 && sta != 0 && sta != 8)
  {
    if (sta2 == 6)
    {
    }
    else
    {
      goal.pose.position.x = mid2_x;
      goal.pose.position.y = mid2_y;
      // goal.pose.orientation.z = float(1/sqrt((double)2));
      // goal.pose.orientation.w = float(1/sqrt((double)2));
      goal.pose.orientation.z = 0;
      goal.pose.orientation.w = 1;

      output_flag.data = 0;
      sta = 8;
      vo_flag = 1;
      sound_flag.data = vo_flag; // 音声
      vo_flagg = 1;
      if (sys_flag == 0)
      {
        system("cd ~/catkin_ws/src/fcsc/scripts ; python3 play.py");
        sys_flag = 1;
      }
      goal_pub.publish(goal);
      flag_pub.publish(output_flag);
      // ROS_INFO("go mid to go back");
      sta2 = 6;
    }
  }

  // go back
  if (input_flag == 1 && (status_id == 0 || status_id == 3) && sta == 4)
  {
    if (sta2 == 7)
    {
    }
    else
    {
      goal.pose.position.x = back_x;
      goal.pose.position.y = back_y;
      // goal.pose.orientation.z = float(1/sqrt((double)2));
      // goal.pose.orientation.w = float(1/sqrt((double)2));
      goal.pose.orientation.z = 0;
      goal.pose.orientation.w = 1;

      output_flag.data = 0;
      sta = 4;
      vo_flag = 1;
      sound_flag.data = vo_flag; // 音声
      goal_pub.publish(goal);
      flag_pub.publish(output_flag);
      // ROS_INFO("go back");
      sta2 = 7;
    }
  }

  // reach back
  if (input_flag == 1 && status_id == 3 && sta == 4)
  {
    output_flag.data = 0;
    sta = 0;
    flag_pub.publish(output_flag);
  }

  ROS_INFO("goal_set_c sta: %d",sta);
  ROS_INFO("avo_input_flag: %d",  avo_input_flag);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "goal_set");
  ros::NodeHandle nh;
  ros::Subscriber map_sub = nh.subscribe("/map", 1, map_receive);
  ros::Subscriber odom_sub = nh.subscribe("/odom", 1, odom_receive);
  ros::Subscriber flag_sub = nh.subscribe("/input_flag", 1, flag_receive);
  ros::Subscriber avo_flag_sub = nh.subscribe("/flag", 1, avo_flag_receive);
  // ros::Subscriber fin_sub = nh.subscribe("/fin_data", 1, fin_receive);
  sound_flag_pub = nh.advertise<std_msgs::Int8>("/sound_flag", 1);
  ros::Subscriber status_sub = nh.subscribe("/move_base/status", 1, status_receive);
  flag_pub = nh.advertise<std_msgs::Int8>("/flag_mobile_robot", 1);
  goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
  ros::spin();
  return 0;
}
