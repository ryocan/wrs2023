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


// // 本番用
// #define shelf_x 1.0
// #define shelf_y 2.55
// #define avo_x 1.75
// #define avo_y 1.2
// #define back_x -0.3
// #define back_y -0.1

// #define mid_x 1.75
// #define mid_y 1.2
// #define mid2_x 2.0
// #define mid2_y 2.55
// #define midavo_a_x 1.0 //回避場所に行く時の経由地点
// #define midavo_a_y 0  //回避場所に行く時の経由地点
// #define midavo_b_x 1.75   //回避場所から棚に戻る時の経由地点
// #define midavo_b_y 1.2   //回避場所から棚に戻る時の経由地点


// // p3
// #define shelf_x 1.0
// #define shelf_y 1.45
// #define avo_x 0
// #define avo_y 0
// #define back_x -0.3
// #define back_y 0

// #define mid_x 1.6
// #define mid_y 0.1
// #define mid2_x 1.6
// #define mid2_y 2.55
// #define midavo_a_x 2.00 //回避場所に行く時の経由地点
// #define midavo_a_y 2.55  //回避場所に行く時の経由地点
// #define midavo_b_x 1.75   //回避場所から棚に戻る時の経由地点
// #define midavo_b_y 1.2    //回避場所から棚に戻る時の経由地点



// //小島シミュレーション用
// #define shelf_x 4.019172286987305
// #define shelf_y -2.3886914253234863
// #define avo_x 5.519172286987305
// #define avo_y 0
// #define avo2_x -5.519172286987305
// #define avo2_y 0
// #define back_x 5.519172286987305
// #define back_y -2.3886914253234863

// #define mid_x 4.919172286987305
// #define mid_y -2.3886914253234863
// #define mid2_x 3.519172286987305
// #define mid2_y 0

//パドック
#define shelf_x 1.304511
#define shelf_y 1.430354
#define avo_x 1.564511
#define avo_y 0
#define back_x 0
#define back_y 0

#define mid_x 1.364511
#define mid_y 0
#define mid2_x 1.364511
#define mid2_y 0
#define midavo_a_x 1.364511 //回避場所に行く時の経由地点
#define midavo_a_y 0  //回避場所に行く時の経由地点
#define midavo_b_x 1.364511  //回避場所から棚に戻る時の経由地点
#define midavo_b_y 0   //回避場所から棚に戻る時の経由地点

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
int sta1 = 0;//意味ないフラグ
static int sta2 = 0;
static int sta3 = 0;
static int sta4 = 0;
static int sta5 = 0;
static int sta6 = 0; //inputが3きているかどうか判定


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



      // start
    if (input_flag == -1 && status_id == 0 && sta == -1)
    {
      output_flag.data = 0;
      sta = 7;
      flag_pub.publish(output_flag);
      ROS_INFO("1sta->%d  sta2->%d  sta3->%d  sta4->%d  sta5->%d",sta,sta1,sta2,sta3,sta4,sta5);
      ROS_INFO("ifbunn 1");
    }



  // reach mid
    if (input_flag == 0 && status_id == 3 && sta == 7)
    {
      output_flag.data = 0;
      sta = 0;
      flag_pub.publish(output_flag);
      ROS_INFO("2sta->%d  sta2->%d  sta3->%d  sta4->%d  sta5->%d",sta,sta1,sta2,sta3,sta4,sta5);
      ROS_INFO("ifbunn 2");
    }
    // reach shelf
    else if ((input_flag == 0 || input_flag == 2 || input_flag == 3) && status_id == 3 && sta == 0)
    {
      output_flag.data = 1;
      sta = 1;
      flag_pub.publish(output_flag);
      ROS_INFO("3sta->%d  sta2->%d  sta3->%d  sta4->%d  sta5->%d",sta,sta1,sta2,sta3,sta4,sta5);
      ROS_INFO("ifbunn 3");
    }
      // reach mid
    else if (input_flag == 1 && status_id == 3 && sta == 8)
    {
      output_flag.data = 0;
      sta = 4;
      flag_pub.publish(output_flag);
      ROS_INFO("4sta->%d  sta2->%d  sta3->%d  sta4->%d  sta5->%d",sta,sta1,sta2,sta3,sta4,sta5);
      ROS_INFO("ifbunn 4");
    }
      // reach back
    else if (input_flag == 1 && status_id == 3 && sta == 99)
    {
      output_flag.data = 0;
      sta = 100;
      flag_pub.publish(output_flag);
      ROS_INFO("5sta->%d  sta2->%d  sta3->%d  sta4->%d  sta5->%d",sta,sta1,sta2,sta3,sta4,sta5);
      ROS_INFO("ifbunn 5");
    }

     // chinretsu
    if (input_flag == 2 && status_id == 3 && sta == 1)
    {
      output_flag.data = 1;
      sta = 3;
      flag_pub.publish(output_flag);
      ROS_INFO("6sta->%d  sta2->%d  sta3->%d  sta4->%d  sta5->%d",sta,sta1,sta2,sta3,sta4,sta5);
      ROS_INFO("ifbunn 6");
    }


  if (avo_input_flag == 0)
  {

    sta3 = 0;
    sta5 = 0;

    // if(sta4 == 1)
    // {
    //   sta = 1;
    //   sta4 = 0;
    //   ROS_INFO("aaaaaaaaaaaaaaa");
    // }


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
        ROS_INFO("7sta->%d  sta2->%d  sta3->%d  sta4->%d  sta5->%d",sta,sta1,sta2,sta3,sta4,sta5);
        ROS_INFO("ifbunn 7");
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
        // ROS_INFO("go shelf");
        vo_flag = 1;
        sound_flag.data = vo_flag; // 音声
        sound_flag_pub.publish(sound_flag);
        goal_pub.publish(goal);
        flag_pub.publish(output_flag);
        ROS_INFO("8sta->%d  sta2->%d  sta3->%d  sta4->%d  sta5->%d",sta,sta1,sta2,sta3,sta4,sta5);
        ROS_INFO("ifbunn 8");
        sta2 = 2;
      }
    }
  }

    // go mid to go back
  if (input_flag == 1 && (status_id == 0 || status_id == 3) && sta != 100)
  {
    if (sta2 == 6)
    {
    }
    else
    {
      goal.pose.position.x = mid2_x;
      goal.pose.position.y = mid2_y;
      goal.pose.orientation.z = float(1/sqrt((double)2));
      goal.pose.orientation.w = float(1/sqrt((double)2));
      // goal.pose.orientation.z = 0;
      // goal.pose.orientation.w = 1;

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
      ROS_INFO("9sta->%d  sta2->%d  sta3->%d  sta4->%d  sta5->%d",sta,sta1,sta2,sta3,sta4,sta5);
      ROS_INFO("ifbunn 9");
      // ROS_INFO("go mid to go back");
      sta2 = 6;
    }
  }

  // go back
  else if (input_flag == 1 && (status_id == 0 || status_id == 3) && sta == 4)
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
      sta = 99;
      vo_flag = 1;
      sound_flag.data = vo_flag; // 音声
      goal_pub.publish(goal);
      flag_pub.publish(output_flag);
      ROS_INFO("10sta->%d  sta2->%d  sta3->%d  sta4->%d  sta5->%d",sta,sta1,sta2,sta3,sta4,sta5);
      ROS_INFO("ifbunn 10");
      // ROS_INFO("go back");
      sta2 = 7;
    }
  }


  ///////////////回避動作///////////////////
  //avo = 1, 2, 4, 3, 5, 6, 7 
 

  // go shelf during going avo//棚前に戻る時

  if(sta5 == 1)
  {
    

  }

  if (sta == 61 && avo_input_flag == 0)
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
      ROS_INFO("11sta->%d  sta2->%d  sta3->%d  sta4->%d  sta5->%d",sta,sta1,sta2,sta3,sta4,sta5);
      ROS_INFO("ifbunn 11");
      sta2 = 3;
      vo_flag = 1; // vo_flagwo 1 nisuruto goalsurumadenaru
      sound_flag.data = vo_flag;
    }
  }

    // reach midavo_b
  if (status_id == 3 && sta == 60)
  {
     output_flag.data = 0;
     sta = 61;
     flag_pub.publish(output_flag);
     ROS_INFO("13sta->%d  sta2->%d  sta3->%d  sta4->%d  sta5->%d",sta,sta1,sta2,sta3,sta4,sta5);
     ROS_INFO("ifbunn 13");
  }

  // go midavo_b during going avo//棚前に戻る時

  if(sta5 == 1)
  {
    

  }

  if (sta == 6 && avo_input_flag == 0)
  {
    if (sta2 == 60)
    {
    }
    else
    {
      goal.pose.position.x = midavo_b_x;
      goal.pose.position.y = midavo_b_y;
      goal.pose.orientation.z = float(1 / sqrt((double)2));
      goal.pose.orientation.w = float(1 / sqrt((double)2));
      // goal.pose.orientation.z = 0;
      // goal.pose.orientation.w = 1;

      output_flag.data = 0;
      sta = 60;
            if (sys_flag == 0)
      {
        system("cd ~/catkin_ws/src/fcsc/scripts ; python3 play.py");
        sys_flag = 1;
      }
      goal_pub.publish(goal);
      flag_pub.publish(output_flag);
      ROS_INFO("11sta->%d  sta2->%d  sta3->%d  sta4->%d  sta5->%d",sta,sta1,sta2,sta3,sta4,sta5);
      ROS_INFO("ifbunn 11");
      sta2 = 60;
      vo_flag = 1; // vo_flagwo 1 nisuruto goalsurumadenaru
      sound_flag.data = vo_flag;
    }
  }
  //  ROS_INFO("%d",sound_flag.data );




  //アームから動いていいフラグを受け取る
  if (avo_input_flag != 0)
  {
    if (input_flag == 3 && sta == 3)
    {
      output_flag.data = 0;
      sta = 5;
      sta3 = 1;
      flag_pub.publish(output_flag);
      ROS_INFO("12sta->%d  sta2->%d  sta3->%d  sta4->%d  sta5->%d",sta,sta1,sta2,sta3,sta4,sta5);
      ROS_INFO("ifbunn 12");
        // ROS_INFO("ccccccccccccccccccccccc");
    }
  }


  // reach avo
  if (status_id == 3 && sta == 51)
  {
     output_flag.data = 0;
     sta = 6;
     flag_pub.publish(output_flag);
     ROS_INFO("13sta->%d  sta2->%d  sta3->%d  sta4->%d  sta5->%d",sta,sta1,sta2,sta3,sta4,sta5);
     ROS_INFO("ifbunn 13");
  }

  //go avo
  if ((status_id == 0 || status_id == 3) && sta == 50)
      {
        if (sta2 == 50)
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
          sta = 51;
          sta5 = 2;
          vo_flag = 1;
          sound_flag.data = vo_flag; // 音声
          vo_flagg = 1;
          if (sys_flag == 0)
          {
            // system("cd ~/catkin_ws/src/fcsc/scripts ; python3 play.py");
            sys_flag = 1;
          }
          goal_pub.publish(goal);
          flag_pub.publish(output_flag);
          ROS_INFO("14sta->%d  sta2->%d  sta3->%d  sta4->%d  sta5->%d",sta,sta1,sta2,sta3,sta4,sta5);
          ROS_INFO("ifbunn 14");
          sta2 = 50;
        }
      }
      
      // reach midavo_a
  else if (status_id == 3 && sta == 9)
  {
     output_flag.data = 0;
     sta = 50;
     flag_pub.publish(output_flag);
     ROS_INFO("13sta->%d  sta2->%d  sta3->%d  sta4->%d  sta5->%d",sta,sta1,sta2,sta3,sta4,sta5);
     ROS_INFO("ifbunn 13");
  }



  if (avo_input_flag != 0 && sta3 == 1)
  {
    // if((avo_input_flag == 2 || avo_input_flag == 4 || avo_input_flag == 6) && avo_input_flag != 0){
    if(avo_input_flag != 1 && avo_input_flag != 0){
      if(sta5 == 1)
      {
        sta = 5;
        sta3 = 1;
        sta2 = -1;
      }


     // go midavo_a
     if ((status_id == 0 || status_id == 3) && sta == 5)
      {
        if (sta2 == 4)
       {
        }
        else
        {
          goal.pose.position.x = midavo_a_x;
          goal.pose.position.y = midavo_a_y;
          goal.pose.orientation.z = float(1 / sqrt((double)2));
          goal.pose.orientation.w = float(1 / sqrt((double)2));
          // goal.pose.orientation.z = 0;
          // goal.pose.orientation.w = 1;

          output_flag.data = 0;
          sta = 9;
          sta5 = 2;
          vo_flag = 1;
          sound_flag.data = vo_flag; // 音声
          vo_flagg = 1;
          if (sys_flag == 0)
          {
            system("cd ~/catkin_ws/src/fcsc/scripts ; python3 play3.py");//回避音声
            sys_flag = 1;
          }
          goal_pub.publish(goal);
          flag_pub.publish(output_flag);
          ROS_INFO("14sta->%d  sta2->%d  sta3->%d  sta4->%d  sta5->%d",sta,sta1,sta2,sta3,sta4,sta5);
          ROS_INFO("ifbunn 14");
          sta2 = 4;
        }
      }
    }

    // else if(avo_input_flag == 1 || avo_input_flag == 3 || avo_input_flag == 7) {
    // else if(avo_input_flag == 1 && (status_id == 0 || status_id == 3)) {
    //   system("cd ~/catkin_ws/src/fcsc/scripts ; python3 play2.py");
    //   sta = 6;
    //   sta4 = 1;
    //   sta5 = 1;
    // }
  }


  // ROS_INFO("goal_set_c sta: %d",sta);
  // ROS_INFO("avo_input_flag: %d",  avo_input_flag);
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