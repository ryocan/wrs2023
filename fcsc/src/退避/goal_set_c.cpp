#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <fcsc/Fin.h>

#include <std_msgs/Int8.h>

//input_flag  0->陳列前  1->陳列中  2->陳列後
//input_flag  -1->停止 0->棚前に行く  1->陳列中  2->バックヤードに帰る
//output_flag 0->作業停止  1->作業可能

#define resolution 0.05;
#define origin_x 30*20
#define origin_y 30*20
//x->width  y->height
#define shelf_x 642
#define shelf_y 589
#define back_x 600
#define back_y 600

//障害物によって回避を始める範囲(x->width  y->height)
#define min_map_x 622
#define max_map_x 662
#define min_map_y 589
#define max_map_y 699

//障害物の大きさの閾値[m]
#define min_size 0.25
#define max_size 0.4


static  int avoid_flag; //avoid_flga 0=回避していない 1=回避中 2=回避完了 
static  int output_flag;//output_flag 0->作業停止  1->作業可能

ros::Publisher flag_pub;
ros::Publisher goal_pub;

nav_msgs::Odometry rover_odom_recieve;
double roll, pitch, yaw;
void odom_receive(const nav_msgs::Odometry::ConstPtr& odom_msg){
  rover_odom_recieve = *odom_msg;

  tf::Quaternion q(rover_odom_recieve.pose.pose.orientation.x, rover_odom_recieve.pose.pose.orientation.y, rover_odom_recieve.pose.pose.orientation.z, rover_odom_recieve.pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
}

nav_msgs::OccupancyGrid grid_map;
void map_receive(const nav_msgs::OccupancyGrid::ConstPtr& map){
  grid_map = *map;
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



int input_flag;//input_flag  0->棚前に行く  1->陳列中  2->バックヤードに帰る
std_msgs::Int8 input_flagg;
void flag_receive(const std_msgs::Int8::ConstPtr& input_flaggg){
  input_flagg = *input_flaggg;
}



fcsc::Fin fin;
void fin_receive(const fcsc::Fin::ConstPtr& input_fin){
  fin = *input_fin;

  geometry_msgs::PoseStamped goal;
  nav_msgs::Odometry rover_odom = rover_odom_recieve;
  
  float fin_globalmap_x[fin.num+1], fin_globalmap_y[fin.num+1];


  input_flag = input_flagg.data;

  goal.header.stamp = ros::Time::now();
  goal.header.frame_id = "odom";

  if(input_flag == -1){

  }
  else{
    if(fin.num != 0){
      for(int bb = 0; bb < fin.num ; bb++){
        fin_globalmap_x[bb] = (float)(rover_odom.pose.pose.position.x + fin.X_point[bb]*cos(yaw) - fin.Y_point[bb]*sin(yaw));
        fin_globalmap_y[bb] = (float)(rover_odom.pose.pose.position.y + fin.X_point[bb]*sin(yaw) + fin.Y_point[bb]*cos(yaw));
ROS_INFO("yaw=%lf  id=%d  x=%f  y=%f  size=%f",yaw,bb,fin_globalmap_x[bb] ,fin_globalmap_y[bb],fin.size[bb] );
      }
    }


  //陳列に行く処理開始
    if(output_flag == 0 && input_flag == 0 && avoid_flag == 0 && status_id != 1){
    //set_goal　tana
      goal.pose.position.x = (shelf_x - origin_x)*resolution;
      goal.pose.position.y = (shelf_y - origin_y)*resolution;
      goal.pose.orientation.z = float(1/sqrt((double)2));
      goal.pose.orientation.w = float(1/sqrt((double)2));
      goal_pub.publish(goal);

    //output_flag 0
      output_flag = 0;
      avoid_flag = 0;
    }
    else if(input_flag == 0 && status_id == 3){
    //output_flag 1(armにinput_flag = 1にしてもらう)
      output_flag = 1;
      input_flag = 1;
      avoid_flag = 0;
    }
  //陳列に行く処理終了

  //陳列中開始
    if(output_flag == 1 && input_flag == 1 && avoid_flag == 0){
      if(fin.num != 0){
        for(int a = 0 ; a < fin.num ; a++){
          if(min_map_x <= fin_globalmap_x[a] && fin_globalmap_x[a] <= max_map_x && min_map_y <= fin_globalmap_y[a]  && fin_globalmap_y[a] <= max_map_y){
          }
       //後ろに下がる処理
          else if(status_id != 1){
            goal.pose.position.x = rover_odom.pose.pose.position.x;
            goal.pose.position.y = (shelf_x - origin_x)*resolution + 1;
            goal.pose.orientation.z = float(1/sqrt((double)2));
            goal.pose.orientation.w = float(1/sqrt((double)2));
            goal_pub.publish(goal);

            output_flag = 0;
            input_flag = 1;
            avoid_flag = 1;

            break;
          }
        }
      } 
      else{
      }
    }
  //陳列中終了

  //回避開始
    if(output_flag == 0 && input_flag == 1 && avoid_flag == 1){
      if(fin.num != 0){
        for(int aa = 0 ; aa < fin.num ; aa++){
          if(min_map_x <= fin_globalmap_x[aa] && fin_globalmap_x[aa] <= max_map_x && min_map_y <= fin_globalmap_y[aa]  && fin_globalmap_y[aa] <= max_map_y){
            output_flag = 0;
            input_flag = 0;
            avoid_flag = 0;
          }
       //後ろに下がる処理
          else if(status_id != 1){
            goal.pose.position.x = rover_odom.pose.pose.position.x;
            goal.pose.position.y = (shelf_x - origin_x)*resolution + 1;
            goal.pose.orientation.z = float(1/sqrt((double)2));
            goal.pose.orientation.w = float(1/sqrt((double)2));
            goal_pub.publish(goal);

            output_flag = 0;
            input_flag = 1;
            avoid_flag = 1;

            break;
          }
        }
      } 
      else{
        output_flag = 0;
        input_flag = 0;
        avoid_flag = 0;
      }
    }
  //回避終了

  //帰る開始
    if(input_flag == 2 && status_id != 1){
      goal.pose.position.x = (back_x - origin_x)*resolution;
      goal.pose.position.y = (back_y - origin_y)*resolution;
      goal.pose.orientation.z = 0;
      goal.pose.orientation.w = 1;
      goal_pub.publish(goal);
    } 
  //帰る終了


    std_msgs::Int8 pub_flag;
    pub_flag.data = output_flag;

    flag_pub.publish(pub_flag);

  }
}


int main(int argc, char** argv){
  ros::init(argc, argv, "goal_set");
  ros::NodeHandle nh;
  ros::Subscriber map_sub = nh.subscribe("/map", 1, map_receive);
  ros::Subscriber odom_sub = nh.subscribe("/odom", 1, odom_receive);
  ros::Subscriber status_sub = nh.subscribe("/move_base/status", 1, status_receive);
  ros::Subscriber flag_sub = nh.subscribe("/input_flag", 1, flag_receive);
  ros::Subscriber fin_sub = nh.subscribe("/fin_data", 1, fin_receive);

  flag_pub = nh.advertise<std_msgs::Int8>("/output_flag", 1);
  goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
  ros::spin();
  return 0;
}







// if(fin.num == 0 || (min_size <= fin.size && fin.size <= max_size) || (min_map_x <= fin_globalmap_x && fin_globalmap_x <= max_map_x && min_map_y <= fin_globalmap_y  && fin_globalmap_x <= max_map_y)){
    //   output_flag = 0;
    //   input_flag = 0;
    //   avoid_flag = 0;
    // }
    // //後ろに下がる処理
    // else{
    //   goal.pose.orientation.x = rover_odom.pose.pose.position.x;
    //   goal.pose.orientation.y = (shelf_x - origin_x)*resolution + 1;
    //   goal.pose.orientation.z = rover_odom.pose.pose.orientation.z;
    //   goal.pose.orientation.w = rover_odom.pose.pose.orientation.w;

    //   output_flag = 0;
    //   input_flag = 1;
    //   avoid_flag = 1;
    // } 




  //   if(fin.num == 0){
  //     if(flag_avoid == 0){
  //       output_flag = 1;
  //     }
  //   }
  //   else{
  //     fin_globalmap_x = (float)(rover_odom.pose.pose.position.x + fin.X_point*cos(yaw) - fin.Y_point*sin(yaw));
  //     fin_globalmap_y = (float)(rover_odom.pose.pose.position.y + fin.X_point*sin(yaw) + fin.Y_point*cos(yaw));
  //     if(min_map_x <= fin_globalmap_x && fin_globalmap_x <= max_map_x && min_map_y <= fin_globalmap_y  && fin_globalmap_x <= max_map_y){
  //       if(min_size <= fin.size && fin.size <= max_size){
  //         output_flag = 0;
  //         // if(fin_globalmap_x < 0){
  //           // robotoはx+に移動 kaihi
  //           goal.pose.orientation.x = rover_odom.pose.pose.position.x;
  //           goal.pose.orientation.y = (shelf_x - origin_x)*resolution + 1;
  //           goal.pose.orientation.z = rover_odom.pose.pose.orientation.z;
  //           goal.pose.orientation.w = rover_odom.pose.pose.orientation.w;
  //           //回避flag ON
  //           flag_avoid = 1;
  //         // }
  //         // else if(0 < fin_map_x){
  //         //   //robotoはx-に移動 kaihi
  //         //   goal.pose.orientation.x = (shelf_x - origin_x)*resolution - 1.5;
  //         //   goal.pose.orientation.y = rover_odom.pose.pose.position.y;
  //         //   goal.pose.orientation.z = rover_odom.pose.pose.orientation.z;
  //         //   goal.pose.orientation.w = rover_odom.pose.pose.orientation.w;
  //         //   //回避flag ON
  //         //   flag_avoid = 1;
  //         // }
  //       }
  //       else{
  //         if(flag_avoid == 0){
  //           output_flag = 1;
  //         }
  //         if(status_id == 3){
  //           回避flag OFF
  //           output_flag 1;
  //         }
  //       }
  //     }
  //     else{
  //       if(回避flag ON){
  //         //set_goal tana
  //         if(status_id == 3){
  //           回避flag OFF
  //           output_flag 1;
  //         }
  //       }
  //     }  
  //   }
  // }
  // else if(input_flag == 2){
  //   output_flag = 0;
  //   // set_goal back
  // }