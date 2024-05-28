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

//input_flag  -1->停止 0->棚前に行く 1->バックヤードに帰る 2->陳列中　3->回避してよい
//avo_flag  0->障害物なし 1->障害物あり
//output_flag 0->作業停止  1->作業可能 2->回避したい
//sta 0->goalに着く前　1->goalに着いた 3-> 陳列中 4->陳列終了でスタート位置に戻る 5->回避中 6->回避する場所に到着 


#define shelf_x 0.7560725331306458
#define shelf_y 1.4258326292037964
#define avo_x 1.7380053997039795
#define avo_y 1.461308240890503
#define back_x -0.655661404132843
#define back_y -0.005593299865722656


ros::Publisher flag_pub;
ros::Publisher goal_pub;

static int input_flag = -1;
std_msgs::Int8 input_flaggg;
std_msgs::Int8 output_flag;
int avoid_flag = 0;
static int sta;
static float goal_x = 0;
static float goal_y = 0;



void flag_receive(const std_msgs::Int8::ConstPtr& input_flagg){
  input_flaggg = *input_flagg;
  input_flag = input_flaggg.data;
}

nav_msgs::OccupancyGrid grid_map;
void map_receive(const nav_msgs::OccupancyGrid::ConstPtr& map){
  grid_map = *map;
}


nav_msgs::Odometry odom_msg;
double roll, pitch, yaw;
void odom_receive(const nav_msgs::Odometry::ConstPtr& rover_odom_recieve){
  // rover_odom_recieve = *odom_msg;
  odom_msg = *rover_odom_recieve;

  tf::Quaternion q(odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
}


int status_id = 0;
geometry_msgs::PoseStamped goal;

void status_receive(const actionlib_msgs::GoalStatusArray::ConstPtr& status_msg){
  if(!status_msg->status_list.empty()){
    int n_status = status_msg->status_list.size();
    status_id = status_msg->status_list[n_status - 1].status;
  }

  goal.header.stamp = ros::Time::now();
  goal.header.frame_id = "map";

  if(avoid_flag == 0){
    //start
    if(input_flag == -1 && status_id == 0 && sta == 0){
      output_flag.data = 0;
      sta = 0;
      flag_pub.publish(output_flag);
    }

    //go shelf
    if(input_flag == 0 && status_id == 0 && sta == 0){
      goal.pose.position.x = shelf_x;
      goal.pose.position.y = shelf_y;
      goal.pose.orientation.z = float(1/sqrt((double)2));
      goal.pose.orientation.w = float(1/sqrt((double)2));
      // goal.pose.orientation.z = 0;
      // goal.pose.orientation.w = 1;

      output_flag.data = 0;
      sta = 0;
      goal_pub.publish(goal);
      flag_pub.publish(output_flag);
    }

    //reach shelf
    if(input_flag == 0 && status_id == 3 && sta == 0){
      output_flag.data = 1;
      sta = 1;
      flag_pub.publish(output_flag);
    }

    //chinretsu
    if(input_flag == 2 && status_id == 0 && sta == 1){
      output_flag.data = 1;
      sta = 3;
      flag_pub.publish(output_flag);
    }

    //go shelf during going back  
    if(sta == 6){
      goal.pose.position.x = shelf_x;
      goal.pose.position.y = shelf_y;
      goal.pose.orientation.z = float(1/sqrt((double)2));
      goal.pose.orientation.w = float(1/sqrt((double)2));
      // goal.pose.orientation.z = 0;
      // goal.pose.orientation.w = 1;

      output_flag.data = 0;
      sta = 0;
      goal_pub.publish(goal);
      flag_pub.publish(output_flag);
    }

  }

  else if(avoid_flag == 1){
    //kaihi recuest
    if(sta = 3){
      output_flag.data = 2;
      sta = 3;
      flag_pub.publish(output_flag);
    }

    if(input_flag == 3 && sta == 3){
      output_flag.data = 0;
      sta = 5;
      flag_pub.publish(output_flag);
    }

    //go back
    if(status_id == 0 && sta == 5){
      goal.pose.position.x = avo_x;
      goal.pose.position.y = avo_y;
      goal.pose.orientation.z = float(1/sqrt((double)2));
      goal.pose.orientation.w = float(1/sqrt((double)2));
      // goal.pose.orientation.z = 0;
      // goal.pose.orientation.w = 1;

      output_flag.data = 0;
      sta = 5;
      goal_pub.publish(goal);
      flag_pub.publish(output_flag);
    }

    //reach kaihi
    if(status_id == 3 && sta == 5){
      output_flag.data = 0;
      sta = 6;
      flag_pub.publish(output_flag);
    }

    //go shelf from back
    if(status_id == 0 && sta == 6){
      goal.pose.position.x = shelf_x;
      goal.pose.position.y = shelf_y;
      goal.pose.orientation.z = float(1/sqrt((double)2));
      goal.pose.orientation.w = float(1/sqrt((double)2));
      // goal.pose.orientation.z = 0;
      // goal.pose.orientation.w = 1;

      output_flag.data = 0;
      sta = 6;
      goal_pub.publish(goal);
      flag_pub.publish(output_flag);
    }

    //reach shelf
    if(status_id == 3 && sta == 6){
      output_flag.data = 1;
      sta = 1;
      flag_pub.publish(output_flag);
    }
  }

  //go back
  if(input_flag == 1 && status_id == 0){
    goal.pose.position.x = back_x;
    goal.pose.position.y = back_y;
      // goal.pose.orientation.z = float(1/sqrt((double)2));
      // goal.pose.orientation.w = float(1/sqrt((double)2));
    goal.pose.orientation.z = 0;
    goal.pose.orientation.w = 1;

    output_flag.data = 0;
    sta = 4;
    goal_pub.publish(goal);
    flag_pub.publish(output_flag);
  }

    //reach back
  if(input_flag == 1 && status_id == 3 && sta == 4){
    output_flag.data = 0;
    sta = 4;
    flag_pub.publish(output_flag);
  }  
}


int main(int argc, char** argv){
  ros::init(argc, argv, "goal_set");
  ros::NodeHandle nh;
  ros::Subscriber map_sub = nh.subscribe("/map", 1, map_receive);
  ros::Subscriber odom_sub = nh.subscribe("/odom", 1, odom_receive);
  ros::Subscriber status_sub = nh.subscribe("/move_base/status", 1, status_receive);
  ros::Subscriber flag_sub = nh.subscribe("/input_flag", 1, flag_receive);
  // ros::Subscriber fin_sub = nh.subscribe("/fin_data", 1, fin_receive);

  flag_pub = nh.advertise<std_msgs::Int8>("/output_flag", 1);
  goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
  ros::spin();
  return 0;
}





  //uint8 PENDING         = 0  スタートの時
  //uint8 ACTIVE          = 1 
  //uint8 PREEMPTED       = 2　経路考えているとき
  //uint8 SUCCEEDED       = 3　ゴールについたら３から０
  //uint8 ABORTED         = 4
  //uint8 REJECTED        = 5
  //uint8 PREEMPTING      = 6
  //uint8 RECALLING       = 7
  //uint8 RECALLED        = 8
  //uint8 LOST            = 9




// static  int avoid_flag; //avoid_flga 0=回避していない 1=回避中 2=回避完了 
// static  int output_flag;//output_flag 0->作業停止  1->作業可能
// static  int fflag = 0;



// ros::Publisher flag_pub;
// ros::Publisher goal_pub;

// void flag_receive(const std_msgs::Int8::ConstPtr& input_flag){
//   input_flag = *input_flagg;
// }

// nav_msgs::OccupancyGrid grid_map;
// void map_receive(const nav_msgs::OccupancyGrid::ConstPtr& map){
//   grid_map = *map;
// }


//   //uint8 PENDING         = 0  スタートの時
//   //uint8 ACTIVE          = 1 
//   //uint8 PREEMPTED       = 2　経路考えているとき
//   //uint8 SUCCEEDED       = 3　ゴールについたら３から０
//   //uint8 ABORTED         = 4
//   //uint8 REJECTED        = 5
//   //uint8 PREEMPTING      = 6
//   //uint8 RECALLING       = 7
//   //uint8 RECALLED        = 8
//   //uint8 LOST            = 9
// int status_id = 0;
// void status_receive(const actionlib_msgs::GoalStatusArray::ConstPtr& status_msg){
//   if(!status_msg->status_list.empty()){
//     int n_status = status_msg->status_list.size();
//     status_id = status_msg->status_list[n_status - 1].status;
//   }
// }



// int input_flag;//input_flag  0->棚前に行く  1->陳列中  2->バックヤードに帰る
// std_msgs::Int8 input_flagg;
// // void flag_receive(const std_msgs::Int8::ConstPtr& input_flaggg){
// //   input_flagg = *input_flaggg;
// fcsc::Fin fin;
// void fin_receive(const fcsc::Fin::ConstPtr& input_fin){
//   fin = *input_fin;
// }



// nav_msgs::Odometry rover_odom_recieve;
// double roll, pitch, yaw;
// void odom_receive(const nav_msgs::Odometry::ConstPtr& odom_msg){
//   rover_odom_recieve = *odom_msg;

//   tf::Quaternion q(rover_odom_recieve.pose.pose.orientation.x, rover_odom_recieve.pose.pose.orientation.y, rover_odom_recieve.pose.pose.orientation.z, rover_odom_recieve.pose.pose.orientation.w);
//   tf::Matrix3x3 m(q);
//   m.getRPY(roll, pitch, yaw);


//   geometry_msgs::PoseStamped goal;

//   goal.header.stamp = ros::Time::now();
//   goal.header.frame_id = "odom";

// if(status_id==1){
// //移動中
// if((status_id==3)||(status_id==0)){
// //ゴールに到達・もしくはゴールに到達して待機中。

//   if(input_flag == 0&& status_id != 1){
//       goal.pose.position.x = (shelf_x - origin_x)*resolution;
//       goal.pose.position.y = (shelf_y - origin_y)*resolution;
//       // goal.pose.orientation.z = float(1/sqrt((double)2));
//       // goal.pose.orientation.w = float(1/sqrt((double)2));
//       goal.pose.orientation.z = 0;
//       goal.pose.orientation.w = 1;
//       goal_pub.publish(goal);

//       fflag = 0;
//   }





  // if(input_flag == -1){

  // }

  // if(input_flag == 0&& status_id != 1){
  //         goal.pose.position.x = (shelf_x - origin_x)*resolution;
  //     goal.pose.position.y = (shelf_y - origin_y)*resolution;
  //     // goal.pose.orientation.z = float(1/sqrt((double)2));
  //     // goal.pose.orientation.w = float(1/sqrt((double)2));
  //     goal.pose.orientation.z = 0;
  //     goal.pose.orientation.w = 1;
  //     goal_pub.publish(goal);

  //     fflag = 0;


  // }
  // if(input_flag == 6&& status_id != 1){
  //     goal.pose.position.x = (shelf_x1 - origin_x)*resolution;
  //     goal.pose.position.y = (shelf_y1 - origin_y)*resolution;
  //     // goal.pose.orientation.z = float(1/sqrt((double)2));
  //     // goal.pose.orientation.w = float(1/sqrt((double)2));
  //     goal.pose.orientation.z = 0;
  //     goal.pose.orientation.w = 1;
  //     goal_pub.publish(goal);

  //     fflag = 0;


  // }


  // if(input_flag == 1&& status_id != 1){

  //               goal.pose.position.x = rover_odom.pose.pose.position.x;
  //           goal.pose.position.y = rover_odom.pose.pose.position.y + 0.75;
  //           goal.pose.orientation.z = float(1/sqrt((double)2));
  //           goal.pose.orientation.w = float(1/sqrt((double)2));
  //           goal_pub.publish(goal);

  // }

  // if(input_flag == 2&& status_id != 1){
  //         goal.pose.position.x = (back_x - origin_x)*resolution;
  //     goal.pose.position.y = (back_y - origin_y)*resolution;
  //     goal.pose.orientation.z = 0;
  //     goal.pose.orientation.w = 1;
  //     goal_pub.publish(goal);

  // }

  // if(input_flag == 3&& fflag != 1){
  //         goal.pose.position.x = (shelf_x - origin_x)*resolution;
  //     goal.pose.position.y = (shelf_y - origin_y)*resolution - 0.1;
  //     goal.pose.orientation.z = float(1/sqrt((double)2));
  //     goal.pose.orientation.w = float(1/sqrt((double)2));
  //     goal_pub.publish(goal);

  //     fflag = 1;
  // }
  // if(input_flag == 4){
  //                   goal.pose.position.x = rover_odom.pose.pose.position.x;
  //           goal.pose.position.y = rover_odom.pose.pose.position.y - 0.75;
  //           goal.pose.orientation.z = float(1/sqrt((double)2));
  //           goal.pose.orientation.w = float(1/sqrt((double)2));
  //           goal_pub.publish(goal);
  // }

//   else{
//     if(fin.num != 0){
//       for(int bb = 0; bb < fin.num ; bb++){
//         fin_globalmap_x[bb] = (float)(rover_odom.pose.pose.position.x + fin.X_point[bb]*cos(yaw) - fin.Y_point[bb]*sin(yaw));
//         fin_globalmap_y[bb] = (float)(rover_odom.pose.pose.position.y + fin.X_point[bb]*sin(yaw) + fin.Y_point[bb]*cos(yaw));
// ROS_INFO("yaw=%lf  id=%d  x=%f  y=%f  size=%f",yaw,bb,fin_globalmap_x[bb] ,fin_globalmap_y[bb],fin.size[bb] );
//       }
//     }


//   //陳列に行く処理開始 if(output_flag == 0 && input_flag == 0 && avoid_flag == 0 && status_id != 1)
//     if(input_flag == 0 && avoid_flag == 0 && status_id != 1){
//     //set_goal　tana
//       goal.pose.position.x = (shelf_x - origin_x)*resolution;
//       goal.pose.position.y = (shelf_y - origin_y)*resolution;
//       goal.pose.orientation.z = float(1/sqrt((double)2));
//       goal.pose.orientation.w = float(1/sqrt((double)2));
//       goal_pub.publish(goal);

//     //output_flag 0
//       output_flag = 0;
//       avoid_flag = 0;
//       ROS_INFO("idoukaishi");
//     }
//     else if(input_flag == 0 && status_id == 3){
//     //output_flag 1(armにinput_flag = 1にしてもらう)
//       output_flag = 1;
//       input_flag = 1;
//       avoid_flag = 0;
//       ROS_INFO("touchaku");
//     }
//   //陳列に行く処理終了

//   //陳列中開始  if(output_flag == 1 && input_flag == 1 && avoid_flag == 0)
//     if(input_flag == 1 && avoid_flag == 0){
//       if(fin.num != 0){
//         for(int a = 0 ; a < fin.num ; a++){
//           if(min_map_x <= fin_globalmap_x[a] && fin_globalmap_x[a] <= max_map_x && min_map_y <= fin_globalmap_y[a]  && fin_globalmap_y[a] <= max_map_y){
//           }
//        //後ろに下がる処理
//           else if(status_id != 1){
//             goal.pose.position.x = rover_odom.pose.pose.position.x;
//             goal.pose.position.y = (shelf_x - origin_x)*resolution + 1;
//             goal.pose.orientation.z = float(1/sqrt((double)2));
//             goal.pose.orientation.w = float(1/sqrt((double)2));
//             goal_pub.publish(goal);

//             output_flag = 0;
//             input_flag = 1;
//             avoid_flag = 1;

//             ROS_INFO("back");


//             break;
//           }
//         }
//       } 
//       else{
//       }
//     }
//   //陳列中終了

//   //回避開始     if(output_flag == 0 && input_flag == 4 && avoid_flag == 1)
//     if(input_flag == 5){
//             goal.pose.position.x = rover_odom.pose.pose.position.x;
//             goal.pose.position.y = rover_odom.pose.pose.position.y;
//             goal.pose.orientation.z = float(1/sqrt((double)2));
//             goal.pose.orientation.w = float(1/sqrt((double)2));

//     }
// if(input_flag == 4){
//     // if(input_flag == 4 && avoid_flag == 1){
//       // if(fin.num != 0){
//       //   for(int aa = 0 ; aa < fin.num ; aa++){
//       //     if(min_map_x <= fin_globalmap_x[aa] && fin_globalmap_x[aa] <= max_map_x && min_map_y <= fin_globalmap_y[aa]  && fin_globalmap_y[aa] <= max_map_y){
//       //       output_flag = 0;
//       //       input_flag = 0;
//       //       avoid_flag = 0;
//       //     }
//       //  //後ろに下がる処理
//       //     else if(status_id != 1){
//       //       goal.pose.position.x = rover_odom.pose.pose.position.x;
//       //       goal.pose.position.y = (shelf_x - origin_x)*resolution + 1;
//       //       goal.pose.orientation.z = float(1/sqrt((double)2));
//       //       goal.pose.orientation.w = float(1/sqrt((double)2));
//       //       goal_pub.publish(goal);

//       //       output_flag = 0;
//       //       input_flag = 1;
//       //       avoid_flag = 1;

//       //       break;
//       //     }
//       //   }
//       // } 
//       // else{
//       //   output_flag = 0;
//       //   input_flag = 0;
//       //   avoid_flag = 0;
//       // }
//             goal.pose.position.x = rover_odom.pose.pose.position.x;
//             goal.pose.position.y = rover_odom.pose.pose.position.y + 0.75;
//             goal.pose.orientation.z = float(1/sqrt((double)2));
//             goal.pose.orientation.w = float(1/sqrt((double)2));
//             goal_pub.publish(goal);
//             ROS_INFO("kaihi");
//     }

//   //回避終了

//   //帰る開始
//     if(input_flag == 2 && status_id != 1){
//       goal.pose.position.x = (back_x - origin_x)*resolution;
//       goal.pose.position.y = (back_y - origin_y)*resolution;
//       goal.pose.orientation.z = 0;
//       goal.pose.orientation.w = 1;
//       goal_pub.publish(goal);
//       ROS_INFO("kitaku");
//     } 
//   //帰る終了


//     std_msgs::Int8 pub_flag;
//     pub_flag.data = output_flag;

//     flag_pub.publish(pub_flag);

//   }





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