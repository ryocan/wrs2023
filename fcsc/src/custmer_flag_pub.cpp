#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>

std_msgs::Int8 custmer_flag;

ros::Publisher custmer_pub;
static int flagg = 2;
void flag_receive(const std_msgs::String::ConstPtr& input_flag){
  std_msgs::String input_flagg = *input_flag;
  int flag = atoi(input_flagg.data.c_str());//人がいるかいないかの判別 flag==0いない　
  
  if(flag>0){
    flag =1;
  }
  else{
    flag = 0;
  }

  if(flagg!=flag){
  if(flag==0){
  custmer_flag.data = 0;
  custmer_pub.publish(custmer_flag);
  flagg = flag;
  }
  if(flag>0){
  custmer_flag.data = 1;
  custmer_pub.publish(custmer_flag);
  flagg = flag;
  }

  }
}

  int main(int argc, char** argv){
  ros::init(argc, argv, "goal_set");
  ros::NodeHandle nh;
  ros::Subscriber flag_sub = nh.subscribe("/flag", 1, flag_receive);
  custmer_pub = nh.advertise<std_msgs::Int8>("/flag_customer", 1);
  ros::spin();
  return 0;
}