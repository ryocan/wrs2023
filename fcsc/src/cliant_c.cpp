#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Char.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <string.h>
#include <iostream>
#include <vector>
#include <unistd.h>//sleep関数用


#define BUFFER_SIZE 256

ros::Publisher flag_pub;

int status_id = 0;
void status_receive(const actionlib_msgs::GoalStatusArray::ConstPtr& status_msg){
  if(!status_msg->status_list.empty()){
    int n_status = status_msg->status_list.size();
    status_id = status_msg->status_list[n_status - 1].status;
  }
}

void tokenize(std::string const& str, const char delim,
    std::vector<std::string>& out)
{
    size_t start;
    size_t end = 0;

    while ((start = str.find_first_not_of(delim, end)) != std::string::npos)
    {
        end = str.find(delim, start);
        out.push_back(str.substr(start, end - start));
    }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "cliant");
  ros::NodeHandle n;
  ros::Subscriber status_sub = n.subscribe("/move_base/status", 1, status_receive);
  char destination[80];
  /* IP アドレス、ポート番号、ソケット */
  //char destination[80] = "192.168.0.10";
  std::cout << "input IP adress" << std::endl;
  std::cin >> destination;
  //char destination[80] = "192.168.3.19";
  unsigned short port = 9999; //8000
  int dstSocket;
  char buffer[BUFFER_SIZE];

  /* sockaddr_in 構造体 */
  struct sockaddr_in dstAddr;

  /* 各種パラメータ */
  int status;
  int numsnt;
  int numrcv;
  char *toSendText = "This is a test";

  /************************************************************/
  /* 相手先アドレスの入力 */
  //printf("Connect to ? : (name or IP address) ");
  //scanf("%s", destination);

  /* sockaddr_in 構造体のセット */
  memset(&dstAddr, 0, sizeof(dstAddr));
  dstAddr.sin_port = htons(port);
  dstAddr.sin_family = AF_INET;
  dstAddr.sin_addr.s_addr = inet_addr(destination);
 
  /* ソケット生成 */
  dstSocket = socket(AF_INET, SOCK_STREAM, 0);

  /* 接続 */
  printf("Trying to connect to %s: \n", destination);
  connect(dstSocket, (struct sockaddr *) &dstAddr, sizeof(dstAddr));
  //受信用スレッド
  int b = 0;

  /* パケット送出 */
  /*for(int i=0; i<10; i++)*/
  char s[10];
  int robot_x = 0;
  int robot_y = 0;
  int number,l;
  int recieve_flag = 0;
  char robot_str_x[20],robot_str_y[20],robot_info[40],goal_str_x[10],goal_str_y[10],goal_str_flag[10];
  int num = 3;
  char *goal_info; //送られてくるゴールの情報
  char *end1, *end2, *end3;

  std_msgs::String flag;

  while(dstSocket > 0) {

  	numrcv = recv(dstSocket, buffer, BUFFER_SIZE, 0);
    
        char delim[] = ",";
        char* token;

    // std::vector<std::string> out;
    token = strtok(buffer, delim);

    //;  // 最大で８個に分割されると想定
    // split(s, buffer, result, sizeof(result)/sizeof(result[0]));
// char *result ;
// result = strtok(buffer, ",");

// if(result != NULL) {
    // flag.data = *out[0];
//   }
// int i=0;
// std::string outtt;
//         std::string outtttt(buffer,sizeof[buffer]/sizeof(outttt[0]));



// const char *outt = out;

    //     for (auto& s  : out) {
    //    std::cout << s << std::endl;
    //    // outt[i] = s;
    //    if(i==0){

    //     // outtt = s;
    //    }
    //    i++;
    // }
    flag.data = buffer;
          ROS_INFO("%s",flag.data.c_str());
        // ROS_INFO("%d"out[0].typeid(out).name());
        ROS_INFO("%s",token);

    flag_pub = n.advertise<std_msgs::String>("/flag", 1);
    flag_pub.publish(flag);
     
     static int ex = 0;
     ros::spinOnce();
     ROS_INFO("status_id :%d, ex: %d",status_id, ex);
     
     

/*    if (status_id==3 && ex==0){
        //tcpで杉本くんのプログラムに値を送る
        char* hello = "goal,goal,goal";
        send(dstSocket, hello, strlen(hello), 0);
        ex = 1;
    }
    else if(status_id!=3 && ex==1){
        char* hello = "notgoal";
        send(dstSocket, hello, strlen(hello), 0);
        ex = 0;
    }*/
    
    printf("cliant received: %s\n", buffer);


    usleep(100000);
    // strcpy(robot_info,"abcdd");    
    if(numrcv == -1){
        ROS_INFO("END");
    	break;
    }
    // send(dstSocket, robot_info, strlen(robot_info)+1, 0);
  }
  /* ソケット終了 */
  close(dstSocket);

  return 0;
}


