// /*serverから目的地情報を受け取ってロボットに司令を出すノード、serverにはrobotの自己位置情報を送る（1秒に一回、変更可）*/

// #include <ros/ros.h>
// #include <move_base_msgs/MoveBaseAction.h>
// #include <actionlib/client/simple_action_client.h>
// #include <nav_msgs/Odometry.h>
// #include <std_msgs/Int8.h>
// #include <tf/transform_broadcaster.h>
// #include <tf/transform_listener.h>
// #include <stdio.h>
// #include <sys/types.h>
// #include <sys/socket.h>
// #include <netinet/in.h>
// #include <arpa/inet.h>
// #include <netdb.h>
// #include <geometry_msgs/PoseWithCovarianceStamped.h>

// #define BUFFER_SIZE 256

//  //ros用
// typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
// geometry_msgs::PoseWithCovarianceStamped robot_odom,start_odom;
// double r,p,y,yaw;
// double goal_x = 0,goal_y = 0,goal_flag = 0;
// int count;

// int first_receive_flag = 0;

// void *func_thread(void *p);
// void OdomRecieve(const geometry_msgs::PoseWithCovarianceStamped odom);
// void *sendgoal_thread(void *arg);

// //ros用スレッド
// void *func_thread(void *p) {
// 	ros::NodeHandle nh;
// 	ros::Subscriber odom_sub = nh.subscribe("amcl_pose"/*fusion_odom*/, 10, OdomRecieve);
// 	ros::spin ();
// }

// //robot情報取得
// void OdomRecieve(const geometry_msgs::PoseWithCovarianceStamped odom){

// 	if(first_receive_flag == 0){
// 		start_odom = odom;
// 		first_receive_flag = 1;
// 	}

// 	robot_odom = odom;

// //クォータニオン→オイラー角
// 	tf::Quaternion temp_quat(robot_odom.pose.pose.orientation.x,
// 			   robot_odom.pose.pose.orientation.y,
// 			   robot_odom.pose.pose.orientation.z,
// 			   robot_odom.pose.pose.orientation.w);
// 	r = 0.0; p = 0.0; y = 0.0;
// 	tf::Matrix3x3(temp_quat).getRPY(r, p, y);
// 	yaw = y;//z軸回りの回転角

// }

// void *sendgoal_thread(void *arg){

// 	double goal_pos_x = 0.0, goal_pos_y = 0.0, goal_pos_yaw = 0.0;
// 	int stop_flag = 0,goal_send_flag = 0,pre_goal_send_flag = 0;
// 	ros::Rate rate(100);

// 	MoveBaseClient ac("move_base", true);

// 	while(!ac.waitForServer(ros::Duration(5.0))){
// 		ROS_INFO("Waiting for the move_base action server to come up");
// 	}

// 	move_base_msgs::MoveBaseGoal goal;

// 	while(ros::ok()){
// 		//cout << "input continue:0 or stop:1"<< endl;
// 		//cin >> stop_flag;
// 		stop_flag = 0;

// 		if(stop_flag == 1){ //send stop

// 			goal.target_pose.header.frame_id = "map";
// 			goal.target_pose.header.stamp = ros::Time::now();

// 			goal.target_pose.pose.position = robot_odom.pose.pose.position;
// 			goal.target_pose.pose.orientation.x = 0;
// 			goal.target_pose.pose.orientation.y = 0;		
// 			goal.target_pose.pose.orientation.z = 0;
// 			goal.target_pose.pose.orientation.w = 1;
			
// 			ROS_INFO("Sending stop");
// 			ac.sendGoal(goal);
// 			continue;
// 		}

// 		//cout << "now goal_send_flag is " << pre_goal_send_flag << endl;
// 		//cout << " input next goal_send_flag " << endl;
// 		//cin >> goal_send_flag;
// 		goal_send_flag = static_cast<int>(goal_flag);

// 		if(goal_send_flag == pre_goal_send_flag){
// 		}
// 		else if(goal_send_flag != pre_goal_send_flag){
// 			goal.target_pose.header.frame_id = "map";
// 			goal.target_pose.header.stamp = ros::Time::now();
// 			//cout << "now position(x,y) is" << robot_odom.pose.pose.position.x << "," << robot_odom.pose.pose.position.y << endl;
// 			//cout << "input goal_send x,y" << endl;
// 			//cin >> goal_pos_x;
// 			//cin >> goal_pos_x;
// 			goal_pos_x = goal_x/1000;
// 			goal_pos_y = goal_y/1000;

// 			goal.target_pose.pose.position.x = goal_pos_x;
// 			goal.target_pose.pose.position.y = goal_pos_y;
// 			goal.target_pose.pose.orientation.x = 0;
// 			goal.target_pose.pose.orientation.y = 0;
// 			goal.target_pose.pose.orientation.z = sin(yaw/2);
// 			goal.target_pose.pose.orientation.w = cos(yaw/2);
// 			printf("goal_pos_x = %lf,goal_pos_y = %lf\n",goal_pos_x,goal_pos_y);
// 			ROS_INFO("Sending goal");
// 			ac.sendGoal(goal);
// 		}

// 		pre_goal_send_flag = goal_send_flag;
// //イートインスペース座標0.8,-3.5
// 		double r_g_dis;
// 		// double r_g_dis = sqrt(pow((robot_odom.pose.pose.position.x - 0.8)) + pow((robot_odom.pose.pose.position.y - (-3.5))));
// 		if (r_g_dis < 0.3){
// 			printf("reset_poseshvfsuufsubvvauek\n");
// 			//usleep(1000000);
// 			count++;
// 			if (count == 10 ){
// 				goal.target_pose.pose.position.x = 0;
// 				goal.target_pose.pose.position.y = 0;
// 				goal.target_pose.pose.orientation.x = 0;
// 				goal.target_pose.pose.orientation.y = 0;
// 				goal.target_pose.pose.orientation.z = sin(yaw/2);
// 				goal.target_pose.pose.orientation.w = cos(yaw/2);
// 			//printf("goal_pos_x = %lf,goal_pos_y = %lf\n",goal_pos_x,goal_pos_y);
// 			//ROS_INFO("Sending goal");
// 				ac.sendGoal(goal);
// 				count = 0;
// 			}

// 		}
			
					

		
// 		rate.sleep();
// 	}
// }

// int main(int argc, char** argv){
//   ros::init(argc, argv, "cliant");
//   ros::NodeHandle n;
//   char destination[80];
//   /* IP アドレス、ポート番号、ソケット */
//   //char destination[80] = "192.168.0.10";
//   std::cout << "input IP adress" << std::endl;
//   std::cin >> destination;
//   //char destination[80] = "192.168.3.19";
//   unsigned short port = 9876;
//   int dstSocket;
//   char buffer[BUFFER_SIZE];

//   /* sockaddr_in 構造体 */
//   struct sockaddr_in dstAddr;

//   /* 各種パラメータ */
//   int status;
//   int numsnt;
//   int numrcv;
//   char *toSendText = "This is a test";

//   /************************************************************/
//   /* 相手先アドレスの入力 */
//   //printf("Connect to ? : (name or IP address) ");
//   //scanf("%s", destination);

//   /* sockaddr_in 構造体のセット */
//   memset(&dstAddr, 0, sizeof(dstAddr));
//   dstAddr.sin_port = htons(port);
//   dstAddr.sin_family = AF_INET;
//   dstAddr.sin_addr.s_addr = inet_addr(destination);
 
//   /* ソケット生成 */
//   dstSocket = socket(AF_INET, SOCK_STREAM, 0);

//   /* 接続 */
//   printf("Trying to connect to %s: \n", destination);
//   connect(dstSocket, (struct sockaddr *) &dstAddr, sizeof(dstAddr));
//   //受信用スレッド
//   int b = 0;
//   pthread_t pthread_1,pthread_2;
//   pthread_create( &pthread_1, NULL, &func_thread, &b);

//   pthread_create( &pthread_2, NULL, &sendgoal_thread, NULL);
//   /* パケット送出 */
//   /*for(int i=0; i<10; i++)*/
//   char s[10];
//   int robot_x = 0;
//   int robot_y = 0;
//   int number,l;
//   int recieve_flag = 0;
//   char robot_str_x[20],robot_str_y[20],robot_info[40],goal_str_x[10],goal_str_y[10],goal_str_flag[10];
//   int num = 3;
//   char *goal_info; //送られてくるゴールの情報
//   char *end1, *end2, *end3;
//   while(1) {
//     if(recieve_flag == 0){
//     	printf("robot_info recieving...\n");
// 	    while(1){
// 		robot_x = static_cast<int>(robot_odom.pose.pose.position.x * 1000);
// 		robot_y = static_cast<int>(robot_odom.pose.pose.position.y * 1000);
// 		//printf("robot_x: %d\n",robot_x);
// 		sleep(1);
// 		if(robot_x != 0 || robot_y != 0){
// 			printf("robot_info recieved\n");
// 			recieve_flag = 1;
// 			break;
// 		}
// 	    }
//     }
//     else {
//     	robot_x = static_cast<int>(robot_odom.pose.pose.position.x * 1000);
//     	robot_y = static_cast<int>(robot_odom.pose.pose.position.y * 1000);
//     }	
//     snprintf(robot_str_x, sizeof(robot_str_x), "%d", robot_x);
//     snprintf(robot_str_y, sizeof(robot_str_y), "%d", robot_y);
//     sprintf(robot_info, "%s,%s\n", robot_str_x, robot_str_y);
//     printf("cliant sending...\n");	
//     send(dstSocket, robot_info, strlen(robot_info)+1, 0);
//     numrcv = recv(dstSocket, buffer, BUFFER_SIZE, 0);
// //来た文字列を分割、数値に戻す
//     goal_info=strtok(buffer,",");
//     goal_x = atof(goal_info); 
//     for(int i=1;i<3;i++){
//     	goal_info = strtok(NULL,",");
// 	if (i == 1){
// 		goal_y = atof(goal_info); 
// 	}
// 	else if (i == 2){
// 		goal_flag = atof(goal_info); 
// 	}
//     }
    
//    /* for(int i=0;i<3;i++){
//     	printf("goal_str_x:%s goal_str_y:%s goal_str_flag:%s\n", goal_str_x,goal_str_y,goal_str_flag);
//     }
    
//     long goal_x = strtol(goal_str_x, &end1, 10);
//     long goal_y = strtol(goal_str_y, &end2, 10);
//     long goal_flag = strtol(goal_str_flag, &end3, 10);*/
//     printf("goal_x:%lf goal_y:%lf goal_flag:%lf\n",goal_x,goal_y,goal_flag);
//     printf("cliant received: %s\n", buffer);
//     sleep(1);
//   }
//   /* ソケット終了 */
//   close(dstSocket);
// }



/*serverから目的地情報を受け取ってロボットに司令を出すノード、serverにはrobotの自己位置情報を送る（1秒に一回、変更可）*/

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int8.h>
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


#define BUFFER_SIZE 256

ros::Publisher flag_pub;

int main(int argc, char** argv){
  ros::init(argc, argv, "cliant");
  ros::NodeHandle n;
  char destination[80];
  /* IP アドレス、ポート番号、ソケット */
  //char destination[80] = "192.168.0.10";
  std::cout << "input IP adress" << std::endl;
  std::cin >> destination;
  //char destination[80] = "192.168.3.19";
  unsigned short port = 8000;
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

    flag.data = buffer;
    flag_pub.publish(flag);
    flag_pub = n.advertise<std_msgs::String>("/flag", 1);

    
    printf("cliant received: %s\n", buffer);

    sleep(1);
    strcpy(robot_info,"abcdd");    
    if(numrcv == -1){
    	break;
    }
    send(dstSocket, robot_info, strlen(robot_info)+1, 0);
  }
  /* ソケット終了 */
  close(dstSocket);

  return 0;
}

