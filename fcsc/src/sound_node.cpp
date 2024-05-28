#include <ros/ros.h>
#include <AL/al.h>
#include <AL/alc.h>
#include <AL/alut.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <std_msgs/Int8.h>

#include <unistd.h>//sleep関数用



//move_baseのステータスのコールバック関数
//void satatus_Callback (const actionlib_msgs::GoalStatusArray::ConstPtr& status_msg){
void satatus_Callback (const std_msgs::Int8& status_msg){	
 int status_id = 0;
 static int eX = 0; 
//中身のチェック
 /*
 if(!status_msg->status_list.empty()){
   int n_status = status_msg->status_list.size();
   status_id = status_msg->status_list[n_status - 1].status;
 } 
 else{}
 */

 status_id = status_msg.data;//id更新
//音を鳴らす
    //t:音の秒数 f:音の高さ(周波数) w_t:待機時間

 double w_t = 0.0001;

 if (status_id>0){

   double t= 0.45;
     int f = 1400;//行き

  if(status_id==1){
/*
    if(eX==0){
      //system("(cd ~/catkin_ws/src/fcsc/scripts ; python3 play1.py)&");
      system("cd ~/catkin_ws/src/fcsc/scripts ; python3 play.py");
     eX = 1;
    }*/

  }
  else{
    
    f = 700;//帰り
    t = 1.0;

/*    if(eX==0){
    //system("(cd ~/catkin_ws/src/fcsc/src ; python3 play.py)&");
      system("cd ~/catkin_ws/src/fcsc/scripts ; python3 play.py");
     eX = 1;
    }*/
  }
  
	int T = 22050 * t;


  ALCdevice *device;
  ALCcontext *context;
  ALshort data[22050];
  ALuint buffer, source;

	//デバイスを開く
    device = alcOpenDevice(NULL);
    //コンテキストを生成
    context = alcCreateContext(device, NULL);
    //使用するコンテキストの指定
    alcMakeContextCurrent(context);
    //バッファの生成
    alGenBuffers(1, &buffer);




   //信号を生成する
   for (int i = 0; i < T; ++i) {
       data[i] = sin(i * 3.14159 * 2 * f / 22050) * 32767;
   }
   //信号をバッファに入れる
   alBufferData(buffer, AL_FORMAT_MONO16, data, sizeof(data), 22050);
   //ソースを生成
   alGenSources(1, &source);
   //バッファからソースを作る
   alSourcei(source, AL_BUFFER, buffer);


   //ソースを再生する

   alSourcePlay(source);


   int tmp_t = t * 1000 * 1000;
   usleep(tmp_t);

    //音の信号を初期化するための処置
    for (int i = 0; i < T; ++i) {
        data[i] = sin(i * 3.14159 * 2 * 0 / 22050) * 32767;
    } 

   //お片づけ
   alSourceStop(source);
   alDeleteSources(1, &source);
   alDeleteBuffers(1, &buffer);
   alcMakeContextCurrent(NULL);
   alcDestroyContext(context);
   alcCloseDevice(device);

   int tmp_wt = w_t * 1000 * 1000;
   usleep(tmp_wt);
 }

 else{
  eX = 0;
 }

}


int main(int argc, char** argv){
  ros::init(argc, argv, "sound_node");
  ros::NodeHandle nh;
  //ros::Subscriber status_sub = nh.subscribe("/move_base/status", 1, satatus_Callback);
  ros::Subscriber status_sub = nh.subscribe("/sound_flag", 1, satatus_Callback);
  ros::spin();
  return 0;
}