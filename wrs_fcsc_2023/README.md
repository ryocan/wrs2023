# wrs_fcsc_2023
wrs fcsc2023の陳列廃棄タスク用の自作pkg．稚拙なコードで恥ずかしいが，何も残さないわけにはいかないのでここに全部メモしておく．

・[手法とコード説明](https://github.com/ryocan/wrs_fcsc_2023/tree/main#%E6%89%8B%E6%B3%95%E3%81%A8%E3%82%B3%E3%83%BC%E3%83%89%E8%AA%AC%E6%98%8E)

・[大会で使ったlaunch](https://github.com/ryocan/wrs_fcsc_2023/tree/main#%E5%A4%A7%E4%BC%9A%E3%81%A7%E4%BD%BF%E3%81%A3%E3%81%9Flaunch:~:text=%E3%81%84%E3%81%9Flaunch-,%E5%A4%A7%E4%BC%9A%E3%81%A7%E4%BD%BF%E3%81%A3%E3%81%9Flaunch,-1%E3%81%A4%E3%82%81%EF%BC%9Arealsense)

・[他に必要なpkg](https://github.com/ryocan/wrs_fcsc_2023/tree/main#%E4%BB%96%E3%81%AB%E5%BF%85%E8%A6%81%E3%81%AApkg)

・[このpkgでのlaunch](https://github.com/ryocan/wrs_fcsc_2023/tree/main#%E3%81%93%E3%81%AEpkg%E3%81%A7%E3%81%AElaunch)

・[他pkgを用いたlaunch](https://github.com/ryocan/wrs_fcsc_2023/tree/main#%E4%BB%96pkg%E3%82%92%E7%94%A8%E3%81%84%E3%81%9Flaunch)

## 手法とコード説明
実際に動かす時に使っているプログラムと手法の概要を説明していく．
チャンピオン動画も載せておく(https://youtu.be/GLgYPF3gECU)．
pkg内には大会で動かすのには必要ないコードも入ってしまっているのだが，後述するプログラムだけ読んでもらえれば良いかと思う，
基本的にロボットアームは「棚の引き出し」「廃棄タスク」「棚の押し戻し」の3つしか行っていない．

### ・src/systemManager.cpp

これがシステム全体を動かすためのメインプログラムになっている．
移動ロボットにフラグを送ったり，移動ロボットから受け取ったフラグをもとに移動ロボットを動かしたりしている．
flag_mobile_robot_systemというのが，移動ロボットから送られてくるフラグを読み込んだものになる．
flag_mobile_robot_systemが0というのは，移動ロボットが動いているので，ロボットアームは動作してはいけないという状態になっている．
なので0のときはなにもしない．
flag_mobile_robot_systemが1のときはロボットアームが動いて良い時なので，ロボットアームのタスクを動かす．
このとき，flag_ur3_systemというロボットアームのタスク状態を管理するフラグを用いて，「棚の引き出し」「廃棄タスク」「棚の押し戻し」の状態を分けている．

### ・include/systemManager_declare.h

systemManager.cppやそのほかのコードで使うグローバル変数をここで宣言している．

### ・include/systemManager_pub.hpp

ロボットアームから移動ロボットにflagを送るためのpublisherのコード．

### ・include/systemManager_draw.hpp

棚引き出しのためのコード．
これはRealSenseのRGB画像から棚に貼ったArUcoを検出して棚の引き出しを行っている．
なおこのコードを書いたときは，カメラ座標からロボット座標系への座標変換ができていなかったので，力技プログラムになっている．
まず，ロボットアームでArUcoマーカを検出する．
その後，RealSenseからArUcoマーカの重心までの距離を測る．
そこから棚の引き出し量を算出して，棚を引き出すというもの．
Youtubeの動画を載せておく．(https://youtu.be/LdUHL5t58Vw)

### ・include/systemManager_disposal.hpp

廃棄タスクのコード．
まずグリッパのアクティベートを行って，グリッパを動かせるようにする．
その後，yolact画像からマスク画像を生成する．
なおこのとき，yolact画像で色付きのマスク画像が得られているのだが，物体ごとに処理を変えたりはしないので，yolact画像から背景の黒色以外を全部物体となるようなマスク画像の作り方をしている．
その後，マスク画像をもとに物体の輪郭(形状)を算出する．
得られた物体輪郭より各物体の重心を計算する．
その重心結果にRealSenseから得られた物体重心までの距離を用いて，RealSense座標系の3次元座標に変換する．
それをもとに，カメラ座標からロボットのbase_link座標系に変換する．
そのbase_link座標系上の物体の座標をもとにロボットアームを制御する．
手首回転量の算出は，ロボットアームに搭載したカメラから，yolact画像→主成分分析(pca)を行い，第2主成分までの角度を制御値としている．
人の介入は，環境地図から検出している．
flag_customer_systemが環境地図からの検出結果で，0が人検出なし，それ以外が人検出ありになっている．
ので，0以外を受け取ったときには，作業を中断する音声を流し，棚を戻すステップに無理やりいく．
でも作業中断した後に，棚前に戻ってきて，作業を再開させないといけない．
ので，もう一度物体認識から再開する感じにしている．

### systemManager_putback.hpp

棚を押し戻すプログラム．
棚を引き出した姿勢を予め保存しておき，その姿勢を経由して棚を引き出している．
特にセンシングはしていない．

## 大会で使ったlaunch
1つめ：realsenseとyolact
```
roslaunch wrs_fcsc_2023 rs_yolact.launch 
```

2つめ：ur3の起動
```
roslaunch wrs_fcsc_2023 ur3.launch
```
3つめ：グリッパの起動(USBのポートは随時確認)
```
rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /dev/ttyUSB0
```
4つめ：systemManager(ロボットアームと移動ロボットの制御)
```
rosrun wrs_fcsc_2023 systemManager
```
5つめ：debugu用：移動ロボットからの疑似的なflag送信(1回だけ)
```
rostopic pub -1 /flag_mobile_robot std_msgs/Int8 "data: 1" 
rostopic pub -1 /flag_customer std_msgs/Int8 "data: 0"
```

## 他に必要なpkg
このwrs_fcsc_2023をgit cloneするだけだともちろん動かないので，下記サイトをsrc内にクローンしてほしい．
なおyolact_rosについては，[補足説明](https://github.com/ryocan/wrs_fcsc_2023_yolact.git)を参照してほしい
また新しいUbuntu環境だとapt installを何個かやらなきゃいけないんだけど，忘れちゃいましたごめんなさい．
たぶん勝手にあぶり出てくると思います．

| pkg名  | URL |
| ------------- | ------------- |
| baldor  | https://github.com/crigroup/baldor  |
| criutils  | https://github.com/crigroup/criutils | 
| geometric_shapes | https://github.com/ros-planning/geometric_shapes |
| geometry2 | https://github.com/ros/geometry2 |
| handeye | https://github.com/crigroup/handeye |
| image_pipeline | https://github.com/ros-perception/image_pipeline |
| moveit | https://github.com/ros-planning/moveit |
| moveit_calibration | https://github.com/ros-planning/moveit_calibration |
| moveit_msgs | https://github.com/ros-planning/moveit_msgs |
| moveit_resources | https://github.com/ros-planning/moveit_resources |
| moveit_tutorials | https://github.com/ros-planning/moveit_tutorials |
| moveit_visual_tools(noetic-develに！！) | https://github.com/ros-planning/moveit_visual_tools.git |
| realsense-ros(branchはros1-legacyに！)| https://github.com/IntelRealSense/realsense-ros |
| robotiq | https://github.com/ros-industrial/robotiq.git |
| rviz_visual_tools |https://github.com/PickNikRobotics/rviz_visual_tools |
| srdfdom | https://github.com/ros-planning/srdfdom.git |
| universal_robot | https://github.com/ros-industrial/universal_robot.git |
| Univerlas_Robots_ROS_Driver | https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git |
| yolact | https://github.com/dbolya/yolact.git |
| yolact_ros | https://github.com/Eruvae/yolact_ros.git |
| yolact_ros_msgs | https://github.com/Eruvae/yolact_ros_msgs.git |

## このpkgでのlaunch
UR3：実機
```
roslaunch wrs_fcsc_2023 ur3.launch
```
UR3：Gazebo
```
roslaunch wrs_fcsc_2023 ur3_gazebo.launch
```
realsenseとyolactを一緒にlaunch
```
roslaunch wrs_fcsc_2023 rs_yolact.launch
```
robotiq
```
roslaunch wrs_fcsc_2023 robotiq.launch
```

## 他pkgを用いたlaunch
ur3.launchは下記3つをまとめたものなので，独立してlaunchしたければ下記を別々のターミナルで動かす
```
roslaunch ur_robot_driver ur3_bringup.launch robot_ip:=133.91.73.226 kinematics_config:=${wrs_fcsc_2023}/config/my_robot_calibration.yaml
```
```
roslaunch ur3_moveit_config moveit_planning_execution.launch
```
```
roslaunch ur3_moveit_config moveit_rviz.launch
```

ur3_gazebo.launchは下記3つをまとめたものなので，独立してlaunchしたければ下記を別々のターミナルで動かす
```
roslaunch ur_gazebo ur3_bringup.launch
```
```
roslaunch ur3_moveit_config moveit_planning_execution.launch sim:=true
```
```
roslaunch ur3_moveit_config moveit_rviz.launch
```

上記robotiq.launchは下記2つをまとめたものなので，独立してlaunchしたければ下記を動かす
```
sudo chmod 666 /dev/ttyUSB0
rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /dev/ttyUSB0
```
