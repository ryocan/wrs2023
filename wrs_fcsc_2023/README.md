# wrs_fcsc_2023
wrs fcsc2023の陳列廃棄タスク用の自作pkg

・[他に必要なpkg](https://github.com/ryocan/wrs_fcsc_2023/tree/main#%E4%BB%96%E3%81%AB%E5%BF%85%E8%A6%81%E3%81%AApkg)

・[このpkgでのlaunch](https://github.com/ryocan/wrs_fcsc_2023/tree/main#%E3%81%93%E3%81%AEpkg%E3%81%A7%E3%81%AElaunch)

・[他pkgを用いたlaunch](https://github.com/ryocan/wrs_fcsc_2023/tree/main#%E4%BB%96pkg%E3%82%92%E7%94%A8%E3%81%84%E3%81%9Flaunch)

## 他に必要なpkg
このwrs_fcsc_2023をgit cloneするだけだともちろん動かないので，下記サイトをsrc内にクローンしてほしい．
なおyolact_rosについては，[補足説明](https://github.com/ryocan/wrs_fcsc_2023_yolact.git)を参照してほしい

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

## apt install
realsenseのinstall
```
sudo apt-get install ros-noetic-realsense2-camera
```
UR3
```
sudo apt-get install ros-noetic-universal-robots
```

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

## 大会用
```
1つめ：realsenseとyolact
roslaunch wrs_fcsc_2023 rs_yolact.launch 

2つめ：ur3
roslaunch wrs_fcsc_2023 ur3.launch

3つめ：グリッパ
rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /dev/ttyUSB0

4つめ：メインのsystemManager
rosrun wrs_fcsc_2023 systemManager

5つめ：疑似的なflag送信
1回だけ
rostopic pub -1 /flag_mobile_robot std_msgs/Int8 "data: 1" 
rostopic pub -1 /flag_customer std_msgs/Int8 "data: 0"

10Hzで
rostopic pub -r 10 /flag_mobile_robot std_msgs/Int8 "data: 1" 
```