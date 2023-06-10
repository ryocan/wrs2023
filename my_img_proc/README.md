## my_img_proc
画像処理とUR3の制御をするpkg

## launch
### ur3関連
<!-- 下記を実行すればUR3のbringup, moveit, rvizが可能になる -->
```
roslaunch my_img_proc ur3.launch
```

<!-- 上記ur3.launchは下記3つのlaunchをまとめたものなので，独立してlaunchしたければ下記を別々のターミナルで動かす
```
roslaunch ur_robot_driver ur3_bringup.launch robot_ip:=133.91.73.226 kinematics_config:=${my_img_proc}/config/my_robot_calibration.yaml
```
```
roslaunch ur3_moveit_config moveit_planning_execution.launch
```
```
roslaunch ur3_moveit_config moveit_rviz.launch
``` -->

### 画像処理関連
```
roslaunch my_img_proc rs_yolact.launch
```