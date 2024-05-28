#include <ros/ros.h>
#include <detection_msgs/BoundingBoxes.h> //検出したものを受け取りやす
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include<wrs_fcsc_2023/yolo_detect.h>

#include <iostream>
#include <string>
#include <cstring>


ros::Publisher depth_pub;

//yoloで認識した物の座標を算出するノード
detection_msgs::BoundingBoxes detection_data;//格納
void detection_Callback(const detection_msgs::BoundingBoxes::ConstPtr& detection_msgs)
{

    detection_data = *detection_msgs;
  
}
wrs_fcsc_2023::yolo_detect depth_data;
void depth_Callback(const sensor_msgs::Image::ConstPtr& depth_msgs)
{

    cv_bridge::CvImagePtr cv_ptr;

    //深度を格納する入れ物

    try
    {
        cv_ptr = cv_bridge::toCvCopy(depth_msgs, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    int i = 0;
    int j = 0;
    int id = 0;//人のカウント
    //バウンドボックスの中心座標格納
    int x[100] = {};
    int y[100] = {};

    i = detection_data.bounding_boxes.size();//データ数確認
    ROS_INFO("num of data:%d",i);

    if(i>0)
    {//物体を検出したら分類する
        for(j=0; j<i; j++)
        {

            if(detection_data.bounding_boxes[j].Class =="person")
            {//人なら座標を獲得する

                x[id] = detection_data.bounding_boxes[j].xmin + (detection_data.bounding_boxes[j].xmax - detection_data.bounding_boxes[j].xmin)/2;
                y[id] = detection_data.bounding_boxes[j].ymin + (detection_data.bounding_boxes[j].ymax - detection_data.bounding_boxes[j].ymin)/2;

                ROS_INFO("x: %d y: %d",x[id] ,y[id]);

                id++;
            }
        }
    }


    ROS_INFO("num of human is %d",id);

    float depth[100] = {};//距離

    int k=0;
    if(id>0)
    {
        depth_data.id.resize(id);//検出した人数にあわせてリサイズ　すごく重要
        depth_data.depth.resize(id);
        depth_data.person_num = id;
        for(k=0;k<id; k++)
        {

          depth[k] = 0.001*cv_ptr->image.at<u_int16_t>(y[k], x[k]);
            ROS_INFO("id: %d distance: %lf",k+1 ,depth[k]);

           /* ROS_INFO("j: %d", j);
            ROS_INFO("data.id.size: %d", depth_data.id.size());*/
            depth_data.id[k] = k+1;
            depth_data.depth[k] = depth[k];

        }

        depth_pub.publish(depth_data);
    }
    else{}
 // depth = 0.001*cv_ptr->image.at<u_int16_t>(point_target_pixel.y, point_target_pixel.x);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "yolo_zahyou");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    ros::Subscriber urg_sub = nh.subscribe("yolov5/detections", 100,detection_Callback);
    image_transport::Subscriber image_sub_depth = it.subscribe("/camera/aligned_depth_to_color/image_raw", 1, depth_Callback);
    depth_pub = nh.advertise<wrs_fcsc_2023::yolo_detect>("/person_depth", 100);//publish 座標と人数を送信
    ros::spin();
    return 0;
}