#include "wrs_fcsc_2023/systemManager_disposal.hpp"
#include "wrs_fcsc_2023/systemManager_draw.hpp"
#include "wrs_fcsc_2023/systemManager_putback.hpp"
#include "wrs_fcsc_2023/systemManager_pub.hpp"

void flagCb_mobile_robot(const std_msgs::Int8::ConstPtr& flag_msg)
{
    flag_mobile_robot_system = *flag_msg ;
}

void flagCb_customer(const std_msgs::Int8::ConstPtr& flag_msg)
{
    flag_customer_system = *flag_msg ;
}

int main(int argc, char **argv)
{
    // ros setup
    ros::init(argc, argv, "systemManager");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    // publisher
    pub_flag_ur3 = nh.advertise<std_msgs::Int8>("input_flag", 10);

    // subscriber
    ros::Subscriber sub_mobile_robot = nh.subscribe("flag_mobile_robot", 1000, flagCb_mobile_robot);
    ros::Subscriber sub_customer = nh.subscribe("flag_customer", 1000, flagCb_customer);

    image_transport::Subscriber image_sub_color = it.subscribe("/camera/color/image_raw", 1, imageCb_color);
    image_transport::Subscriber image_sub_yolact = it.subscribe("/yolact_ros/visualization", 1, imageCb_yolact);
    image_transport::Subscriber image_sub_depth_draw = it.subscribe("/camera/aligned_depth_to_color/image_raw", 1, imageCb_depth_draw);
    image_transport::Subscriber image_sub_depth_disposal = it.subscribe("/camera/aligned_depth_to_color/image_raw", 1, imageCb_depth_disposal);


    // publisher
    gripperCommand_pub = nh.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("Robotiq2FGripperRobotOutput", 10);



    ros::Rate loop_rate(1000);
    static bool IsFirst = true;
    static bool IsFirstUR3 = true;
    while(ros::ok())
    {
        if(IsFirst)
        {
            cout << "pub flag -1" << endl;
            pub_flag(-1);   //start system

            sleep(10);
            cout << "pub flag 0" << endl;
            pub_flag(0);    // go to shelf

            IsFirst = false;
        }
    
        switch (flag_mobile_robot_system.data)
        {
            case 0: 
                break;

            case 1:
                switch (flag_ur3_system)
                {
                    case 1: // draw shelf
                        draw();
                        break;

                    case 2: // disposal
                        pub_flag(2);    // for mobile robot
                        disposal();
                        break;
                    
                    case 3: // put back shelf
                        putback();
                        break;
                    
                    case 4: // finish task
                        if(flag_ur3_putback == 5)
                            pub_flag(3);    // for mobile robot
                        if(flag_customer_system.data == 0 && flag_system_continue == 1) // retry
                        {
                            // for each flag
                            flag_ur3_draw = 1;
                            flag_ur3_disposal = 1; 
                            flag_ur3_disposal_getImg = 0;
                            flag_ur3_putback = 1; 

                            // retry whole task
                            flag_ur3_system = 1;
                        }
                        else if (flag_system_continue == 0)
                        {
                            pub_flag(1);
                        }
                        break;
                    
                    default:
                        break;
                }
                break;
            
            default:
                break;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}