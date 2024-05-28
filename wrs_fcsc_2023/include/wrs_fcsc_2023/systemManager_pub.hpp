#include "wrs_fcsc_2023/systemManager_declare.hpp"
ros::Publisher pub_flag_ur3;

void pub_flag(int num)
{
    std_msgs::Int8 out;

    out.data = num;
    pub_flag_ur3.publish(out);
}