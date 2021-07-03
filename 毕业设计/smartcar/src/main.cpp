#include <ros/ros.h>
#include "smartcar/ChasecarControl.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "chase_car_control");
    ros::NodeHandle nh("~");
    
    ChasecarControl chase_car_control(nh);

    ros::spin();  //收到目标位置消息，进入callback函数
    return 0;
}
