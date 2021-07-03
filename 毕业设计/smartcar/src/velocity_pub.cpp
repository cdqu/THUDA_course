//发布速度指令

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv)
{
    //ROS node init
    ros::init(argc, argv, "velocity_pub");
    //create node handle
    ros::NodeHandle n;
    //create a publisher
    ros::Publisher robo1_vel_pub = n.advertise<geometry_msgs::Twist>("/robo1/cmd_vel",10);

    ros::Rate loop_rate(10);

    double acc = 0;
    int count = 0;

    while(ros::ok())
    {
        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x = 0.2 + count * acc;
        vel_msg.angular.z = 0.02;

        robo1_vel_pub.publish(vel_msg);
        ROS_INFO("publish robot1 velocity command[%0.2f m/s, %0.2f rad/s]",
                                                 vel_msg.linear.x, vel_msg.angular.z);

        loop_rate.sleep(); 
        count = count + 1;
    }
    return 0;
}
