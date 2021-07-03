/*
//类成员函数定义
#include "smartcar/ChasecarControl.hpp"
#include "smartcar/dubins.hpp"
#include <iostream>


ChasecarControl::ChasecarControl(ros::NodeHandle& nodeHandle):nh_(nodeHandle)
{
    chase_ctrl_srv_=nh_.advertiseService("chase_ctrl",&ChasecarControl::chaseCtrlSrvCallback, this);
    odom_sub_=nh_.subscribe("/robo2/odom", 100, &ChasecarControl::odomCallback, this);
    cmd_vel_pub_=nh_.advertise<geometry_msgs::Twist>("/robo2/cmd_vel",1);

    //ROS_INFO_STREAM("p_angle:"<<p_angle<<" p_linear:"<<p_linear);
}

ChasecarControl::PathPoint printConfiguration(double q[3], double x, void* user_data)
{
    ChasecarControl::PathPoint p;
    printf("%f, %f, %f, %f\n", q[0], q[1], q[2], x);
    p.x = q[0];
    p.y = q[1];
    p.theta = q[3];
    return p;
}

//chaseCtrlSrvCallback
bool ChasecarControl::chaseCtrlSrvCallback(smartcar::mess::Request& req, smartcar::mess::Response& res)
{
    tar_pose_.position.x=req.targetX;  //获取目标
    tar_pose_.position.y=req.targetY;
    ROS_INFO_STREAM("accept targetX:"<<req.targetX<<", targetY:"<<req.targetY);

    //生成dubins路径，按设定步长采点
    ros::spinOnce();
    double q0[] = {cur_position_.x, cur_position_.y, cur_rpy_.y};  //起点
    double q1[] = {tar_pose_.position.x, tar_pose_.position.y, 3.142}; //终点
    double turning_radius = 0.5;
    DubinsPath path;
    dubins_shortest_path( &path, q0, q1, turning_radius);
    PathPoint p_;
    double q[3];
    double x = 0.0;
    double stepSize = 0.5;  //设置步长
    double length = dubins_path_length(&path);
    std::cout << length << std::endl;
    while(x < length)
    {
        dubins_path_sample(&path, x, q);
        p_ = printConfiguration(q, x, NULL);
        path_point.push_back(p_);
        cout << p_.x << " " << p_.y << " " << p_.theta << endl;
        x += stepSize;
    }
    int num = path_point.size();
    int count = 0;

    res.result=true;
    in_process = true;
    double omega, v;  //角速度、速度
    double begin = ros::Time::now().toSec(); //计时开始

    while (true)
    {
    	if(!in_process)
	    {
	        break;
	    }
        float distance = sqrt(pow(tar_pose_.position.x - cur_position_.x, 2) 
                        + pow(tar_pose_.position.y - cur_position_.y, 2));  //到目标距离
        if(distance < 0.1)  //到达目标
        {
            geometry_msgs::Twist vel_msg;
            vel_msg.angular.z = 0;
            vel_msg.linear.x = 0;
            cmd_vel_pub_.publish(vel_msg);

            in_process = false;
            ROS_INFO_STREAM("Tracking done!");
        }
        //未到达目标，在航程中，控制小车循迹
        tar_now.x = path_point[count].x;
        tar_now.y = path_point[count].y;
        tar_now.theta = path_point[count].theta;
        float dis = sqrt(pow(tar_now.x - cur_position_.x, 2) 
                    + pow(tar_now.y - cur_position_.y, 2));  //到阶段点距离
        if(dis < 0.1)  //到达阶段点
        {
            count = count + 1;  //下一个点
            tar_now.x = path_point[count].x;
            tar_now.y = path_point[count].y;
            tar_now.theta = path_point[count].theta;
            cout << "next:" << tar_now.x << " " << tar_now.y << endl;
            omega = p_angle * (atan2(tar_now.y - cur_position_.y,
                                     tar_now.x - cur_position_.x) - cur_rpy_.y);
            v = 1;
        }
        else
        {
            omega = p_angle * (atan2(tar_now.y - cur_position_.y,
                                     tar_now.x - cur_position_.x) - cur_rpy_.y);
            v = 1;
        }
        geometry_msgs::Twist vel_msg;
        vel_msg.angular.z = omega;
        vel_msg.linear.x = v;
        cmd_vel_pub_.publish(vel_msg);

        ros::spinOnce();
    }
    res.timeCost=ros::Time::now().toSec()-begin;
    return true;
}

//odomCallback获取位置信息
void ChasecarControl::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    cur_pose_ = msg->pose.pose;
    cur_position_ = cur_pose_.position;
    //tf变换
    tf::quaternionMsgToTF(cur_pose_.orientation, cur_quat_);
    tf::Matrix3x3(cur_quat_).getRPY(cur_rpy_.r, cur_rpy_.p, cur_rpy_.y);  //四元数转成欧拉角
}
*/


//类成员函数定义
#include "smartcar/ChasecarControl.hpp"

ChasecarControl::ChasecarControl(ros::NodeHandle& nodeHandle):nh_(nodeHandle)
{
    chase_ctrl_srv_=nh_.advertiseService("chase_ctrl",&ChasecarControl::chaseCtrlSrvCallback, this);
    odom_sub_=nh_.subscribe("/robo2/odom", 100, &ChasecarControl::odomCallback, this);
    cmd_vel_pub_=nh_.advertise<geometry_msgs::Twist>("/robo2/cmd_vel",1);

    //ROS_INFO_STREAM("p_angle:"<<p_angle<<" p_linear:"<<p_linear);
}

//chaseCtrlSrvCallback
bool ChasecarControl::chaseCtrlSrvCallback(smartcar::mess::Request& req, smartcar::mess::Response& res)
{
    tar_pose_.position.x=req.targetX;  //获取目标
    tar_pose_.position.y=req.targetY;

    ROS_INFO_STREAM("accept targetX:"<<req.targetX<<", targetY:"<<req.targetY);

    res.result=true;
    in_process = true;
    double begin=ros::Time::now().toSec();  //计时开始

    while (true)
    {
    	if(!in_process)
	    {
	        break;
	    }    
    	ros::spinOnce();  //
    }
    res.timeCost=ros::Time::now().toSec()-begin;
    return true;
}

//odomCallback
void ChasecarControl::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    cur_pose_ = msg->pose.pose;
    cur_position_ = cur_pose_.position;
    //tf变换
    tf::quaternionMsgToTF(cur_pose_.orientation, cur_quat_);
    tf::Matrix3x3(cur_quat_).getRPY(cur_rpy_.r, cur_rpy_.p, cur_rpy_.y);

    float distance = sqrt(pow(tar_pose_.position.x - cur_position_.x, 2) 
                        + pow(tar_pose_.position.y - cur_position_.y, 2));

    if(in_process)
    {
        if(distance<0.2)  //到达目标
        {
            geometry_msgs::Twist vel_msg;
            vel_msg.angular.z = 0;
            vel_msg.linear.x = 0;
            cmd_vel_pub_.publish(vel_msg);

            in_process = false;
            ROS_INFO_STREAM("Tracking done! cur_x:" << cur_position_.x
                              << " cur_y:" << cur_position_.y << " cur_yaw:" << cur_rpy_.y);
        }
        else
        {
            geometry_msgs::Twist vel_msg;

            vel_msg.angular.z = p_angle * (atan2(tar_pose_.position.y - cur_position_.y, 
                                tar_pose_.position.x - cur_position_.x) - cur_rpy_.y);
            //vel_msg.linear.x = 1.0 * exp(-2 * vel_msg.angular.z);
            vel_msg.linear.x = 0.65 * sqrt(pow(tar_pose_.position.x-cur_position_.x, 2) +
                                      pow(tar_pose_.position.y-cur_position_.y, 2)); 

            //publish
            cmd_vel_pub_.publish(vel_msg);
        }
    }
}