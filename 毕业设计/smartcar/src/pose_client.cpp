//获取目标小车位置，处理位置信息，做预测
#include <ros/ros.h>
#include "gazebo_msgs/GetModelState.h"
#include <queue>
#include <iostream>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

Mat polyfit(queue<Point2f>& in_point, int m);


int main(int argc, char **argv)
{
    queue<Point2f> pos_q, copy_q;  //创建vector存储历史点位
    Point2f p;
    int m = 6;

    ros::init(argc, argv, "pose_client");

    ros::NodeHandle n;

    ros::ServiceClient pose_client = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    gazebo_msgs::GetModelState srv;
    srv.request.model_name = "robo1";

    ros::Rate loop_rate(1);

    while(ros::ok())
    {
        if(pose_client.call(srv))  //数据处理模块
        {
            ROS_INFO("result:[%0.4f, %0.4f]", srv.response.pose.position.x,   srv.response.pose.position.y);
            p.x = srv.response.pose.position.x;
            p.y = srv.response.pose.position.y;
	    pos_q.push(p);
	    ROS_INFO("num:%d",pos_q.size());
	    if(pos_q.size() == 10)
	    {
            	copy_q = pos_q;
	    	Mat mat_k = polyfit(copy_q, m);
	    	float y;
            	for (int j = 0; j < m + 1; ++j)
	    	{
			y += mat_k.at<float>(j, 0)*pow(p.x,j);
	    	}
	    	ROS_INFO("y:%0.4f", y);
		pos_q.pop();
	    }
        }
        else
        {
            ROS_ERROR("Failed to call service /gazebo/get_model_state");
        }
        loop_rate.sleep();
    }
    return 0;
}


Mat polyfit(queue<Point2f>& in_point, int m)
{
	int size = in_point.size();
	//所求未知数个数
	int x_num = m + 1;
	//构造矩阵U和Y
	Mat mat_u(size, x_num, CV_32F);
	Mat mat_y(size, 1, CV_32F);
	Point2f p_;
	for (int i = 0; i < size; ++i)
	{
		p_ = in_point.front();
		for (int j = 0; j < mat_u.cols; ++j)
		{
			mat_u.at<float>(i, j) = pow(p_.x, j);
		}
		mat_y.at<float>(i, 0) = p_.y;
		in_point.pop();
	}
	cout << mat_u << endl;
	cout << mat_y << endl; 
	//矩阵运算，获得系数矩阵K
	Mat mat_k(x_num, 1, CV_32F);
	mat_k = (mat_u.t()*mat_u).inv()*mat_u.t()*mat_y;
	cout << mat_k << endl;
	return mat_k;
}

