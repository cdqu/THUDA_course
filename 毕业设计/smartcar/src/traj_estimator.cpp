//获取目标小车位置，处理位置信息，做预测
#include <ros/ros.h>
#include "gazebo_msgs/GetModelState.h"
#include <vector>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <smartcar/mess.h>
using namespace std;
using namespace cv;


typedef vector<pair<double, Point2f > > TargetObservation;
Eigen::MatrixXd estimateTrajectory(const TargetObservation & obs, int n_poly);


int main(int argc, char **argv)
{
	const int npoly = 4;
	const int dt = 5;
	TargetObservation tar_ob;
	Point2f p, p_kf;

	//Kalman Filter param
	double q = 0.5, sigma = 0.25;
	Eigen::MatrixXd P = Eigen::MatrixXd::Identity(4, 4);
	Eigen::MatrixXd P_ = Eigen::MatrixXd::Zero(4, 4);
	Eigen::MatrixXd S = Eigen::MatrixXd::Zero(2, 2);
	Eigen::MatrixXd K = Eigen::MatrixXd::Zero(4, 2);
	Eigen::VectorXd v = Eigen::VectorXd::Zero(2);
	Eigen::MatrixXd A(4, 4);
	A << 1, 0, dt, 0,
		0, 1, 0, dt,
		0, 0, 1, 0,
		0, 0, 0, 1;
	Eigen::MatrixXd H(2, 4);
	H << 1, 0, 0, 0,
		0, 1, 0, 0;
	Eigen::MatrixXd Q(4, 4);
	Q << q * dt * dt * dt / 3, 0, q * dt * dt / 2, 0,
		0, q * dt * dt * dt / 3, 0, q * dt * dt / 2,
		q * dt * dt / 2, 0, q * dt, 0,
		0, q * dt * dt / 2, 0, q * dt;
	Eigen::MatrixXd R(2, 2);
	R << sigma * sigma, 0,
		0, sigma * sigma;
	Eigen::VectorXd x_ = Eigen::VectorXd::Zero(4);
	Eigen::VectorXd xk = Eigen::VectorXd::Zero(4);
	Eigen::VectorXd yk = Eigen::VectorXd::Zero(2);


	//ros模块初始化
	ros::init(argc, argv, "pose_client");
	ros::NodeHandle n;
	ros::ServiceClient pose_client = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	gazebo_msgs::GetModelState srv;
	srv.request.model_name = "robo1";
	ros::ServiceClient move2point = n.serviceClient<smartcar::mess>("/chase_car_control/chase_ctrl");
	smartcar::mess srv2;

	ros::Rate loop_rate(dt);

	random_device rd;
	default_random_engine generator{rd()};
	normal_distribution<double> guss_noise(0, sigma);
	int count = 0;
	while(ros::ok())
	{
        if(pose_client.call(srv))  //数据处理模块
    	{
			ROS_INFO("result:[%0.4f, %0.4f]", srv.response.pose.position.x, srv.response.pose.position.y);
			p.x = srv.response.pose.position.x;
			p.y = srv.response.pose.position.y;
			//p.x = srv.response.pose.position.x + guss_noise(generator);
			//p.y = srv.response.pose.position.y + guss_noise(generator);
			//cout << "noise:" << p.x << "," << p.y << endl;

			//Kalman Filter
			if(count==0)
			{
				xk(0) = srv.response.pose.position.x;
				xk(1) = srv.response.pose.position.y;
			}
			else
			{
				x_ = A * xk;
				P_ = A * P * A.transpose() + Q;
				S = H * P_ * H.transpose() + R;
				K = P_ * H.transpose() * S.inverse();
				yk(0) = p.x;
				yk(1) = p.y;
				v = yk - H * x_;
				xk = x_ + K * v;
				P = P_ - K * S * K.transpose();
			}
			p_kf.x = xk(0);
			p_kf.y = xk(1);
			//cout << "KF:" << xk(0) << "," << xk(1) << endl;

			tar_ob.push_back(make_pair(count / dt, p));  //参数p_kf表示进行KF
			ROS_INFO("num:%d",tar_ob.size());
			cout << count << endl;
			
			if(count == 15 * dt)   //观察15秒
			{
				Eigen::MatrixXd param;
				param = estimateTrajectory(tar_ob, npoly);
				cout << param << endl;
				float t_pre = 35.0;  //希望预测的时间点 40 x+1 p=1 v0.5
				Eigen::VectorXd T_pre(npoly);
				Eigen::Vector2d point_pre;
				T_pre(0) = 1.0;
				for(int i = 1; i < npoly; ++i)
				{
					T_pre(i) = T_pre(i - 1) * t_pre;
				}
				point_pre = T_pre.transpose() * param;
				
				srv2.request.targetX = point_pre(0);
				srv2.request.targetY = point_pre(1);
				cout << srv2.request.targetX << endl;
				cout << srv2.request.targetY << endl;

				srv2.request.targetOrientationZ = 1;  //z为无关项，任意设
				if(move2point.call(srv2))
				{
					ROS_INFO("point send");
				}
				else
				{
					ROS_ERROR("Failed to call service /chase_car_control/chase_ctrl");
				}

			}
		}
        else
        {
            ROS_ERROR("Failed to call service /gazebo/get_model_state");
        }
    	loop_rate.sleep();
		count = count + 1;
	}
	return 0;
}


//拟合函数
Eigen::MatrixXd estimateTrajectory(const TargetObservation & obs, int n_poly)
{
	double _lambda = 0.01;
    double _pdt_time = 5;
	const int n_min = 2;

	Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_poly, n_poly);
	Eigen::MatrixXd b = Eigen::MatrixXd::Zero(n_poly, 2);
	Eigen::VectorXd T(n_poly);
	Eigen::VectorXd poi(2);

	{ // the estimation error
		double w = 0;
        for (const auto & ob: obs)
        {
            T(0) = 1.0;
            for (int i = 1; i < n_poly; ++i)
			{
				T(i) = T(i - 1) * ob.first;
			}
            w = 1.0;
            A += T * T.transpose() * w;
			poi(0) = ob.second.x;
			poi(1) = ob.second.y;
			b += T * poi.transpose() * w;
		}
        A /= w;
        b /= w;
	}

	{ // regulator
        Eigen::MatrixXd C = Eigen::MatrixXd::Zero(n_poly, n_poly);
        auto power = [] (double x, int n) -> double
        {
            double ret = 1.0;
            while (n)
            {
                if (n & 1) ret *= x;
                x *= x;
                n >>= 1;
            }
            return ret;
        };

        auto NDiffK = [](int n, int k) -> int
        {
            static int factorial[] = {1, 1, 2, 6, 24, 120, 720, 5040};
            return factorial[n]/factorial[n - k];
        };

        for (int i = n_min; i < n_poly; ++i)
        	for (int j = n_min; j < n_poly; ++j)
                C(i, j) = NDiffK(i, n_min) * NDiffK(j, n_min) * 
                       	 	(power(_pdt_time, i + j - n_min - n_min + 1) - 
                            power(obs.front().first, i + j - n_min - n_min + 1))/
                            (i + j - n_min - n_min + 1);
                //A += _lambda * C * obs.size();
                double expected_obs = abs(obs.front().first * 20.0);
                A =  A + _lambda * obs.size() * C;
    }

	Eigen::MatrixXd A_tmp = A.transpose() * A;
	Eigen::MatrixXd b_tmp = A.transpose() * b;
	Eigen::MatrixXd ret =  A_tmp.ldlt().solve(b_tmp);

	return ret;
}


/*
//KF滤波，减小测量误差
Eigen::MatrixXd my_kalman(Eigen::MatrixXd ob_matrix, int N, double dt)
{
	double q = 1, sigma = 0.5;
	Eigen::MatrixXd P = Eigen::MatrixXd::Identity(4, 4);
	Eigen::MatrixXd P_ = Eigen::MatrixXd::Zero(4, 4);
	Eigen::MatrixXd S = Eigen::MatrixXd::Zero(2, 2);
	Eigen::MatrixXd K = Eigen::MatrixXd::Zero(4, 2);
	Eigen::VectorXd v = Eigen::VectorXd::Zero(2);
	Eigen::MatrixXd A(4, 4);
	A << 1, 0, dt, 0,
		0, 1, 0, dt,
		0, 0, 1, 0,
		0, 0, 0, 1;
	Eigen::MatrixXd H(2, 4);
	H << 1, 0, 0, 0,
		0, 1, 0, 0;
	Eigen::MatrixXd Q(4, 4);
	Q << q * dt * dt * dt / 3, 0, q * dt * dt / 2, 0,
		0, q * dt * dt * dt / 3, 0, q * dt * dt / 2,
		q * dt * dt / 2, 0, q * dt, 0,
		0, q * dt * dt / 2, 0, q * dt;
	Eigen::MatrixXd R(2, 2);
	R << sigma * sigma, 0,
		0, sigma * sigma;
	Eigen::VectorXd x_ = Eigen::VectorXd::Zero(4);
	Eigen::VectorXd xk = Eigen::VectorXd::Zero(4);
	Eigen::VectorXd yk = Eigen::VectorXd::Zero(2);
	xk(0) = ob_matrix(0, 0);
	xk(1) = ob_matrix(1, 0);
	Eigen::MatrixXd x_km = Eigen::MatrixXd::Zero(2,N);
	x_km(0) = ob_matrix(0, 0);
	x_km(1) = ob_matrix(1, 0);

	for (int k = 1; k < N; k++)
	{
		x_ = A * xk;
		P_ = A * P * A.transpose() + Q;
		S = H * P_ * H.transpose() + R;
		K = P_ * H.transpose() * S.inverse();
		yk(0) = ob_matrix(0, k);
		yk(1) = ob_matrix(1, k);
		v = yk - H * x_;
		xk = x_ + K * v;
		P = P_ - K * S * K.transpose();

		x_km(0, k) = xk(0);
		x_km(1, k) = xk(1);
	}

	return x_km;
}
*/