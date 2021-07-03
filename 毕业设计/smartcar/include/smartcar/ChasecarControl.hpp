#include <ros/ros.h>
#include "gazebo_msgs/GetModelState.h"
#include <tf/tf.h>
#include <smartcar/mess.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <vector>
using namespace std;

#define PI -3.1415926

class ChasecarControl
{
private:
    ros::NodeHandle nh_;

    //service 接收目标位置信息，进入回调函数
    ros::ServiceServer chase_ctrl_srv_;
    bool chaseCtrlSrvCallback(smartcar::mess::Request& req, 
                              smartcar::mess::Response& res);

    //subcriber 订阅里程计消息，用于后续tf变换
    ros::Subscriber odom_sub_;
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    //publisher 发布速度控制
    ros::Publisher cmd_vel_pub_;

    geometry_msgs::Pose cur_pose_;  //位姿: Position + Orientation(quaternion)
    geometry_msgs::Pose tar_pose_;
    geometry_msgs::Point cur_position_;
    tf::Quaternion cur_quat_;

    struct RPY
    {
        double r;
        double p;
        double y;   
    };
    RPY cur_rpy_;

    bool in_process=false;
    float p_angle=1.5;

public:
    ChasecarControl(ros::NodeHandle& nh);
    struct PathPoint
    {
        double x;
        double y;
        double theta;
    };
    vector<PathPoint> path_point;  //记录规划路径
    PathPoint tar_now;  //阶段性目标
    //int printConfiguration(double q[3], double x, void *user_data);
};
