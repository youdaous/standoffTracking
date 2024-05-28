#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Thrust.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <sensor_msgs/Imu.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <cmath>
#include <sensor_msgs/NavSatFix.h>
#include <tf2/utils.h>
#include <iostream>
#include <vector>
#include "lvfguide_pkg/newpath.h"

// 声明全局变量
// 状态量结构体
struct State {
    double x, y, u, v;
};
// 制导命令结构体
struct Guide_law {
    double psi, u, v, r, yaw_err;
};
// 四元数结构体
struct Quaternion {
    double w, x, y, z;
};
// 欧拉角结构体
struct EulerAngles {
    double roll, pitch, yaw;
};
// 障碍源的圆形表示结构体
struct Obstacle {
    State obstacle_state;
    double radius;
    bool collision_flag;
    Obstacle(State _obstacle_state, double _radius, bool _collision): obstacle_state(_obstacle_state), radius(_radius), collision_flag(_collision) {};
};
// 障碍集合结构体
struct Obstacles_set {
    std::vector<Obstacle> vertices;
};
// 多边形边界点
struct Point {
    double x, y;
    Point(double _x, double _y) : x(_x), y(_y) {}
};
// 多边形点集合
struct Boundary {
    std::vector<Point> vertices;
};
// dubinpath起点和终点
struct dubin_start_end
{
    geometry_msgs::PoseStamped startPose;
    geometry_msgs::PoseStamped goalPose;
};

// 初始化状态
State hunter_state = {0, 0, 0, 0};  // 追捕船状态(local系)
State target_state = {60, 22.5, 0, 0};  // 目标船状态(local系)
State obstacle_state = {0, 0, 0, 0};    // 障碍船状态(local系)
Obstacles_set myobstacles;
int safe_distance = 20;  // 安全距离m
double v_d = 3; // 期望速度m/s
double lonO=121.20980712;//原点的经度
double latO=31.05732962;//原点的纬度
double thrust = 1;
EulerAngles guide_angle = {0, 0, 0}; // 制导期望欧拉角
EulerAngles sensor_angle = {0, 0, 0};   // 自身姿态欧拉角
EulerAngles course_angle = {0, 0, 0};   // 航向角

// 定义函数：四元数-->欧拉角
EulerAngles ToEulerAngles(Quaternion& q) {
    EulerAngles angles;
 
    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);
 
    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);
 
    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);
 
    return angles;
}
// 定义函数：欧拉角-->四元数
Quaternion ToQuaternion(EulerAngles& angles) // yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
    double cy = cos(angles.yaw * 0.5);
    double sy = sin(angles.yaw * 0.5);
    double cp = cos(angles.pitch * 0.5);
    double sp = sin(angles.pitch * 0.5);
    double cr = cos(angles.roll * 0.5);
    double sr = sin(angles.roll * 0.5);
 
    Quaternion q;
    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;
 
    return q;
}

// 障碍碰撞检测函数
Obstacle Is_collision(const State& hunter, double& guide_yaw, const Obstacles_set& obstacle_set)
{
    int vertexCount = obstacle_set.vertices.size();
    Obstacle return_obstacle({0, 0, 0, 0}, 0, false);
    double pre_hunter_x = hunter.x + cos(guide_yaw) * v_d * 2;
    double pre_hunter_y = hunter.y + sin(guide_yaw) * v_d * 2;
    for (int i = 0; i < vertexCount; i++)
    {
        double dis = sqrt(pow(pre_hunter_x - obstacle_set.vertices[i].obstacle_state.x, 2) + pow(pre_hunter_y - obstacle_set.vertices[i].obstacle_state.y, 2));
        if (dis < (obstacle_set.vertices[i].radius))
        {
            return_obstacle = obstacle_set.vertices[i];
            return_obstacle.collision_flag = true;
            break;
        }
        else
        {
            continue;
        }
    }
    return return_obstacle;
}

// 越界检测函数(判断点是否在多边形内)
bool Is_in_boundary(const State& hunter, double& guide_yaw, const Boundary& boundary)
{
    int vertexCount = boundary.vertices.size();
    bool inside = false;
    double pre_hunter_x = hunter.x + cos(guide_yaw) * v_d * 1;
    double pre_hunter_y = hunter.y + sin(guide_yaw) * v_d * 1;
    Point point(pre_hunter_x, pre_hunter_y);


    for (int i = 0, j = vertexCount - 1; i < vertexCount; j = i++)
    {
        if ((boundary.vertices[i].y > point.y) != (boundary.vertices[j].y > point.y) &&
            (point.x < (boundary.vertices[j].x - boundary.vertices[i].x) * (point.y - boundary.vertices[i].y) /
            (boundary.vertices[j].y - boundary.vertices[i].y) + boundary.vertices[i].x)) {
            inside = !inside;
        }
    }
    return inside;
}

// dubinpath起点和终点生成函数
dubin_start_end gen_dubin_StartEnd(double xx1,double yy1,double an1,double xx2,double yy2,double an2)
{
    dubin_start_end return_startEnd;
    geometry_msgs::PoseStamped startPose;
    geometry_msgs::PoseStamped goalPose;
    startPose.header.stamp = ros::Time::now();
    startPose.header.frame_id = "map";
    startPose.pose.position.x = xx1;
    startPose.pose.position.y = yy1;
    startPose.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, an1);
    startPose.pose.orientation.x = q.x();
    startPose.pose.orientation.y = q.y();
    startPose.pose.orientation.z = q.z();
    startPose.pose.orientation.w = q.w();


    goalPose.header.stamp = ros::Time::now();
    goalPose.header.frame_id = "map";
    goalPose.pose.position.x = xx2;
    goalPose.pose.position.y = yy2;
    goalPose.pose.position.z = 0.0;
    // double yaw = std::atan2(goalPose.pose.position.y - startPose.pose.position.y, goalPose.pose.position.x - 
    //             startPose.pose.position.x);
    q.setRPY(0, 0, an2);
    goalPose.pose.orientation.x = q.x();
    goalPose.pose.orientation.y = q.y();
    goalPose.pose.orientation.z = q.z();
    goalPose.pose.orientation.w = q.w();
    return_startEnd = {startPose, goalPose};
    return return_startEnd;
}

// 边界dubin路径生成函数
nav_msgs::Path DubinPath_from_boundary(const Boundary& boundary)
{
    nav_msgs::Path return_path;
    guaijiao::SimpleDubinsPath my_dubin;

    return_path.poses.clear();///可能要改
    int vertexCount = boundary.vertices.size();

    for (int i = 0; i < vertexCount; i++)
    {
        if(i==0)
        {
            double yaw1 = std::atan2(boundary.vertices[i].y - boundary.vertices[vertexCount-1].y, boundary.vertices[i].x - boundary.vertices[vertexCount-1].x);
            double yaw2 = std::atan2(boundary.vertices[i+1].y - boundary.vertices[i].y, boundary.vertices[i+1].x - boundary.vertices[i].x);
            dubin_start_end dubin_StartEnd = gen_dubin_StartEnd(boundary.vertices[i].x, boundary.vertices[i].y, yaw1, boundary.vertices[i+1].x, boundary.vertices[i+1].y, yaw2);
            my_dubin.makePath(dubin_StartEnd.startPose, dubin_StartEnd.goalPose, return_path);
        }
        else if(i>0 && i<(vertexCount-1))
        {
            double yaw1 = std::atan2(boundary.vertices[i].y - boundary.vertices[i-1].y, boundary.vertices[i].x - boundary.vertices[i-1].x);
            double yaw2 = std::atan2(boundary.vertices[i+1].y - boundary.vertices[i].y, boundary.vertices[i+1].x - boundary.vertices[i].x);
            dubin_start_end dubin_StartEnd = gen_dubin_StartEnd(boundary.vertices[i].x, boundary.vertices[i].y, yaw1, boundary.vertices[i+1].x, boundary.vertices[i+1].y, yaw2);
            my_dubin.makePath(dubin_StartEnd.startPose, dubin_StartEnd.goalPose, return_path);
        }
        else
        {
            double yaw1 = std::atan2(boundary.vertices[i].y - boundary.vertices[i-1].y, boundary.vertices[i].x - boundary.vertices[i-1].x);
            double yaw2 = std::atan2(boundary.vertices[0].y - boundary.vertices[i].y, boundary.vertices[0].x - boundary.vertices[i].x);
            dubin_start_end dubin_StartEnd = gen_dubin_StartEnd(boundary.vertices[i].x, boundary.vertices[i].y, yaw1, boundary.vertices[0].x, boundary.vertices[0].y, yaw2);
            my_dubin.makePath(dubin_StartEnd.startPose, dubin_StartEnd.goalPose, return_path);
        }

    }
    return return_path;

}

// 边界矢量场指导
Guide_law boundary_guide(const State& hunter, const double& guide_yaw, nav_msgs::Path path)
{
    Guide_law guide_law = {0, 0, 0, 0, 0};
    double k = 0.5;
    double u, v;
    std::vector<geometry_msgs::PoseStamped>::iterator closest;
    double minDist = std::numeric_limits<double>::max();
    for (auto it = path.poses.begin(); it != path.poses.end(); it++)
    {
        double dist = std::sqrt(std::pow(hunter.x - it->pose.position.x, 2) + std::pow(hunter.y - it->pose.position.y, 2));
        if (dist < minDist)
        {
        minDist = dist;
        closest = it;
        }
    }
    // Store closest
    geometry_msgs::PoseStamped pose_d = *closest;

    // Erase previous elements
    path.poses.erase(path.poses.begin(), closest);

    // Path tangential angle
    double gamma_p = tf2::getYaw(pose_d.pose.orientation);

    int factor = 1;

    // double err_yaw = guide_yaw - gamma_p;
    // if (err_yaw > M_PI)
    // {
    //     err_yaw = err_yaw -  2 * M_PI;
    // }
    // else if (err_yaw < -M_PI)
    // {
    //     err_yaw = err_yaw + 2 * M_PI;
    // }

    // if (abs(err_yaw) > M_PI_2)
    // {
    //     factor = -1;
    // }
    // ROS_INFO("factor:%d", factor);
    // if (abs(abs(err_yaw) - M_PI_2) < 0.05)
    // {
    //     thrust = 0;
    // }

    double y_e = -(hunter.x - pose_d.pose.position.x) * std::sin(gamma_p) + (hunter.y - pose_d.pose.position.y) * std::cos(gamma_p);
    u = factor * v_d * 2 * sqrt(exp(k * y_e)) / (1 + exp(k * y_e));
    v = v_d * (1 - exp(k * y_e)) / (1 + exp(k * y_e));
    guide_law.psi = gamma_p +  atan2(v, u);
    guide_law.r = y_e;
    guide_law.u = u;
    guide_law.v = v;
    return guide_law;
}

// Lyapunov vector field guidance law
Guide_law Lvf(const State& hunter, const State& target, const double& d)
{
    // distance between hunter and target
    double dx = hunter.x - target.x;
    double dy = hunter.y - target.y;
    double r = sqrt((dx * dx + dy * dy));
    // ROS_INFO("target.u:%.3f, v:%.3f, x:%.3f, y:%.3f", target.u, target.v, target.x, target.y);
    // 计算矢量场参数c
    double c = 1;
    if (r > 2 * d)
    {
        c = 0;
    }
    else
    {
        c = 0.25 * pow((1 - cos((r / d * M_PI))), 2);
    }

    // 计算速度矢量和偏航角，给出制导命令
    Guide_law guide ={0, 0, 0, 0, 0};
    double vx_d, vy_d, v_t;
    double v_hunter = sqrt(hunter.u * hunter.u + hunter.v * hunter.v);
    double v_dc;
    v_dc = v_hunter;
    // 实际速度替换期望速度
    if (v_hunter < 0.8 * v_d)
    {
        v_dc = v_d;
    }

    double lam = -v_dc / (r * sqrt((pow(r, 4) + (c * c - 2) * r * r * d * d + pow(d, 4))));
    vx_d = lam * ((r * r - d * d) * dx + c * r * d * dy);
    vy_d = lam * ((r * r - d * d) * dy - c * r * d * dx);
    guide.r = r;
    v_t = sqrt((pow(target.u, 2) + pow(target.v, 2)));  // 目标速度标量；
    if (v_d <= v_t)
    {
        ROS_INFO("v_d is too small!");
        guide.psi = atan2(vy_d, vx_d);
        guide.u = vx_d;
        guide.v = vy_d;
        return guide;
    }
    double alpha = (-(vx_d * target.u + vy_d * target.v) + 
    sqrt(pow(vx_d * target.u + vy_d * target.v, 2) - pow(v_dc, 2) * ((pow(target.u, 2) + pow(target.v, 2)) - pow(v_dc, 2)))) / pow(v_dc, 2);
    vx_d = target.u + alpha * vx_d;
    vy_d = target.v + alpha * vy_d;
    guide.psi = atan2(vy_d, vx_d);
    guide.u = vx_d;
    guide.v = vy_d;
    // ROS_INFO("u:%.3f, v:%.3f, yaw:%.3f", guide.u, guide.v, guide.psi);
    return guide;
}

// 直线矢量场函数
// Guide_law line_vf()

// 前视补偿函数
Guide_law lookahead_guide(const Guide_law& lvfguide, const double& yaw_t, const State& hunter, const State& target, const double& d)
{
    State hunter_lookahead = hunter;
    double yaw_err = yaw_t - lvfguide.psi;
    double err;

    if (yaw_err > M_PI)
    {
        err = yaw_err -  2 * M_PI;
    }
    else if (yaw_err < -M_PI)
    {
        err = yaw_err + 2 * M_PI;
    }
    else
    {
        err = yaw_err;
    }

    // if (err < 0.05 && err > -0.05)
    // {
    //     ROS_INFO("err is ignored!");
    //     Guide_law lvf_nocorrection = lvfguide;
    //     lvf_nocorrection.yaw_err = err;
    //     return lvf_nocorrection;
    // }

    // const double distance_max = 0.4 * sqrt(pow(hunter.u - target.u, 2) + pow(hunter.v - target.v, 2));
    const double distance_max = sqrt(pow(hunter.u - target.u, 2) + pow(hunter.v - target.v, 2));
    double alpha;
    double lookahead_distace;

    // alpha = abs(err) / M_PI;
    // alpha = 2 / (1 + exp(-2 * abs(err))) - 1;
    // alpha = (1 - cos(abs(err))) * 0.5;
    alpha = (1 - cos(abs(err)));

    lookahead_distace = alpha * distance_max;
    // double dir = atan2(target.y - hunter.y, target.x - hunter.x);
    hunter_lookahead.x = hunter.x + cos(yaw_t) * lookahead_distace;
    hunter_lookahead.y = hunter.y + sin(yaw_t) * lookahead_distace;
    Guide_law lvfguide_lookahead = Lvf(hunter_lookahead, target, d);
    lvfguide_lookahead.yaw_err= err;
    return lvfguide_lookahead;
}

// 偏移补偿函数
double driftCompensation(const double& head_sensor, const double& course_angle)
{
    double beta = head_sensor - course_angle;
    double delta;
    if (beta>0 && beta<M_PI/2)
    {
        delta = sqrt(2 + 1 / (beta*beta)) - 1 / beta;
    }
    else if(beta<0 && beta>-M_PI/2)
    {
        delta = -sqrt(2 + 1 /(beta*beta)) - 1 / beta;
    }
    else
    {
        delta = 0;
    }
    // delta = beta + pow(beta, 3) / 3;
    return delta;
}

// 回调函数，订阅hunter经纬度
void gpsCallback_hunter(const sensor_msgs::NavSatFix& msg)
{

    double dx = std::cos(msg.latitude*M_PI/180) * (msg.longitude - lonO) * 111319.9;
    double dy = (msg.latitude - latO) * 111319.9;
    hunter_state.x = dx;
    hunter_state.y = dy;
}

// 回调函数，订阅hunter速度和并计算航向
void velCallback_hunter(const geometry_msgs::TwistStamped& msg)
{
    hunter_state.u = msg.twist.linear.x;
    hunter_state.v = msg.twist.linear.y;
    course_angle.yaw = atan2(hunter_state.v, hunter_state.u);
}

void anglesCallback(const sensor_msgs::Imu& msg)
{
    Quaternion q;
    q.x=msg.orientation.x;
    q.y=msg.orientation.y;
    q.z=msg.orientation.z;
    q.w=msg.orientation.w;

    sensor_angle = ToEulerAngles(q);//计算出的欧拉角全是弧度
}

// 回调函数，订阅target经纬度
void gpsCallback_target(const sensor_msgs::NavSatFix& msg)
{

    double dx = std::cos(msg.latitude*M_PI/180) * (msg.longitude - lonO) * 111319.9;
    double dy = (msg.latitude - latO) * 111319.9;
    target_state.x = dx;
    target_state.y = dy;
}

// 回调函数，订阅target速度
void velCallback_target(const geometry_msgs::TwistStamped& msg)
{
    target_state.u = msg.twist.linear.x;
    target_state.v = msg.twist.linear.y;
}

// 回调函数，订阅obstacle经纬度
void gpsCallback_obstacle(const sensor_msgs::NavSatFix& msg)
{
    double dx = std::cos(msg.latitude*M_PI/180) * (msg.longitude - lonO) * 111319.9;
    double dy = (msg.latitude - latO) * 111319.9;
    obstacle_state.x = dx;
    obstacle_state.y = dy;
    myobstacles.vertices.back() = Obstacle(obstacle_state, 10, false);
    // myobstacles.vertices.pop_back();
    // myobstacles.vertices.push_back(Obstacle(obstacle_state, 10, false));
    
}

// 回调函数，订阅obstacle速度
void velCallback_obstacle(const geometry_msgs::TwistStamped& msg)
{
    obstacle_state.u = msg.twist.linear.x;
    obstacle_state.v = msg.twist.linear.y;
    myobstacles.vertices.back() = Obstacle(obstacle_state, 10, false);
    // myobstacles.vertices.pop_back();
    // myobstacles.vertices.push_back(Obstacle(obstacle_state, 10, false));
}

int main(int argc, char **argv)
{
    // 命令行参数
    bool obstacle_check_flag = false;
    bool boundary_check_flag = false;
    switch (argc)
    {
    case 2:
        safe_distance = atof(argv[1]);
        break;

    case 3:
        safe_distance = atof(argv[1]);
        v_d = atof(argv[2]);
        break;

    case 4:
        safe_distance = atof(argv[1]);
        v_d = atof(argv[2]);
        if (atof(argv[3]) == 1)
        {
            obstacle_check_flag = true;
        }
        
        break;

    case 5:
        safe_distance = atof(argv[1]);
        v_d = atof(argv[2]);
        if (atof(argv[3]) == 1)
        {
            obstacle_check_flag = true;
        }
        if (atof(argv[4]) == 1)
        {
            boundary_check_flag = true;
        }
        break;
    
    default:
        break;
    }
    
    // ROS节点初始化
    ros::init(argc, argv, "move_lvfguide_node");
    // 创建节点句柄
    ros::NodeHandle n;

    // 发布边界轨迹，rviz可视化
    ros::Publisher boundary_pub = n.advertise<nav_msgs::Path>("boundary_path", 1000);

    // 发布期望偏航角和实际偏航角，以欧拉角表示
    ros::Publisher yaw_pub = n.advertise<geometry_msgs::PointStamped>("rover1/yawPub", 10);

    //发送raw航向信息
    ros::Publisher raw_pub = n.advertise<mavros_msgs::AttitudeTarget>("rover1/mavros/setpoint_raw/attitude",10);

    // // 发送高级航向信息
    // ros::Publisher attitude_pub = n.advertise<geometry_msgs::PoseStamped>("rover1/mavros/setpoint_attitude/attitude",10);

    // // 发送油门信息
    // ros::Publisher thrust_pub = n.advertise<mavros_msgs::Thrust>("rover1/mavros/setpoint_attitude/thrust",10);

    // 发送速度
    // ros::Publisher vel_pub = n.advertise<geometry_msgs::TwistStamped>("rover1/mavros/setpoint_velocity/cmd_vel", 10);

    // 创建一个Subscriber，订阅名为mavros/global_position/global的topic，注册回调函数poseCallback 订阅hunter经纬度
    ros::Subscriber gps_sub = n.subscribe("rover1/mavros/global_position/global", 10, gpsCallback_hunter);

    // 订阅hunter速度消息
    ros::Subscriber vel_sub = n.subscribe("/rover1/mavros/local_position/velocity_local", 10, velCallback_hunter);

    // 订阅target经纬度
    ros::Subscriber gps_sub_target = n.subscribe("rover2/mavros/global_position/global", 10, gpsCallback_target);

    // 订阅target速度消息
    ros::Subscriber vel_sub_target = n.subscribe("rover2/mavros/local_position/velocity_local", 10, velCallback_target);

    // 创建一个Subscriber，订阅名为mavros/imu/data的topic，注册回调函数poseCallback 订阅四元数
    ros::Subscriber angles_sub = n.subscribe("rover1/mavros/imu/data", 10, anglesCallback);

    // 订阅障碍船obstacle经纬度
    ros::Subscriber gps_sub_obstacle = n.subscribe("rover3/mavros/global_position/global", 10, gpsCallback_obstacle);

    // 订阅障碍船obstacl速度
    ros::Subscriber vel_sub_obstacle = n.subscribe("rover3/mavros/local_position/velocity_local", 10, velCallback_obstacle);

    // 设置循环的频率
    ros::Rate loop_rate(10);

    // 初始化边界路径
    Boundary myboundary;

    // Point boundary_points[] = {Point(0, 0), Point(190, -55), Point(207, 32), Point(285, 122), Point(360, 185), Point(205, 185),
    //                             Point(128, 160), Point(70, 180), Point(90, 90), Point(50, 40), Point(5, 33), Point(0, 0)};
    Point boundary_points[] = {Point(82.8, 35.4), Point(150, -15), Point(217.2, 74.6), Point(150, 125)};
    int length = sizeof(boundary_points) / sizeof(boundary_points[0]);
   
    for (int i = 0; i < length; i++)
    {
        myboundary.vertices.push_back(boundary_points[i]);
    }

    nav_msgs::Path myboundaryPath = DubinPath_from_boundary(myboundary);

    // 初始化手动障碍集合
    
    // myobstacles.vertices.push_back(Obstacle({70, 25, 0, 0}, 10, false));
    // myobstacles.vertices.push_back(Obstacle({75, 11, 0, 0}, 20, false));
    // myobstacles.vertices.push_back(Obstacle({140, -15, 0, 0}, 12.5, false));
    // 观测运动障碍初始化
    myobstacles.vertices.push_back(Obstacle({-100, -100, 0, 0}, 15, false));
    

    for(int i=0;i<5;i++)
    { 
        loop_rate.sleep();
    }
    Guide_law lvf_guide;
    double guide_yaw;
    double distance;
    double factor = 1;
    bool flag = true;
    while (ros::ok())
    {
        boundary_pub.publish(myboundaryPath);
        // target_state = {210, 75, 0, 0};
        // 运行lyapunov VF制导函数
        lvf_guide = Lvf(hunter_state, target_state, factor * safe_distance);
        distance = lvf_guide.r;

        // 相对距离硬约束
        if ((distance < (0.95 * safe_distance)) && flag)
        {
            flag = false;
            factor = 1.05;
            ROS_INFO("flag=%d!!\n", flag);
        }
        else if (!flag && (distance > (1.025 * safe_distance)))
        {
            flag = true;
            factor = 1;
            ROS_INFO("flag=%d!!\n", flag);
        }

        // 前视补偿
        lvf_guide = lookahead_guide(lvf_guide, course_angle.yaw, hunter_state, target_state, factor * safe_distance);
        guide_yaw = lvf_guide.psi;

        // 边界检测
        // thrust = 1;
        if (boundary_check_flag)
        {
            double target_yaw = atan2(target_state.v, target_state.u);
            if (!Is_in_boundary(hunter_state, guide_yaw, myboundary))
            // if (true)
            {
                ROS_INFO("Cross boundary!!!");
                Guide_law lvf_boundary = boundary_guide(hunter_state, target_yaw, myboundaryPath);
                guide_yaw =lvf_boundary.psi;
                ROS_INFO("boundary distance:%.3f", lvf_boundary.r);
            }
        }

        // 障碍检测
        if (obstacle_check_flag)
        {
            Obstacle check_obstacle = Is_collision(hunter_state, guide_yaw, myobstacles);
            if (check_obstacle.collision_flag)
            {
                ROS_INFO("avoid obstacle!!!");
                Guide_law lvf_obstacle = Lvf(hunter_state, check_obstacle.obstacle_state, check_obstacle.radius);
                double to_obstacleCenter_angle = atan2(check_obstacle.obstacle_state.y - hunter_state.y, check_obstacle.obstacle_state.x - hunter_state.x);
                if (to_obstacleCenter_angle < guide_yaw)
                {
                    ROS_INFO("CW!!");
                    lvf_obstacle.psi = 2 * to_obstacleCenter_angle - lvf_obstacle.psi;
                    if (lvf_obstacle.psi > M_PI)
                    {
                        lvf_obstacle.psi = lvf_obstacle.psi -  2 * M_PI;
                    }
                    else if (lvf_obstacle.psi < -M_PI)
                    {
                        lvf_obstacle.psi = lvf_obstacle.psi + 2 * M_PI;
                    }
                    
                }
                // double obstacle_distance = lvf_obstacle.r;
                // lvf_guide = lvf_obstacle;
                guide_yaw = lvf_obstacle.psi;
                ROS_INFO("obstacle distance:%.3f", lvf_obstacle.r);
            }
        }

        
        // 漂移补偿
        double drift = driftCompensation(sensor_angle.yaw, course_angle.yaw);
        // double drift = 0;
        guide_yaw = guide_yaw + drift;
        guide_angle.yaw = guide_yaw;
        Quaternion guide_qua = ToQuaternion(guide_angle);

        // 发布setpoint_raw消息
        mavros_msgs::AttitudeTarget attitude_raw;//发布的期望姿态

        attitude_raw.orientation.w = guide_qua.w;
        attitude_raw.orientation.x = guide_qua.x;
        attitude_raw.orientation.y = guide_qua.y;
        attitude_raw.orientation.z = guide_qua.z;
        
        attitude_raw.thrust = thrust;//油门值
        attitude_raw.type_mask =0b00000111 ;//0b00000111

        ros::Time current_time = ros::Time::now();
        attitude_raw.header.stamp = current_time;

        raw_pub.publish(attitude_raw);

        // // 发布setpoint_attitude和setpoint_attitude/thrust消息
        // geometry_msgs::PoseStamped attitude;
        // mavros_msgs::Thrust thrust;

        // attitude.pose.orientation.w = guide_qua.w;
        // attitude.pose.orientation.x = guide_qua.x;
        // attitude.pose.orientation.y = guide_qua.y;
        // attitude.pose.orientation.z = guide_qua.z;
        // thrust.thrust = 1.0;
        // ros::Time current_time = ros::Time::now();
        // attitude.header.stamp = current_time;
        // thrust.header.stamp = current_time;

        // attitude_pub.publish(attitude);
        // thrust_pub.publish(thrust);
        double beta = sensor_angle.yaw - course_angle.yaw;
        ROS_INFO("yaw_d:%.3f, yaw_t:%.3f , r:%.3f, yaw_err:%.3f, flag:%d", lvf_guide.psi, sensor_angle.yaw, distance, lvf_guide.yaw_err, flag);
        // ROS_INFO("drift:%.3f, beta:%.3f", drift, beta);
        // // 创建TwistStamped消息
        // geometry_msgs::TwistStamped velocity_msg;
        // double u_body = lvf_guide.u * cos(sensor_angle.yaw) - lvf_guide.v * sin(sensor_angle.yaw);
        // double v_body = lvf_guide.u * sin(sensor_angle.yaw) + lvf_guide.v * cos(sensor_angle.yaw);
        
        // // 设置线速度和角速度
        // velocity_msg.twist.linear.x = u_body; // 1 m/s in the x-axis
        // velocity_msg.twist.linear.y = v_body; // 0.5 rad/s in the z-axis
        // ROS_INFO("u:%.3f, v:%.3f, yaw:%.3f", u_body, v_body, sensor_angle.yaw);
        
        // // 发布速度消息
        // vel_pub.publish(velocity_msg);
        // // ROS_INFO("x:%.3f, y:%.3f , yaw:%.3f", hunter_state.x, hunter_state.y, guide_angle.yaw);

        // 发布期望航向角和实际航向角以分析误差
        geometry_msgs::PointStamped yaws;
        yaws.point.x = guide_angle.yaw;
        yaws.point.y = distance;
        yaws.point.z = course_angle.yaw - guide_angle.yaw;
        ros::Time current_time_yawpub = ros::Time::now();
        yaws.header.stamp = current_time;
        
        yaw_pub.publish(yaws);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
