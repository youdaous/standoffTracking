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
#include<sensor_msgs/Imu.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <cmath>
#include <sensor_msgs/NavSatFix.h>
#include <tf2/utils.h>

// 声明全局变量
// 状态量结构体
struct State {
    double x, y, u, v;
};
// 指导命令结构体
struct Guide_law {
    double psi, kappa, r; 
};
// 四元数结构体
struct Quaternion {
    double w, x, y, z;
};
// 欧拉角结构体
struct EulerAngles {
    double roll, pitch, yaw;
};
// 初始化状态
State hunter_state = {0, 0, 0, 0};  // 追捕船状态(local系)
State target_state = {60, 20, 0, 0};  // 目标船状态(local系)
int d = 20;  // 安全距离m
double v_d = 4; // 期望速度m/s
double lonO=121.20980712;//原点的经度
double latO=31.05732962;//原点的纬度
EulerAngles guide_angle = {0, 0, 0};
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

// Lyapunov vector field guidance law
Guide_law Lvf(const State& hunter, const State& target)
{
    // distance between hunter and target
    double dx = hunter.x - target.x;
    double dy = hunter.y - target.y;
    double r = sqrt((dx * dx + dy * dy));

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
    Guide_law guide ={0, 0, 0};
    guide.r = r;
    double vx_d, vy_d, v_t, kappa;
    double lam = -v_d / (r * sqrt((pow(r, 4) + (c * c - 2) * r * r * d * d + pow(d, 4))));
    vx_d = lam * ((r * r - d * d) * dx + c * r * d * dy);
    vy_d = lam * ((r * r - d * d) * dy - c * r * d * dx);
    // kappa = c * pow(d, 3) * ((c*c-2) * r*r + 2 * d*d) / pow((pow(r, 4) + (c * c - 2) * r * r * d * d + pow(d, 4)), 1.5);
    // guide.kappa = kappa;
    v_t = sqrt((pow(target.u, 2) + pow(target.v, 2)));  // 目标速度标量；
    if (v_d <= v_t)
    {
        guide.psi = atan2(vy_d, vx_d);
        return guide;
    }
    double alpha = (-(vx_d * target.u + vy_d * target.v) + 
    sqrt(pow(vx_d * target.u + vy_d * target.v, 2) - pow(v_d, 2) * ((pow(target.u, 2) + pow(target.v, 2)) - pow(v_d, 2)))) / pow(v_d, 2);
    vx_d = target.u + alpha * vx_d;
    vy_d = target.v + alpha * vy_d;
    guide.psi = atan2(vy_d, vx_d);
    return guide;
}

// 前视补偿函数
Guide_law lookahead_guide(const Guide_law& lvfguide, const double& yaw_t, const State& hunter, const State& target)
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

    if (err < 0.15 && err > -0.15)
    {
        
        Guide_law lvf_nocorrection = lvfguide;
        lvf_nocorrection.kappa = err;
        return lvf_nocorrection;
    }

    const double distance_max = 0.5;
    double alpha;
    double lookahead_distace;

    // alpha = abs(err) / M_PI;
    // alpha = 2 / (1 + exp(-2 * abs(err))) - 1;
    alpha = (1 - cos(abs(err))) * 0.5;

    lookahead_distace = alpha * distance_max;
    // double dir = atan2(target.y - hunter.y, target.x - hunter.x);
    hunter_lookahead.x = hunter.x + cos(yaw_t) * lookahead_distace;
    hunter_lookahead.y = hunter.y + sin(yaw_t) * lookahead_distace;
    Guide_law lvfguide_lookahead = Lvf(hunter_lookahead, target);
    lvfguide_lookahead.kappa = err;
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
    return delta;
}

// 回调函数
void gpsCallback_hunter(const sensor_msgs::NavSatFix& msg)
{

    double dx = std::cos(msg.latitude*M_PI/180) * (msg.longitude - lonO) * 111319.9;
    double dy = (msg.latitude - latO) * 111319.9;
    hunter_state.x = dx;
    hunter_state.y = dy;
}

// 回调函数
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

int main(int argc, char **argv)
{

    // ROS节点初始化
    ros::init(argc, argv, "lvfguide_node");
    // 创建节点句柄
    ros::NodeHandle n;

    // 发布期望偏航角和实际偏航角，以欧拉角表示
    ros::Publisher yaw_pub = n.advertise<geometry_msgs::PointStamped>("rover1/yawPub", 10);

    //发送航向信息
    ros::Publisher raw_pub = n.advertise<mavros_msgs::AttitudeTarget>("rover1/mavros/setpoint_raw/attitude",10);

    // // 发送高级航向信息
    // ros::Publisher attitude_pub = n.advertise<geometry_msgs::PoseStamped>("rover1/mavros/setpoint_attitude/attitude",10);

    // // 发送油门信息
    // ros::Publisher thrust_pub = n.advertise<mavros_msgs::Thrust>("rover1/mavros/setpoint_attitude/thrust",10);

    // 发送速度
    ros::Publisher vel_pub = n.advertise<geometry_msgs::TwistStamped>("rover1/mavros/setpoint_velocity/cmd_vel", 10);

    // 创建一个Subscriber，订阅名为mavros/global_position/global的topic，注册回调函数poseCallback 订阅hunter经纬度
    ros::Subscriber gps_sub = n.subscribe("rover1/mavros/global_position/global", 10, gpsCallback_hunter);

    // 订阅hunter速度消息
    ros::Subscriber vel_sub = n.subscribe("/rover1/mavros/local_position/velocity_local", 10, velCallback_hunter);

    // 订阅hunter姿态消息
    ros::Subscriber angles_sub = n.subscribe("rover1/mavros/imu/data", 10, anglesCallback);

    // 设置循环的频率
    ros::Rate loop_rate(10);
    

    for(int i=0;i<5;i++)
    { 
        loop_rate.sleep();
    }

    while (ros::ok())
    {
        Guide_law lvf_guide = Lvf(hunter_state, target_state);
        
        // if (abs(sensor_angle.yaw - lvf_guide.psi) > 0.01)
        
        // lvf_guide = lookahead_guide(lvf_guide, course_angle.yaw, hunter_state, target_state);

        // double drift = driftCompensation(sensor_angle.yaw, course_angle.yaw);
        double drift = 0;
        double guide_yaw = lvf_guide.psi + drift;
        guide_angle.yaw = guide_yaw;
        Quaternion guide_qua = ToQuaternion(guide_angle);
        // 发布setpoint_raw消息
        mavros_msgs::AttitudeTarget attitude_raw;//发布的期望姿态

        attitude_raw.orientation.w = guide_qua.w;
        attitude_raw.orientation.x = guide_qua.x;
        attitude_raw.orientation.y = guide_qua.y;
        attitude_raw.orientation.z = guide_qua.z;
        // attitude_raw.body_rate.z = v_d * lvf_guide.kappa;   // yaw_date
        attitude_raw.thrust = 1.0;//油门值
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
        ROS_INFO("yaw_d:%.3f, yaw_t:%.3f , r:%.3f, yaw_err:%.3f", lvf_guide.psi, sensor_angle.yaw, lvf_guide.r, lvf_guide.kappa);
        ROS_INFO("drift:%.3f, beta:%.3f", drift, beta);
        // ROS_INFO("%.3f", -M_PI);

        // 发布期望航向角和实际航向角以分析误差
        geometry_msgs::PointStamped yaws;
        yaws.point.x = lvf_guide.psi;
        yaws.point.y = sensor_angle.yaw;
        yaws.point.z = sensor_angle.yaw - lvf_guide.psi;
        ros::Time current_time_yawpub = ros::Time::now();
        yaws.header.stamp = current_time;
        
        yaw_pub.publish(yaws);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

