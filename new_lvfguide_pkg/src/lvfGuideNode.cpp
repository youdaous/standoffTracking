#include "../include/new_lvfguide_pkg/lvfGuide.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <mavros_msgs/Thrust.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Path.h>
#include <cmath>
#include <sensor_msgs/NavSatFix.h>
#include <iostream>
#include <vector>

// 全局状态对象
State hunter_state = {0, 0, 0, 0, 0, 0};
State target_state = {0, 0, 0, 0, 0, 0};
State obstacle_state = {0, 0, 0, 0, 0, 0};

double lonO=121.20980712;//原点的经度
double latO=31.05732962;//原点的纬度

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
    hunter_state.course = atan2(hunter_state.v, hunter_state.u);
    // ROS_INFO("vel_y:%.3f", hunter_state.v);
}

void anglesCallback(const sensor_msgs::Imu& msg)
{
    Quaternion q;
    EulerAngles angle;
    q.x=msg.orientation.x;
    q.y=msg.orientation.y;
    q.z=msg.orientation.z;
    q.w=msg.orientation.w;
    angle = ToEulerAngles(q);//计算出的欧拉角全是弧度
    hunter_state.head = angle.yaw;
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
    // double dx = std::cos(msg.latitude*M_PI/180) * (msg.longitude - lonO) * 111319.9;
    // double dy = (msg.latitude - latO) * 111319.9;
    // obstacle_state.x = dx;
    // obstacle_state.y = dy;
    // myobstacles.vertices.back() = Obstacle(obstacle_state, 10, false);
    // myobstacles.vertices.pop_back();
    // myobstacles.vertices.push_back(Obstacle(obstacle_state, 10, false));   
}

// 回调函数，订阅obstacle速度
void velCallback_obstacle(const geometry_msgs::TwistStamped& msg)
{
    // obstacle_state.u = msg.twist.linear.x;
    // obstacle_state.v = msg.twist.linear.y;
    // myobstacles.vertices.back() = Obstacle(obstacle_state, 10, false);
    // myobstacles.vertices.pop_back();
    // myobstacles.vertices.push_back(Obstacle(obstacle_state, 10, false));
}

int main(int argc, char **argv)
{  
    // ROS节点初始化
    ros::init(argc, argv, "lvfGuideNode");
    // 创建节点句柄
    ros::NodeHandle n;

    // 发布期望偏航角和实际偏航角，以欧拉角表示
    ros::Publisher yaw_pub = n.advertise<geometry_msgs::PointStamped>("rover1/yawPub", 10);

    // 发送速度和偏航
    ros::Publisher setpoint_pub = n.advertise<mavros_msgs::PositionTarget>("/rover1/mavros/setpoint_raw/local", 10);

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
    
    for(int i=0;i<5;i++)
    { 
        loop_rate.sleep();
    }
    int safe_distance = 20;  // 安全距离m
    double v_d = 2; // 期望速度m/s
    Guide_law lvf_guide;
    double guide_yaw;
    EulerAngles guide_angle;
    double distance;
    double factor = 1;
    bool flag = true;
    while (ros::ok())
    {
        n.param("safe_distance", safe_distance, 20);
        
        // 运行lyapunov VF制导函数
        lvf_guide = Lvf(hunter_state, target_state, factor * safe_distance, v_d);
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
        lvf_guide = lookahead_guide(lvf_guide, hunter_state, target_state, factor * safe_distance, v_d);
        guide_yaw = lvf_guide.psi;

        // 漂移补偿
        double drift = driftCompensation(hunter_state.head, hunter_state.course);
        // double drift = 0;
        guide_yaw = guide_yaw + drift;
        guide_angle.yaw = guide_yaw;
        Quaternion guide_qua = ToQuaternion(guide_angle);

        // 发布setpoint_raw消息
        // mavros_msgs::AttitudeTarget attitude_raw;//发布的期望姿态

        // attitude_raw.orientation.w = guide_qua.w;
        // attitude_raw.orientation.x = guide_qua.x;
        // attitude_raw.orientation.y = guide_qua.y;
        // attitude_raw.orientation.z = guide_qua.z;
        
        // attitude_raw.thrust = thrust;//油门值
        // attitude_raw.type_mask =0b00000111 ;//0b00000111

        // ros::Time current_time = ros::Time::now();
        // attitude_raw.header.stamp = current_time;

        // raw_pub.publish(attitude_raw);


        // double beta = sensor_angle.yaw - course_angle.yaw;
        ROS_INFO("yaw_d:%.3f, yaw_t:%.3f , r:%.3f, yaw_err:%.3f, distance_flag:%d", guide_angle.yaw, hunter_state.course, distance, lvf_guide.yaw_err, flag);
        
        // publish TargetPosition msg
        mavros_msgs::PositionTarget setpoint;
        setpoint.header.stamp = ros::Time::now();
        setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

        // Control mask (indicating which fields are valid)
        setpoint.type_mask =
            mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY | mavros_msgs::PositionTarget::IGNORE_PZ | // Ignore position
            mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ | // Ignore acceleration
            mavros_msgs::PositionTarget::IGNORE_YAW_RATE; // Ignore yaw rate

        // Desired linear velocities (Body NED frame)
        setpoint.velocity.x = 1.3; // Forward velocity
        setpoint.velocity.y = 0.0; // No lateral movement
        setpoint.velocity.z = 0.0; // Maintain altitude

        // Desired yaw angle (in radians)
        setpoint.yaw = guide_yaw; // 90 degrees yaw

        setpoint_pub.publish(setpoint);

        // // 发布期望航向角和实际航向角以分析误差
        // geometry_msgs::PointStamped yaws;
        // yaws.point.x = guide_angle.yaw;
        // yaws.point.y = distance;
        // yaws.point.z = course_angle.yaw - guide_angle.yaw;
        // ros::Time current_time_yawpub = ros::Time::now();
        // yaws.header.stamp = current_time;
        
        // yaw_pub.publish(yaws);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}