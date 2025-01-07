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
State leader_state = {0, 0, 0, 0, 0, 0};

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
void gpsCallback_leader(const sensor_msgs::NavSatFix& msg)
{
    double dx = std::cos(msg.latitude*M_PI/180) * (msg.longitude - lonO) * 111319.9;
    double dy = (msg.latitude - latO) * 111319.9;
    leader_state.x = dx;
    leader_state.y = dy;
    // myobstacles.vertices.back() = Obstacle(obstacle_state, 10, false);
    // myobstacles.vertices.pop_back();
    // myobstacles.vertices.push_back(Obstacle(obstacle_state, 10, false));   
}

// 回调函数，订阅obstacle速度
void velCallback_leader(const geometry_msgs::TwistStamped& msg)
{
    leader_state.u = msg.twist.linear.x;
    leader_state.v = msg.twist.linear.y;
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

    // 创建私有 NodeHandle
    ros::NodeHandle private_nh("~");

    // 发布期望偏航角和实际偏航角，以欧拉角表示
    ros::Publisher yaw_pub = n.advertise<geometry_msgs::PointStamped>("hunter/yawPub", 10);
    ros::Publisher formation_msg_pub = n.advertise<geometry_msgs::TwistStamped>("hunter/formation_msg_pub", 10);

    // 发送速度和偏航
    ros::Publisher setpoint_pub = n.advertise<mavros_msgs::PositionTarget>("hunter/mavros/setpoint_raw/local", 10);

    // 订阅hunter经纬度, 创建一个Subscriber，订阅名为mavros/global_position/global的topic，注册回调函数poseCallback 
    ros::Subscriber gps_sub = n.subscribe("hunter/mavros/global_position/global", 10, gpsCallback_hunter);
    // 订阅hunter速度消息
    ros::Subscriber vel_sub = n.subscribe("hunter/mavros/local_position/velocity_local", 10, velCallback_hunter);
    // 订阅hunter四元数, 创建一个Subscriber，订阅名为mavros/imu/data的topic，注册回调函数poseCallback 
    ros::Subscriber angles_sub = n.subscribe("hunter/mavros/imu/data", 10, anglesCallback);

    // 订阅target经纬度
    ros::Subscriber gps_sub_target = n.subscribe("target/mavros/global_position/global", 10, gpsCallback_target);
    // 订阅target速度消息
    ros::Subscriber vel_sub_target = n.subscribe("target/mavros/local_position/velocity_local", 10, velCallback_target);

    // 订阅领航船leader经纬度
    ros::Subscriber gps_sub_obstacle = n.subscribe("leader/mavros/global_position/global", 10, gpsCallback_leader);
    // 订阅领航船leader速度
    ros::Subscriber vel_sub_obstacle = n.subscribe("leader/mavros/local_position/velocity_local", 10, velCallback_leader);

    // 设置循环的频率
    ros::Rate loop_rate(10);
    
    for(int i=0;i<5;i++)
    { 
        loop_rate.sleep();
    }
    double safe_distance = 20;  // 安全距离m
    double v_d = 2; // 期望速度m/s
    Formation_param formation_param = {0., 0., 0., 0., 0.};
    Guide_law lvf_guide;
    EulerAngles guide_angle;
    double distance = 0.0;
    double factor = 1.0;
    bool flag = true;
    bool is_follower = false; // 是否是跟随船
    double set_phase_diff = 0.0; // 设定的环绕编队相位差(rad)
    while (ros::ok())
    {
        n.param<double>("safe_distance", safe_distance, 20);
        n.param<double>("desired_velocity", v_d, 2);
        private_nh.param<bool>("is_follower", is_follower, false);
        private_nh.param<double>("set_phase_diff", set_phase_diff, 0.0);      
        
        if (is_follower && (distance < (safe_distance * 2.0))) {
            formation_param = follower_vel(leader_state, target_state, hunter_state, set_phase_diff, factor * safe_distance, v_d);
            v_d = formation_param.vel;
            // ROS_INFO("%s, enter velfunc, v_d is %.3f ", ros::this_node::getName().c_str(), v_d);
        }
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

        // 漂移补偿
        double drift = driftCompensation(hunter_state.head, hunter_state.course);
        // double drift = 0;
        lvf_guide.psi = lvf_guide.psi + drift;
    
        // ROS_INFO("yaw_d:%.3f, yaw_t:%.3f , r:%.3f, yaw_err:%.3f, distance_flag:%d", guide_angle.yaw, hunter_state.course, distance, lvf_guide.yaw_err, flag);
        
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
        setpoint.velocity.x = lvf_guide.vel_linear_x; // Forward velocity
        setpoint.velocity.y = 0.0; // No lateral movement
        setpoint.velocity.z = 0.0; // Maintain altitude

        // Desired yaw angle (in radians)
        setpoint.yaw = lvf_guide.psi; // 90 degrees yaw

        setpoint_pub.publish(setpoint);

        // 发布期望航向角和实际航向角以分析误差
        geometry_msgs::PointStamped yaws;
        yaws.point.x = lvf_guide.psi;
        yaws.point.y = distance;
        yaws.point.z = hunter_state.course - lvf_guide.psi;
        yaws.header.stamp = ros::Time::now();
        yaw_pub.publish(yaws);

        //  发布编队信息
        geometry_msgs::TwistStamped formation_msg;
        formation_msg.twist.linear.x = formation_param.vel;
        formation_msg.twist.linear.y = formation_param.theta_leader;
        formation_msg.twist.linear.z = formation_param.theta_follower;
        formation_msg.twist.angular.x = formation_param.phase_diff;
        formation_msg.twist.angular.y = formation_param.set_phase;
        formation_msg.header.stamp = ros::Time::now();
        formation_msg_pub.publish(formation_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}