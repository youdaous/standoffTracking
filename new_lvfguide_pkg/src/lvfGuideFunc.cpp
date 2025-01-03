#include <ros/ros.h>
#include <cmath>
#include <iostream>
#include <vector>
// #include "lvfguide_pkg/newpath.h"
#include "../include/new_lvfguide_pkg/lvfGuide.h"

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

// 转弯率函数
double omega_function(double r, double d, double v_d) {
    // Define intermediate expressions to simplify the equation
    double x = r / d;
    double cos_pi_x = std::cos(M_PI * x);
    double sin_pi_x = std::sin(M_PI * x);
    double sin_2pi_x = std::sin(2 * M_PI * x);

    double term1 = std::pow(1 - cos_pi_x, 2);
    double term2 = (1.0 / 16.0) * std::pow(term1, 2) - 2.0;

    double numerator1 = term1 * ((term2 * std::pow(x, 2) + 2.0) / 4.0);
    double numerator2 = 2.0 * M_PI * (std::pow(x, 5) + 2.0 * std::pow(x, 3) + x) * (sin_pi_x - 0.5 * sin_2pi_x) / 4.0;

    double denominator = std::pow(std::pow(x, 4) + term2 * std::pow(x, 2) + 1.0, 1.5);

    double omega = (v_d / d) * (numerator1 + numerator2) / denominator;
    return omega;
}

// 计算相位差(逆时针)
double phase_diff(double theta_i, double theta_j)
{
    """
    计算逆时针的相位差
    :param theta_i: 无人机 i 的相位（弧度，范围 [-pi, pi]）
    :param theta_j: 无人机 j 的相位（弧度，范围 [-pi, pi]）
    :return: 同一方向的相位差 Δθ，范围 [0, 2π)
    """
    double delta_theta = theta_i - theta_j
    double delta_theta_norm = 2 * M_PI - (delta_theta + 2 * M_PI) % (2 * M_PI)  // 归一化到 [0, 2π)
    return delta_theta_norm
}

// 跟随速度函数
double follower_vel(const State& leader, const State& target, const State& follower, double set_phase_diff, double d, double v_d)
{
    double theta_leader = atan2(leader.y - target.y, leader.x - target.x);
    double theta_follower = atan2(follower.y - target.y, follower.x - target.x);
    double delta_theta = phase_diff(theta_follower, theta_leader);
    double k = 0.8 * v_d / (d * 2 * M_PI);
    double vel_follower = k * (delta_theta - set_phase_diff) * d + v_d;
    return vel_follower;
}

// Lyapunov vector field guidance law
Guide_law Lvf(const State& hunter, const State& target, double d, double v_d)
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
    double v_dc = v_d;
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
        guide.vel_linear_x = v_d;
        return guide;
    }
    double alpha = (-(vx_d * target.u + vy_d * target.v) + 
    sqrt(pow(vx_d * target.u + vy_d * target.v, 2) - pow(v_dc, 2) * ((pow(target.u, 2) + pow(target.v, 2)) - pow(v_dc, 2)))) / pow(v_dc, 2);
    vx_d = target.u + alpha * vx_d;
    vy_d = target.v + alpha * vy_d;
    guide.psi = atan2(vy_d, vx_d);
    guide.u = vx_d;
    guide.v = vy_d;
    guide.vel_linear_x = v_d;
    // ROS_INFO("u:%.3f, v:%.3f, yaw:%.3f", guide.u, guide.v, guide.psi);
    return guide;
}

// 直线矢量场函数
// Guide_law line_vf()

// 前视补偿函数
Guide_law lookahead_guide(const Guide_law& lvfguide, const State& hunter, const State& target, double d, double v_d)
{
    State hunter_lookahead = hunter;
    double yaw_t = hunter.course;
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
    Guide_law lvfguide_lookahead = Lvf(hunter_lookahead, target, d, v_d);
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