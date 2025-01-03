#ifndef LVFGUIDE_H
#define LVFGUIDE_H
// 船的状态类
struct State {
    double x, y;
    double u, v;
    double course;
    double head; 
};

// 制导命令结构体
struct Guide_law {
    double vel_linear_x;
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

// 声明函数：四元数-->欧拉角
EulerAngles ToEulerAngles(Quaternion& q);
// 声明函数：欧拉角-->四元数
Quaternion ToQuaternion(EulerAngles& angles);

// 转弯率函数
double omega_function(double r, double d, double v_d);

// 计算相位差(逆时针)
double phase_diff(double theta_i, double theta_j);

// 跟随速度函数
double follower_vel(const State& hunter, const State& target, const State& follower, double d, double v_d);

// Lyapunov vector field guidance law
Guide_law Lvf(const State& hunter, const State& target, double d, double v_d);

// 前视补偿函数
Guide_law lookahead_guide(const Guide_law& lvfguide, const State& hunter, const State& target, double d, double v_d);

// 偏移补偿函数
double driftCompensation(const double& head_sensor, const double& course_angle);

#endif