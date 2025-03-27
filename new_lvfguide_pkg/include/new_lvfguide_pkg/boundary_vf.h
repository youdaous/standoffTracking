#ifndef BOUNDARY_VF
#define BOUNDARY_VF

#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include "./lvfGuide.h"
#include "./newpath.h"

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

// 直线编队结构参数
struct LineFormationPara
{
    double modify_vel;
    double delta_dis;
    double set_dis;
};


// 越界检测函数(判断点是否在多边形内)
bool Is_in_boundary(const State& hunter, const Guide_law& lvfguide, const Boundary& boundary);

// dubinpath起点和终点生成函数
dubin_start_end gen_dubin_StartEnd(double xx1,double yy1,double an1,double xx2,double yy2,double an2);

// 边界dubin路径生成函数
nav_msgs::Path DubinPath_from_boundary(const Boundary& boundary);

// 边界矢量场指导
Guide_law boundary_guide(const State& hunter, nav_msgs::Path path, double vel_d);

// 边界直线编队速度函数
LineFormationPara follower_boundary_vel(const State& leader, const State& follower, double set_distance);

#endif