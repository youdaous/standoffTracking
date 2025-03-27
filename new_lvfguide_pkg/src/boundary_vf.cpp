#include "../include/new_lvfguide_pkg/boundary_vf.h"
#include "../include/new_lvfguide_pkg/lvfGuide.h"
#include "../include/new_lvfguide_pkg/newpath.h"
#include <cmath>
#include <tf2/utils.h>
#include <nav_msgs/Path.h>

// 越界检测函数(判断点是否在多边形内)
bool Is_in_boundary(const State& hunter, const Guide_law& lvfguide, const Boundary& boundary)
{
    int vertexCount = boundary.vertices.size();
    bool inside = false;
    double pre_hunter_x = hunter.x + lvfguide.u;
    double pre_hunter_y = hunter.y + lvfguide.v;
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
Guide_law boundary_guide(const State& hunter, nav_msgs::Path path, double vel_d)
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
    u = factor * vel_d * 2 * sqrt(exp(k * y_e)) / (1 + exp(k * y_e));
    v = vel_d * (1 - exp(k * y_e)) / (1 + exp(k * y_e));
    guide_law.psi = gamma_p +  atan2(v, u);
    guide_law.r = y_e;
    guide_law.u = u;
    guide_law.v = v;
    guide_law.vel_linear_x = vel_d;
    return guide_law;
}

// 边界直线编队速度函数
LineFormationPara follower_boundary_vel(const State& leader, const State& follower, double set_distance)
{
    // relativate distance
    double rela_dis = sqrt(pow(leader.x - follower.x, 2) + pow(leader.y - follower.y, 2));
    // 相对位置向量
    Point rela_pose = Point((leader.x - follower.x), (leader.y - follower.y));

    double cos_leader_to_rela_pose = (leader.u * rela_pose.x + leader.v * rela_pose.y) / \
    (sqrt(pow(leader.u, 2) + pow(leader.v, 2)) * sqrt(pow(rela_pose.x, 2) + pow(rela_pose.y, 2)));

    double cos_follower_to_rela_pose = (follower.u * rela_pose.x + follower.v * rela_pose.y) / \
    (sqrt(pow(follower.u, 2) + pow(follower.v, 2)) * sqrt(pow(rela_pose.x, 2) + pow(rela_pose.y, 2)));

    double rela_dis_dot = 2 / M_PI * atan(set_distance - rela_dis);

    double vel_follower_modify = (sqrt(pow(leader.u, 2) + pow(leader.v, 2)) * cos_leader_to_rela_pose + rela_dis_dot) / cos_follower_to_rela_pose;
    
    LineFormationPara output;
    output.modify_vel = vel_follower_modify;
    output.delta_dis = rela_dis;
    output.set_dis = set_distance;
}