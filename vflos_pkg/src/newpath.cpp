//生成杜宾路径，一个？
#include <nav_msgs/Path.h>
#include "vflos_pkg/newpath.h"
#include <tf/tf.h>
#include <cmath>

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
namespace guaijiao
{
SimpleDubinsPath::SimpleDubinsPath()
{
	  	
}
SimpleDubinsPath::~SimpleDubinsPath() {}


// geometry_msgs::PoseStamped
void SimpleDubinsPath::generateStraightPath(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal, nav_msgs::Path& dubinsPath)
{
  dubinsPath.header.stamp = ros::Time::now();
  dubinsPath.header.frame_id = "map";
 // dubinsPath.poses.clear();

  double dx = goal.pose.position.x - start.pose.position.x;
  double dy = goal.pose.position.y - start.pose.position.y;
  double dx_norm = dx / std::sqrt(dx * dx + dy * dy);
  double dy_norm = dy / std::sqrt(dx * dx + dy * dy);
  tf2::Quaternion q;
  q.setRPY(0, 0, std::atan2(dy, dx));

  // generate straight line segment 生成直线段，dubinsPath.poses最后没有放入goalPose
  for (double i = 0;
       std::fabs(i * m_pathResolution * dx_norm - dx) > 2 * m_pathResolution ||
       std::fabs(i * m_pathResolution * dy_norm - dy) > 2 * m_pathResolution;
       ++i)
  {
    geometry_msgs::PoseStamped point;
    point.header.stamp = ros::Time::now();
    point.header.frame_id = "map";
    point.pose.position.x = start.pose.position.x + i * m_pathResolution * dx_norm; // 变量
    point.pose.position.y = start.pose.position.y + i * m_pathResolution * dy_norm; // 变量
    point.pose.orientation.x = q.x();
    point.pose.orientation.y = q.y();
    point.pose.orientation.z = q.z();
    point.pose.orientation.w = q.w();
    dubinsPath.poses.push_back(point);
  }
}
// geometry_msgs::PoseStamped start = di.start, goal = di.end
void SimpleDubinsPath::makePath(const geometry_msgs::PoseStamped& start,
                                const geometry_msgs::PoseStamped& goal,
                                nav_msgs::Path& dubinsPath)
{

  DubinsPath path;
  double q0[3], q1[3];
  q0[0] = start.pose.position.x;
  q0[1] = start.pose.position.y;
  q0[2] = tf::getYaw(start.pose.orientation);
  q1[0] = goal.pose.position.x;
  q1[1] = goal.pose.position.y;
  q1[2] = tf::getYaw(goal.pose.orientation);
  
  int i, errcode;
  DubinsIntermediateResults in;//定义了一堆参数
  double params[3];
  double cost;
  double best_cost = INFINITY;
  int best_word = -1;
  errcode = dubins_intermediate_results(&in, q0, q1, m_turningRadius);//////////////////真的坏////////////
    if(errcode != EDUBOK) {
        generateStraightPath(start, goal, dubinsPath);
        //return true;
    }


    path.qi[0] = q0[0];
    path.qi[1] = q0[1];
    path.qi[2] = q0[2];
    path.rho = m_turningRadius;
  for( i = 0; i < 6; i++ ) {
      DubinsPathType pathType = (DubinsPathType)i;
      int result;
      switch(pathType)
      {
      case LSL:
          result = dubins_LSL(&in, params);
          break;
      case RSL:
          result = dubins_RSL(&in, params);
          break;
      case LSR:
          result = dubins_LSR(&in, params);
          break;
      case RSR:
          result = dubins_RSR(&in, params);
          break;
     case LRL:
          result = dubins_LRL(&in, params);
          break;
      case RLR:
          result = dubins_RLR(&in, params);
          break;
      default:
          result = EDUBNOPATH;
      }
      errcode = result;
      if(errcode == EDUBOK) {
          cost = params[0] + params[1] + params[2];
          if(cost < best_cost) {
              best_word = i;
              best_cost = cost;
              path.param[0] = params[0];
              path.param[1] = params[1];
              path.param[2] = params[2];
              path.type = pathType;
          }
      }
  }
  if(best_word == -1) {
    generateStraightPath(start, goal, dubinsPath);
    //return true;
  }
  
  // Generate path
  //double angleIncrement = m_pathResolution / m_turningRadius;
  dubins_path_sample_many(&path,  0.02, dubinsPath);
  dubinsPath.poses.push_back(goal);
  //generatePath(x_q, y_q, x_n, y_n, x_cr, y_cr, x_lc, y_lc, dir, goal, dubinsPath);

  //return true;
}

//生成dubins路径的points
int SimpleDubinsPath::dubins_path_sample_many(DubinsPath* path, double stepSize, nav_msgs::Path& dubinsPath)
{
    int retcode;
    double q[3];
    double x = 0.0;
    double length = dubins_path_length(path);
   //ROS_INFO_STREAM("length: (" << length);
    
    dubinsPath.header.stamp = ros::Time::now();
    dubinsPath.header.frame_id = "map";// map坐标系
    //dubinsPath.poses.clear(); // 删除dubinsPath.poses中所有元素

    while( x <  length ) {
        
        dubins_path_sample( path, x, q );
        
        geometry_msgs::PoseStamped point;
        point.header.stamp = ros::Time::now();
        point.header.frame_id = "map";
        point.pose.position.x = q[0];
        point.pose.position.y = q[1];
        tf2::Quaternion Q; // setRPY: 欧拉角 → 四元数，getRPY: 四元数 → 欧拉角
        Q.setRPY(0, 0, q[2]); // setRPY(roll,pitch,yaw);
        point.pose.orientation.x = Q.x();
        point.pose.orientation.y = Q.y();
        point.pose.orientation.z = Q.z();
        point.pose.orientation.w = Q.w();
        dubinsPath.poses.push_back(point);
        //retcode = cb(q, x, user_data);
        //if( retcode != 0 ) {
        //    return retcode;
        //}
        x += stepSize;
        
    }
    
    return 0;
}
//生成dubins路径的point?
int SimpleDubinsPath::dubins_path_sample( DubinsPath* path, double t, double q[3] )
{
    /* tprime is the normalised variant of the parameter t */
    double tprime = t / path->rho;
    double qi[3]; /* The translated initial configuration */
    double q1[3]; /* end-of segment 1 */
    double q2[3]; /* end-of segment 2 */
    const SegmentType* types = DIRDATA[path->type];
    double p1, p2;

    if( t < 0 || t > dubins_path_length(path) ) {
        return EDUBPARAM;
    }

    /* initial configuration */
    qi[0] = 0.0;
    qi[1] = 0.0;
    qi[2] = path->qi[2];//路径起始角

    /* generate the target configuration */
    p1 = path->param[0];
    p2 = path->param[1];
    dubins_segment( p1,      qi,    q1, types[0] );//q1[2]为第一弧段的终止角
    dubins_segment( p2,      q1,    q2, types[1] );//q2[2]为第二段的终止角
    if( tprime < p1 ) {
        dubins_segment( tprime, qi, q, types[0] );//q[2]为当前计算点的角度
    }
    else if( tprime < (p1+p2) ) {
        dubins_segment( tprime-p1, q1, q,  types[1] );
    }
    else {
        dubins_segment( tprime-p1-p2, q2, q,  types[2] );
    }

    /* scale the target configuration, translate back to the original starting point */
    q[0] = q[0] * path->rho + path->qi[0];
    q[1] = q[1] * path->rho + path->qi[1];
    //q[2] = mod2pi(q[2]);
    while (q[2] > M_PI)
    {
      q[2] -= 2*M_PI;
    }
    while (q[2] <= M_PI)
    {
      q[2] += 2*M_PI;
    }

    return EDUBOK;
}
//?
void SimpleDubinsPath::dubins_segment( double t, double qi[3], double qt[3], SegmentType type)
{
    double st = sin(qi[2]);
    double ct = cos(qi[2]);
    if( type == L_SEG ) {
        qt[0] = +sin(qi[2]+t) - st;
        qt[1] = -cos(qi[2]+t) + ct;
        qt[2] = t;
    }
    else if( type == R_SEG ) {
        qt[0] = -sin(qi[2]-t) + st;
        qt[1] = +cos(qi[2]-t) - ct;
        qt[2] = -t;
    }
    else if( type == S_SEG ) {
        qt[0] = ct * t;
        qt[1] = st * t;
        qt[2] = 0.0;
    }
    qt[0] += qi[0];
    qt[1] += qi[1];
    qt[2] += qi[2];
}

double SimpleDubinsPath::dubins_path_length( DubinsPath* path )
{
    double length = 0.;
    length += path->param[0];
    length += path->param[1];
    length += path->param[2];
    length = length * path->rho;
    return length;
}

//赋值in
int SimpleDubinsPath::dubins_intermediate_results(DubinsIntermediateResults* in, double q0[3], double q1[3], double rho)
{
    double dx, dy, D, d, theta, alpha, beta;
    if( rho <= 0.0 ) {
        return EDUBBADRHO;
    }

    dx = q1[0] - q0[0];
    dy = q1[1] - q0[1];
    D = std::sqrt( dx * dx + dy * dy );
    d = D / rho;
    theta = 0;

    /* test required to prevent domain errors if dx=0 and dy=0 */
    if(d > 0) {
        theta = mod2pi(std::atan2( dy, dx ));
    }
    alpha = mod2pi(q0[2] - theta);
    beta  = mod2pi(q1[2] - theta);

    in->alpha = alpha;
    in->beta  = beta;
    in->d     = d;
    in->sa    = sin(alpha);
    in->sb    = sin(beta);
    in->ca    = cos(alpha);
    in->cb    = cos(beta);
    in->c_ab  = cos(alpha - beta);
    in->d_sq  = d * d;

    return EDUBOK;
}
//确保theta在[0, 2*pi]
double SimpleDubinsPath::mod2pi( double theta )//floor()向下取整
{
    return theta - (2*M_PI) * floor(theta/(2*M_PI));
}
//dubins路径的六种情况
int SimpleDubinsPath::dubins_LSL(DubinsIntermediateResults* in, double out[3]) 
{
    double tmp0, tmp1, p_sq;
    
    tmp0 = in->d + in->sa - in->sb;
    p_sq = 2 + in->d_sq - (2*in->c_ab) + (2 * in->d * (in->sa - in->sb));

    if(p_sq >= 0) {
        tmp1 = std::atan2( (in->cb - in->ca), tmp0 );
        out[0] = mod2pi(tmp1 - in->alpha);
        out[1] = std::sqrt(p_sq);
        out[2] = mod2pi(in->beta - tmp1);
        return EDUBOK;
    }
    return EDUBNOPATH;
}
int SimpleDubinsPath::dubins_RSR(DubinsIntermediateResults* in, double out[3]) 
{
    double tmp0 = in->d - in->sa + in->sb;
    double p_sq = 2 + in->d_sq - (2 * in->c_ab) + (2 * in->d * (in->sb - in->sa));
    if( p_sq >= 0 ) {
        double tmp1 = std::atan2( (in->ca - in->cb), tmp0 );
        out[0] = mod2pi(in->alpha - tmp1);
        out[1] = std::sqrt(p_sq);
        out[2] = mod2pi(tmp1 -in->beta);
        return EDUBOK;
    }
    return EDUBNOPATH;
}
int SimpleDubinsPath::dubins_LSR(DubinsIntermediateResults* in, double out[3]) 
{
    double p_sq = -2 + (in->d_sq) + (2 * in->c_ab) + (2 * in->d * (in->sa + in->sb));
    if( p_sq >= 0 ) {
        double p    = std::sqrt(p_sq);
        double tmp0 = std::atan2( (-in->ca - in->cb), (in->d + in->sa + in->sb) ) - std::atan2(-2.0, p);
        out[0] = mod2pi(tmp0 - in->alpha);
        out[1] = p;
        out[2] = mod2pi(tmp0 - mod2pi(in->beta));
        return EDUBOK;
    }
    return EDUBNOPATH;
}
int SimpleDubinsPath::dubins_RSL(DubinsIntermediateResults* in, double out[3]) 
{
    double p_sq = -2 + in->d_sq + (2 * in->c_ab) - (2 * in->d * (in->sa + in->sb));
    if( p_sq >= 0 ) {
        double p    = std::sqrt(p_sq);
        double tmp0 = std::atan2( (in->ca + in->cb), (in->d - in->sa - in->sb) ) - std::atan2(2.0, p);
        out[0] = mod2pi(in->alpha - tmp0);
        out[1] = p;
        out[2] = mod2pi(in->beta - tmp0);
        return EDUBOK;
    }
    return EDUBNOPATH;
}
int SimpleDubinsPath::dubins_RLR(DubinsIntermediateResults* in, double out[3]) 
{
    double tmp0 = (6. - in->d_sq + 2*in->c_ab + 2*in->d*(in->sa - in->sb)) / 8.;
    double phi  = std::atan2( in->ca - in->cb, in->d - in->sa + in->sb );
    if( fabs(tmp0) <= 1) {
        double p = mod2pi((2*M_PI) - acos(tmp0) );
        double t = mod2pi(in->alpha - phi + mod2pi(p/2.));
        out[0] = t;
        out[1] = p;
        out[2] = mod2pi(in->alpha - in->beta - t + mod2pi(p));
        return EDUBOK;
    }
    return EDUBNOPATH;
}
int SimpleDubinsPath::dubins_LRL(DubinsIntermediateResults* in, double out[3])
{
    double tmp0 = (6. - in->d_sq + 2*in->c_ab + 2*in->d*(in->sb - in->sa)) / 8.;
    double phi = std::atan2( in->ca - in->cb, in->d + in->sa - in->sb );
    if( fabs(tmp0) <= 1) {
        double p = mod2pi( 2*M_PI - acos( tmp0) );
        double t = mod2pi(-in->alpha - phi + p/2.);
        out[0] = t;
        out[1] = p;
        out[2] = mod2pi(mod2pi(in->beta) - in->alpha -t + mod2pi(p));
        return EDUBOK;
    }
    return EDUBNOPATH;
}

// coverage_binn.cpp
bool SimpleDubinsPath::getTargetHeading(double x_q, double y_q, double theta_q,
                                        double x_n, double y_n,
                                        double& yawTarget)
{
  Dir dir = turningDirection(x_q, y_q, theta_q, x_n, y_n);

  // Find the center of the turning circle
  double x_cr, y_cr;
  turningCenter(x_q, y_q, theta_q, x_cr, y_cr, dir);

  // Is target reachable?
  if (std::sqrt(std::pow(x_n - x_cr, 2) + std::pow(y_n - y_cr, 2)) <
      m_turningRadius)
  {
    return false; // 未更新CoverageBinn::findNextCell的yawNext，为0，不过此时执行的函数为generateStraightPath，用不到目标航向角
  }

  // Find angle of tangent line from target to turning circle
  double beta1, beta2;
  tangentLine(x_n, y_n, x_cr, y_cr, beta1, beta2);

  // Find tangent point
  double x_lc, y_lc;
  tangentPoint(x_q, y_q, x_n, y_n, x_cr, y_cr, beta1, beta2, dir, x_lc, y_lc);

  yawTarget = std::atan2(y_n - y_lc, x_n - x_lc); // 公式7.5，yawTarget = θn
  return true;
}

// 确定转弯方向
SimpleDubinsPath::Dir SimpleDubinsPath::turningDirection(double x_q, double y_q,
                                                         double theta_q,
                                                         double x_n, double y_n)
// Note: Prefers right turns when doing 180 turn (or going straight)注意: 在做 180 度转弯（或直行）时更喜欢右转
{
  Dir turningDirection = Right;

  // Rotate x-axis theta_q, and check if the new y-coordinate is on the left (positive) 旋转 x 轴 θq（初始航向），并检查新的 y 坐标是否在左侧（正）
  // 考虑由初始位置和航向定义的直线，从下个位置到这条直线的距离由以下公式得出:（如果大于0，左转，否则，默认右转（z轴正向向上），公式7.1）
  if (-(x_n - x_q) * sin(theta_q) + (y_n - y_q) * cos(theta_q) > 0)
  {
    turningDirection = Left;
  }
  //turningDirection = Left;//圆形期望路径
  return turningDirection;
}
// 计算转弯圆中心坐标
void SimpleDubinsPath::turningCenter(double x_q, double y_q, double theta_q,
                                     double& x_cr, double& y_cr, Dir dir)
// Note: For compliance with previous assumption, must prefer right turns when doing 180 turn (or going straight) 注意: 为了符合之前的假设，在做 180 转（或直行）时必须优先右转
{ // cr1、cr2为不同转弯方向的圆心坐标，公式7.3
  double x_cr1 = x_q + sin(theta_q) * m_turningRadius;
  double y_cr1 = y_q - cos(theta_q) * m_turningRadius;
  double x_cr2 = x_q - sin(theta_q) * m_turningRadius;
  double y_cr2 = y_q + cos(theta_q) * m_turningRadius;

  // Rotate x-axis theta_q, and check if the new y-coordinate of cr1 is on the left (positive) 旋转x轴 θq，检查cr1新的y坐标是否在左边（正）
  // -(x_cr1 - x_q) * sin(theta_q) + (y_cr1 - y_q) * cos(theta_q) = -m_turningRadius * (sin(theta_q)^2 + cos(theta_q)^2) < 0
  if (-(x_cr1 - x_q) * sin(theta_q) + (y_cr1 - y_q) * cos(theta_q) > 0)
  // cr1 is left turning point
  {
    if (dir == Left)
    {
      x_cr = x_cr1;
      y_cr = y_cr1;
    }
    else // dir == Right
    {
      x_cr = x_cr2;
      y_cr = y_cr2;
    }
  }
  else
  // cr1 is right turning point
  {
    if (dir == Right)
    {
      x_cr = x_cr1; 
      y_cr = y_cr1;
    }
    else // dir == Left
    {
      x_cr = x_cr2;
      y_cr = y_cr2;
    }
  }
 // ROS_INFO_STREAM("cr: (" << x_cr << ", " << y_cr << ")");
}
// 切线，β1、β2为两个切线角度，对应两个切点(x_lc1, y_lc1)、(x_lc2, y_lc2)
void SimpleDubinsPath::tangentLine(double x_n, double y_n, double x_cr,
                                   double y_cr, double& beta1, double& beta2)
{
  double a = (x_cr - x_n);
  double b = (y_n - y_cr);
  beta1 = 0;
  beta2 = 0;
  // 公式7.4
  if (std::abs(b + m_turningRadius) < epsilon)
  {
    beta1 = 2 * atan((a - std::sqrt((a * a + b * b -
                                     m_turningRadius * m_turningRadius))) /
                     (b - m_turningRadius));
    beta2 = 2 * atan((a + std::sqrt((a * a + b * b -
                                     m_turningRadius * m_turningRadius))) /
                     (b - m_turningRadius));
  }
  else
  {
    beta1 = 2 * atan((a + std::sqrt((a * a + b * b -
                                     m_turningRadius * m_turningRadius))) /
                     (b + m_turningRadius));
    beta2 = 2 * atan((a - std::sqrt((a * a + b * b -
                                     m_turningRadius * m_turningRadius))) /
                     (b + m_turningRadius));
  }
  // force beta β in [0, pi)
  if (beta1 < 0)
  {
    beta1 = beta1 + M_PI;
  }
  if (beta2 < 0)
  {
    beta2 = beta2 + M_PI;
  }
}
// 切点
void SimpleDubinsPath::tangentPoint(double x_q, double y_q, double x_n,
                                    double y_n, double x_cr, double y_cr,
                                    double beta1, double beta2, Dir dir,
                                    double& x_lc, double& y_lc)
{
  // Circle-line intersection with circle in origin 与原点圆的圆线相交
  // 公式: http://mathworld.wolfram.com/Circle-LineIntersection.html
  double x2 = x_n - x_cr; // move line to origin将线移至原点
  double y2 = y_n - y_cr; // move line to origin
  // 因为半径为1？
  // 切点1(x_lc1, y_lc1)
  double x1 = (x_n + cos(beta1)) - x_cr; // move line to origin
  double y1 = (y_n + sin(beta1)) - y_cr; // move line to origin
  double dx = x2 - x1;
  double dy = y2 - y1;
  double dr = std::sqrt(dx * dx + dy * dy);
  double D = x1 * y2 - x2 * y1; // 叉乘

  double x_lc1 = D * dy / (dr * dr);
  double y_lc1 = -D * dx / (dr * dr);
  x_lc1 = x_lc1 + x_cr; // move back from origin从原点移回
  y_lc1 = y_lc1 + y_cr; // move back from origin

  // 切点2(x_lc2, y_lc2)
  x1 = (x_n + cos(beta2)) - x_cr; // move line to origin
  y1 = (y_n + sin(beta2)) - y_cr; // move line to origin
  dx = x2 - x1;
  dy = y2 - y1;
  dr = std::sqrt(dx * dx + dy * dy);
  D = x1 * y2 - x2 * y1;

  double x_lc2 = D * dy / (dr * dr);
  double y_lc2 = -D * dx / (dr * dr);
  x_lc2 = x_lc2 + x_cr; // move back from origin
  y_lc2 = y_lc2 + y_cr; // move back from origin

  // Find the first tangent point encountered along the direction of rotation 找到沿旋转方向遇到的第一个切点，用作弧段的端点pl
  double v_head[2] = {x_q - x_cr, y_q - y_cr};
  double v_lc1[2] = {x_lc1 - x_cr, y_lc1 - y_cr};
  double v_lc2[2] = {x_lc2 - x_cr, y_lc2 - y_cr};

  x1 = v_head[0];
  y1 = v_head[1];
  x2 = v_lc1[0];
  y2 = v_lc1[1];
  double dot = x1 * x2 + y1 * y2; // dot product 点乘
  double det = x1 * y2 - y1 * x2; // determinant 叉乘
  double angle1 = std::atan2(det, dot);
  if (angle1 < 0)
    angle1 += 2 * M_PI; // wrap to [0, 2*pi]

  x2 = v_lc2[0];
  y2 = v_lc2[1];
  dot = x1 * x2 + y1 * y2; // dot product 
  det = x1 * y2 - y1 * x2; // determinant 
  double angle2 = std::atan2(det, dot);
  if (angle2 < 0)
    angle2 += 2 * M_PI; // wrap to [0, 2*pi]

  x_lc = x_lc2;
  y_lc = y_lc2;
  double angle = angle2;
  if (dir == Left) // 左转谁小取谁
  {
    if (angle1 < angle2)
    {
      x_lc = x_lc1;
      y_lc = y_lc1;
      angle = angle1;
    }
  }
  else if (dir == Right) // 右转谁大取谁
  {
    if (angle1 > angle2 || abs(angle1) < epsilon) // angle == 0 is best
    {
      x_lc = x_lc1;
      y_lc = y_lc1;
      angle = angle1;
    }
  }
}

// geometry_msgs::PoseStamped
void SimpleDubinsPath::generatePath(double x_q, double y_q, double x_n,
                                    double y_n, double x_cr, double y_cr,
                                    double x_lc, double y_lc, Dir dir,
                                    const geometry_msgs::PoseStamped& goal,
                                    nav_msgs::Path& dubinsPath)
{
  dubinsPath.header.stamp = ros::Time::now();
  dubinsPath.header.frame_id = "map";// map坐标系
 // dubinsPath.poses.clear(); // 删除dubinsPath.poses中所有元素

  // find circle segment to follow 找到要遵循的圆段
  double startAngle = std::atan2(y_q - y_cr, x_q - x_cr); // θq - π/2
  if (startAngle < 0)
    startAngle += 2 * M_PI; // wrap to [0, 2*pi]
  double stopAngle = std::atan2(y_lc - y_cr, x_lc - x_cr);
  if (stopAngle < 0)
    stopAngle += 2 * M_PI; // wrap to [0, 2*pi]

  if (dir == Left && stopAngle < startAngle)
  {
    stopAngle += 2 * M_PI;
  }
  else if (dir == Right && stopAngle > startAngle)
  {
    stopAngle -= 2 * M_PI;
  }

  // generate circle segment 生成圆段，dubinsPath.poses开始放入startPose
  double angleIncrement = m_pathResolution / m_turningRadius; // 角度增量=路径分辨率/转弯半径
  // 一次增加一个角度增量，当角度差小于等于2个角度增量的时候，停止增加point。Right好像为-1，Left为1
  for (double i = startAngle; std::abs(i - stopAngle) > 2 * angleIncrement;
       i += dir * angleIncrement)
  {
    geometry_msgs::PoseStamped point;
    point.header.stamp = ros::Time::now();
    point.header.frame_id = "map";
    point.pose.position.x = x_cr + cos(i) * m_turningRadius;
    point.pose.position.y = y_cr + sin(i) * m_turningRadius;
    tf2::Quaternion q; // setRPY: 欧拉角 → 四元数，getRPY: 四元数 → 欧拉角
    q.setRPY(0, 0, i + ((dir == Right) ? -M_PI_2 : M_PI_2)); // setRPY(roll,pitch,yaw);
    point.pose.orientation.x = q.x();
    point.pose.orientation.y = q.y();
    point.pose.orientation.z = q.z();
    point.pose.orientation.w = q.w();
    dubinsPath.poses.push_back(point);
  }

  double dx = x_n - x_lc;
  double dy = y_n - y_lc;
  double dx_norm = dx / std::sqrt(dx * dx + dy * dy); // cosθn
  double dy_norm = dy / std::sqrt(dx * dx + dy * dy); // sinθn
  tf2::Quaternion q;
  q.setRPY(0, 0, std::atan2(y_n - y_lc, x_n - x_lc)); // 公式7.5得yaw=θn，航行中yaw不变，故只计算一次

  // generate straight line segment 生成直线段，dubinsPath.poses最后放入goalPose
  // 一次增加一个dx或dy增量，当距离差小于等于2个增量的时候，停止增加point
  for (double i = 0;
       std::fabs(i * m_pathResolution * dx_norm - dx) > 2 * m_pathResolution ||
       std::fabs(i * m_pathResolution * dy_norm - dy) > 2 * m_pathResolution;
       ++i)
  {
    geometry_msgs::PoseStamped point;
    point.header.stamp = ros::Time::now();
    point.header.frame_id = "map";
    point.pose.position.x = x_lc + i * m_pathResolution * dx_norm; // 变量
    point.pose.position.y = y_lc + i * m_pathResolution * dy_norm; // 变量
    point.pose.orientation.x = q.x();
    point.pose.orientation.y = q.y();
    point.pose.orientation.z = q.z();
    point.pose.orientation.w = q.w();
    dubinsPath.poses.push_back(point);
  }
  
  dubinsPath.poses.push_back(goal);
}

} // namespace otter_coverage
