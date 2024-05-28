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
// #include <guidance/guidance.h>
// #include "guidance/my_path.h"
#include <vflos_pkg/newpath.h>
#include <visualization_msgs/Marker.h>
#include "mavros_msgs/OverrideRCIn.h"

double lonO=121.20980712;//原点的经度
double latO=31.05732962;//原点的纬度

int T = 1; // 时间间隔
double sigma = 0.1; // σ
bool flag_path = false;
bool flag_lastTime = true;
bool flag_first = true;
double delta_x = 0.0;
double delta_y = 0.0;
double pre_x;
double pre_y;
double deltaTime = 0.1;
ros::Time lastTime;
bool flag_d = false;
int turningDir = -1;
double x_cr, y_cr;
double turning_radius = 3.0;
double r_v = 1.0;
double turn_angle = 0.0;
double delta_max = 3.0;
double delta_min = 1.0;
double delta_k = 1.0;
double DELTA = 0.5;
double m_maxSpeed = 1.0;
double m_maxSpeedTurn = 0.55;
double m_minSpeed = 0.5;

double x_d=0.0;
float x_usv=0,y_usv=0;
double x_hou,y_hou;//USV船尾的点
double x_qian,y_qian;//USV船尾的点
double L=16.0;//船的长度
float yaw=0;
float sudu=0;
float vx=0;
float vy=0;
double sj=0;
double xx1=0,yy1=0,xx2=0,yy2=0,a1=0,a2=0;
double v;//给的油门值
int duanshu=4;
int dingyue_Hz = 0;//订阅频率
double usv_zhihou_yaw = 0;
float zuihoudian[1][2]={{0,0}};

float p1[20][2]=
{
  /*
  {10 ,10},{110,10}, 
  {110, 10},{110,110},
  {110,110},{60,60},
  {60,60},{10,60}
  */

  // {10 ,5},{40,5}, 
  // {40, 30},{25,20},
  // {12,30}  

  // zigzag
  // {10, 0}, {30, 0}, {50,30}, {70, 0}, {90, 30}, {110, 0}, {130, 30}, {150, 0}, {170, 30}

  // 边界逃逸
  // {98, 0}, {170, 40}, {110, 130}, {200, 130}
  // {98, 0}, {170, 40}, {110, 130}, {80, 90}
  // {88, -4}, {170, 40}, {80, 90}, {110, 130}
  {88, -4}, {170, 40}, {183.6, 99.8}, {197.2, 159.6}

  // boundary
  // {0, 0},
  // {190, -55},
  // {207, 32},
  // {285, 122},
  // {360, 185},
  // {205, 185},
  // {128, 160},
  // {70, 180},
  // {90, 90},
  // {50, 40},
  // {5, 33} 
};

//四元数转欧拉角
struct Quaternion 
{
  double w, x, y, z;
};
struct EulerAngles 
{
  double roll, pitch, yaw;
};
EulerAngles angles;
Quaternion q;
void ToEulerAngles(Quaternion q) 
{
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
}
//调整船的朝向 转成四元数
struct attitude_quat
{
  float q1;
  float q2;
  float q3;
  float q4;
};
struct attitude_quat att_quat;
double pointDistance(double x0, double y0, double x1, double y1)
{
  return std::sqrt(std::pow(x1 - x0, 2) + std::pow(y1 - y0, 2));
}
void from_euler(float roll, float pitch, float yaw)
{
    const float cr2 = cosf(roll*0.5f);
    const float cp2 = cosf(pitch*0.5f);
    const float cy2 = cosf(yaw*0.5f);
    const float sr2 = sinf(roll*0.5f);
    const float sp2 = sinf(pitch*0.5f);
    const float sy2 = sinf(yaw*0.5f);
    att_quat.q1 = cr2*cp2*cy2 + sr2*sp2*sy2;
    att_quat.q2 = sr2*cp2*cy2 - cr2*sp2*sy2;
    att_quat.q3 = cr2*sp2*cy2 + sr2*cp2*sy2;
    att_quat.q4 = cr2*cp2*sy2 - sr2*sp2*cy2;
}
//订阅rc输出
void rcCallback(const mavros_msgs::OverrideRCIn::ConstPtr& msg)
{
  const auto& channels = msg->channels;
  // 打印channels中的通道值
  ROS_INFO("RC Channels:");
  for (size_t i = 0; i < channels.size(); ++i)
  {
      ROS_INFO("Channel %zu: %d", i + 1, channels[i]);
  }
}

//接收船的sudu信息
void vel_callback(const geometry_msgs::TwistStamped& msg)
{
  // 获取线性速度变量
  double sec,nsec;
	vx = msg.twist.linear.x; // X轴线速度
	vy = msg.twist.linear.y; // Y轴线速度
	sec = msg.header.stamp.sec; // 秒
  sec = sec - 1697000000;
  nsec= msg.header.stamp.nsec;//纳秒
  nsec= nsec / 1e9;
  sj = sec + nsec;
}
//接收船的位置信息
void gpsCallback(const sensor_msgs::NavSatFix& msg)
{
	x_usv = std::cos(msg.latitude*M_PI/180) * (msg.longitude - lonO) * 111319.9 ;
	y_usv = (msg.latitude - latO) * 111319.9 ;
  //ROS_INFO("x:%.3f, y:%.3f", msg.latitude*1000, msg.longitude*10000);
}
void poseCallback(const geometry_msgs::PoseStamped& msg)
{
}
//接收船的角度信息
void anglesCallback(const sensor_msgs::Imu& msg)
{
	q.x=msg.orientation.x;
	q.y=msg.orientation.y;
	q.z=msg.orientation.z;
	q.w=msg.orientation.w;
	ToEulerAngles(q);//计算出的欧拉角全是弧度

}

double wucha = 0.0,qian_wucha;
bool first = true;//是否开始计算误差

nav_msgs::Path path;
nav_msgs::Path path2;

void Time_LOS(double x, double y, double psi)
// TODO: cuts turns, how to fix?
{
  // Identify closest point on path
  std::vector<geometry_msgs::PoseStamped>::iterator closest;
  double minDist = std::numeric_limits<double>::max();
  for (auto it = path.poses.begin(); it != path.poses.end(); it++)
  {
    double dist = std::sqrt(std::pow(x - it->pose.position.x, 2) + std::pow(y - it->pose.position.y, 2));
    if (dist < minDist)
    {
      minDist = dist;
      closest = it;
    }
  }
  //误差计算 自己加的 王后可以去掉
  if(minDist < 1.0)
  {
    first = false;
  }
  if(first == false)
  {
    wucha = minDist;
  }
  // Store closest
  geometry_msgs::PoseStamped pose_d = *closest;

  // Erase previous elements
  path.poses.erase(path.poses.begin(), closest);

  // Path tangential angle
  double gamma_p = tf2::getYaw(pose_d.pose.orientation);

  // Cross-track error
  // 主要改下面这段
  double y_e = -(x - pose_d.pose.position.x) * std::sin(gamma_p) + (y - pose_d.pose.position.y) * std::cos(gamma_p);

  // Time-varying lookahead distance
  double delta_y_e = (delta_max - delta_min) * std::exp(-delta_k * std::pow(y_e, 2)) + delta_min;
  // if turning => small lookahead distance
  // 判断下一个点是否要转弯
  bool isTurning = false;
  if ((closest + 1) != path.poses.end())
  {
    double nextAngle = tf2::getYaw((*(closest + 1)).pose.orientation);
    if (std::fabs(gamma_p - nextAngle) > std::numeric_limits<double>::epsilon())
    {
      delta_y_e = delta_min;
      isTurning = true;
    }
  }

  // velocity-path relative angle
  double chi_r = std::atan(-y_e / delta_y_e);

  // desired course angle
  double chi_d = gamma_p + chi_r;

  // calculate error in heading
  double chi_err = chi_d - psi;
  while (chi_err > M_PI)
  {
    chi_err -= 2 * M_PI;
  }
  while (chi_err < -M_PI)
  {
    chi_err += 2 * M_PI;
  }

  // calculate desired speed
  double u = m_maxSpeed * (1 - std::abs(y_e) / 5 - std::abs(chi_err) / M_PI_2);
  u = std::max(u, m_minSpeed);
  if (isTurning)
  {
    u = m_maxSpeedTurn;
  }
  v = u;
  x_d =chi_d*180/M_PI;
}


void VFILOS(float x, float y,double psi)
{
 if(flag_lastTime == true)
    {
      flag_lastTime = false;
      deltaTime = 0.1;
    }//当前时间赋值
    else
    {
      deltaTime = (ros::Time::now() - lastTime).toSec();//将Time转为double型时间，获取double型的时间间隔
    }
    lastTime = ros::Time::now();
    
    if (path.poses.size() <= 1)//如果算法计算的位姿数量小于等于1
    {
      //pao=0;//;
    }
  
  // Identify closest point on path识别路径上的最近点 (7.6)式
  std::vector<geometry_msgs::PoseStamped>::iterator closest;//声明最近点迭代器
  double minDist = std::numeric_limits<double>::max();//最大界限
  for (auto it = path.poses.begin(); it != path.poses.end(); it++)//遍历m_path.poses，寻找算法得到的点中离当前位置最近的点
  {
    double dist = std::sqrt(std::pow(x - it->pose.position.x, 2) +
                            std::pow(y - it->pose.position.y, 2));//距离计算  pow()用于返回第一个参数的第二个参数次幂的值
    if (dist < minDist)
    {
      minDist = dist;//更新最短距离
      closest = it;//更新最近点
    }
  }

  // Store closest储存最近点 pose_d
  geometry_msgs::PoseStamped pose_d = *closest;
    
  // Erase previous elements删除以前的元素
  path.poses.erase(path.poses.begin(), closest);

  // Path tangential angle路径切向角γp 
  double gamma_p = tf2::getYaw(pose_d.pose.orientation);
  //ROS_INFO("x:%.6f, y:%.3f", pose_d.pose.orientation.z, gamma_p);////////////

  // Cross-track error跨轨误差y(e)   (7.7)式
  double y_e = -(x - pose_d.pose.position.x) * std::sin(gamma_p) + (y - pose_d.pose.position.y) * std::cos(gamma_p);
  double angle_ex = std::abs(gamma_p - psi);
  if(angle_ex > M_PI)
  {
    angle_ex = 2 * M_PI - angle_ex;
  }
  double delta_y_e =
      (angle_ex*std::min(std::exp(std::abs(1/y_e)+0.6), 100.0))*std::abs(delta_max- delta_min) * std::exp(-delta_k * std::pow(y_e, 2)) + delta_min;

  bool isTurning = false;
  if ((closest + 1) != path.poses.end())//如果最近点不是goalPose
  {
    double nextAngle = tf2::getYaw((*(closest + 1)).pose.orientation);
    if (std::fabs(gamma_p - nextAngle) > std::numeric_limits<double>::epsilon())//ε（极小值）是从1到下一个double数字的距离，杜宾路径最近点和下一个点偏航角差值如果大于ε，说明不是直行
    {
      delta_y_e = delta_min;
      isTurning = true;//转弯
    }
  }
//积分项
  static double dint = 0;
  static double dintn;
  static double D;
  if (flag_path == true)
  {
    flag_path = false;
    dint = 0;
    //ROS_INFO_STREAM("New path");//新的路径
  }
  double cur_u, cur_v, U;
  //double Cramer = std::pow(std::cos(psi), 2) - std::pow(std::sin(psi), 2);
  cur_u = delta_x * std::cos(psi) + delta_y * std::sin(psi);//公式1、6、7 cur_u = (delta_x * std::cos(psi) - delta_y * std::sin(psi)) / Cramer;
  cur_v = -delta_y * std::cos(psi) + delta_x * std::sin(psi);//cur_v = (delta_y * std::cos(psi) - delta_x * std::sin(psi)) / Cramer;
  U = std::sqrt(std::pow(cur_u, 2) + std::pow(cur_v, 2));
 // ROS_INFO_STREAM("U: " << U << ", cur_u: " << cur_u << ", cur_v: " << cur_v);
  
  double d_cr = std::sqrt(std::pow(x - x_cr, 2) +
                    std::pow(y - y_cr, 2));
  //ROS_INFO_STREAM("d-r: " << (d_cr - turning_radius) << ", y_e: " << y_e);
  
  if (isTurning == false)
  {D = U * y_e / std::sqrt(std::pow((y_e + sigma*dint), 2) + std::pow(delta_y_e, 2));}
  else
  {D = U * (d_cr - r_v) / std::sqrt(std::pow((d_cr - r_v + sigma*dint), 2) + std::pow(delta_y_e, 2));}
  
  dintn = dint + D * deltaTime;
  dint = dintn;
  
  double chi_r, chi_d;
  if (isTurning == true)
  {
    double gamma_cr =
        std::atan2(y - y_cr, x - x_cr);
    if (turningDir == 1) //turn left
    {
      chi_r = std::atan((d_cr - r_v - sigma*dintn) / delta_y_e);
      chi_d = gamma_cr + M_PI_2 + chi_r;
      //ROS_INFO("zuozuozuo" );
    }
    else
    {
      chi_r = std::atan((-d_cr + r_v - sigma*dintn) / delta_y_e);
      chi_d = gamma_cr - M_PI_2 + chi_r;
      //ROS_INFO("youyouyou" );
    }
  }
  else
  {
    chi_r = std::atan((-y_e - sigma*dintn) / delta_y_e);
    chi_d = gamma_p + chi_r;
     // ROS_INFO("zwuwuwuwu" );
  }

  // calculate error in heading计算航向误差，[-π, π]
  double chi_err = chi_d - psi;
  while (chi_err > M_PI)
  {
    chi_err -= 2 * M_PI;
  }
  while (chi_err < -M_PI)
  {
    chi_err += 2 * M_PI;
  }

  // calculate desired speed计算isTurning = false时的期望速度u
  double u = m_maxSpeed * (1 - std::abs(y_e) / 5 - std::abs(chi_err) / M_PI_2);
  u = std::max(u, m_minSpeed);
  if (isTurning)//如果转弯
    u = m_maxSpeedTurn;//赋值最大转弯速度

  // Publish speed and course to controller将期望速度和航线角发布到controller
 // usv_msgs::SpeedCourse msg;//声明发布的msg
 // msg.speed = u;
  //msg.course = chi_d;
 // m_controllerPub.publish(msg);//发布消息
 v=u;
 x_d =chi_d*180/M_PI;

}


void VFALOS(float x, float y,double psi)
{
  nav_msgs::Path m_path;
  m_path = path;
  if(flag_lastTime == true)
  {
    flag_lastTime = false;
    deltaTime = 0.1;
  }//当前时间赋值
  else
  {
    deltaTime = (ros::Time::now() - lastTime).toSec();//将Time转为double型时间，获取double型的时间间隔
  }
  lastTime = ros::Time::now();
  if (path.poses.size() <= 1)//如果算法计算的位姿数量小于等于1
  {
    //pao=0;//;
  }
  std::vector<geometry_msgs::PoseStamped>::iterator closest;//声明最近点迭代器
  double minDist = std::numeric_limits<double>::max();//最大界限
  for (auto it = m_path.poses.begin(); it != m_path.poses.end(); it++)//遍历m_path.poses，寻找算法得到的点中离当前位置最近的点
  {
    double dist = std::sqrt(std::pow(x - it->pose.position.x, 2) + std::pow(y - it->pose.position.y, 2));//距离计算
    if (dist < minDist)
    {
      minDist = dist;//更新最短距离
      closest = it;//更新最近点
    }
  }
   //误差计算 自己加的 王后可以去掉
  if(minDist < 1.0)
  {
    first = false;
  }
  if(first == false)
  {
    wucha = minDist;
  }
  // Store closest储存最近点
  geometry_msgs::PoseStamped pose_d = *closest;
  
  bool isTurning = false;
  double delta_y_e = delta_min;
  if((closest + 1) != m_path.poses.end())
  {
    geometry_msgs::PoseStamped pose_next = *(closest + 1);
    double gamma_p = tf2::getYaw(pose_d.pose.orientation);
    double gamma_next = tf2::getYaw(pose_next.pose.orientation);
    //ROS_INFO_STREAM("gamma_p: " << gamma_p << ", gamma_next: " << gamma_next);
    //ROS_INFO_STREAM("pose_d: (" << pose_d.pose.position.x << ", " << pose_d.pose.position.y << ")");
    //ROS_INFO_STREAM("pose_next: (" << pose_next.pose.position.x << ", " << pose_next.pose.position.y << ")");
    if (std::fabs(gamma_p - gamma_next) > std::numeric_limits<double>::epsilon())
    {
      isTurning = true;//转弯 判断是否往哪转吧
      if (std::fabs(gamma_next - gamma_p) > M_PI)
      {
        if (gamma_next > (M_PI/2) && gamma_p < -(M_PI/2))
        {
          turn_angle = gamma_next - M_PI*2 - gamma_p;
        }
        else if (gamma_p > (M_PI/2) && gamma_next < -(M_PI/2))
        {
          turn_angle = gamma_next + M_PI*2 - gamma_p;
        }
      }
      else
      {
        turn_angle = gamma_next - gamma_p;
      }
      if (turn_angle > 0)
      {
        turningDir = 1;
      }
      else // dir == Right
      {
        turningDir = -1;
      }
      //ROS_INFO_STREAM("turn_angle: " << turn_angle << ", sin(turn_angle): " << sin(turn_angle));
      
      turning_radius = (std::sqrt(std::pow(pose_next.pose.position.x - pose_d.pose.position.x, 2) + std::pow(pose_next.pose.position.y - pose_d.pose.position.y, 2)) / 2) / sin(std::abs(turn_angle) / 2);// 转弯半径
      r_v = atan(0.3*turning_radius)*2/M_PI*1 + turning_radius-1;
      //ROS_INFO_STREAM("turning_radius: " << turning_radius <<", r_v: " << r_v);
      
      double x_cr1 = pose_d.pose.position.x + sin(gamma_p) * turning_radius;
      double y_cr1 = pose_d.pose.position.y - cos(gamma_p) * turning_radius;
      double x_cr2 = pose_d.pose.position.x - sin(gamma_p) * turning_radius;
      double y_cr2 = pose_d.pose.position.y + cos(gamma_p) * turning_radius;
      
      if (-(x_cr1 - pose_d.pose.position.x) * sin(gamma_p) + (y_cr1 - pose_d.pose.position.y) * cos(gamma_p) > 0)
      // cr1 is left turning point
      {
        if (turningDir == 1)
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
        if (turningDir == -1)
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
      //ROS_INFO_STREAM("cr: (" << x_cr << ", " << y_cr << ")");
    }
  }
  // Erase previous elements删除以前的元素
  m_path.poses.erase(m_path.poses.begin(), closest);
  // Path tangential angle路径切向角γp
  double gamma_p = tf2::getYaw(pose_d.pose.orientation);
  // Cross-track error跨轨误差y(e)
  double y_e = -(x - pose_d.pose.position.x) * std::sin(gamma_p) + (y - pose_d.pose.position.y) * std::cos(gamma_p);
  // Time-varying lookahead distance时变前瞻距离Δy(e)，前瞻距离Δ决定转向的积极性
  //isTurning = false时
  if (isTurning == false)
  {
    double delta_y_e = (delta_max - delta_min + 1.0) * std::exp(-delta_k * std::pow(y_e, 2)) + delta_min+5.0;
  }
  double chi_r, chi_d, beta;
  if (isTurning == false)
  {
    //计算路线
    // velocity-path relative angle速度路径相对角chi_r=arctan(-ye/Δy(e))
    chi_r = std::atan(-y_e / delta_y_e);
    beta = 0.0006 * delta_y_e * y_e / std::sqrt(std::pow(delta_y_e, 2) + std::pow(y_e, 2));
    // desired course angle期望航线角xd=路径切向角γp+chi_r
    chi_d = gamma_p - beta + chi_r;
  }
  else
  {
    double d_cr = std::sqrt(std::pow(x - x_cr, 2) + std::pow(y - y_cr, 2));
    double gamma_cr = std::atan2(y - y_cr, x - x_cr);
    if (turningDir == 1) //turn left
    {
      chi_r = std::atan((d_cr - r_v) / delta_y_e);
      beta = 0.0006 * delta_y_e * (-d_cr + r_v) / std::sqrt(std::pow(delta_y_e, 2) + std::pow((-d_cr + r_v), 2));
      chi_d = gamma_cr + M_PI_2 - beta + chi_r;
    }
    else
    {
      chi_r = std::atan((-d_cr + r_v) / delta_y_e);
      beta = 0.0006 * delta_y_e * (d_cr - r_v) / std::sqrt(std::pow(delta_y_e, 2) + std::pow((d_cr - r_v), 2));
      chi_d = gamma_cr - M_PI_2 - beta + chi_r;
    }
  }

  // calculate error in heading计算航向误差，[-π, π]
  double chi_err = chi_d - psi;
  while (chi_err > M_PI)
  {
    chi_err -= 2 * M_PI;
  }
  while (chi_err < -M_PI)
  {
    chi_err += 2 * M_PI;
  }
  // calculate desired speed计算isTurning = false时的期望速度u
  double u = m_maxSpeed * (1 - std::abs(y_e) / 5 - std::abs(chi_err) / M_PI_2);
  u = std::max(u, m_minSpeed);
  if (isTurning)//如果转弯
  u = m_maxSpeedTurn;//赋值最大转弯速度
  
  v = u;
  x_d =chi_d*180/M_PI;
}

//前头擦线 中心计算
void VFALOS2(double x, double y, double psi)
// TODO: cuts turns, how to fix?
{
  //////////////////////////////////////////////////////////////////////////////////////  
  // Identify closest point on path识别路径上的最近点 (7.6)式
  std::vector<geometry_msgs::PoseStamped>::iterator closest2;//声明最近点迭代器
  double minDist2 = std::numeric_limits<double>::max();//最大界限
  for (auto it = path.poses.begin(); it != path.poses.end(); it++)//寻找算法得到的点中离当前位置最近的点
  {
    //距离计算  pow()用于返回第一个参数的第二个参数次幂的值
    double dist2 = std::sqrt(std::pow(x_qian - it->pose.position.x, 2) +std::pow(y_qian - it->pose.position.y, 2));
    if (dist2 < minDist2)
    {
      minDist2 = dist2;//更新最短距离
      closest2 = it;//更新最近点
    }
  }
  // Erase previous elements
  path.poses.erase(path.poses.begin(), closest2);//注意这个2

  ////算误差
  std::vector<geometry_msgs::PoseStamped>::iterator closest3;//声明最近点迭代器
  double minDist3 = std::numeric_limits<double>::max();//最大界限
  for (auto it = path2.poses.begin(); it != path2.poses.end(); it++)//寻找算法得到的点中离当前位置最近的点
  {
    //距离计算  pow()用于返回第一个参数的第二个参数次幂的值
    double dist3 = std::sqrt(std::pow(x - it->pose.position.x, 2) +std::pow(y - it->pose.position.y, 2));
    if (dist3 < minDist3)
    {
      minDist3 = dist3;//更新最短距离
      closest3 = it;//更新最近点
    }
  }
  //误差计算 自己加的 王后可以去掉
  if(minDist3 < 1.0)
  {
    first = false;
  }
  if(first == false)
  {
    wucha = minDist3;
  }
  /////////////////////////////////////////////////////////////////////////////////////////////////
  // 船后面的点与路径最近的距离
  std::vector<geometry_msgs::PoseStamped>::iterator closest;//声明最近点迭代器
  double minDist = std::numeric_limits<double>::max();//最大界限
  for (auto it = path.poses.begin(); it != path.poses.end(); it++)//寻找算法得到的点中离当前位置最近的点
  {
    //距离计算  pow()用于返回第一个参数的第二个参数次幂的值
    double dist = std::sqrt(std::pow(x - it->pose.position.x, 2) +std::pow(y - it->pose.position.y, 2));
    if (dist < minDist)
    {
      minDist = dist;//更新最短距离
      closest = it;//更新最近点
    }
  }
  ////////////////////////////////////////////////////////////////////////////////////////////////
  // Store closest
  geometry_msgs::PoseStamped pose_d = *closest;

  // Path tangential angle
  double gamma_p = tf2::getYaw(pose_d.pose.orientation);

  // Cross-track error
  double y_e = -(x - pose_d.pose.position.x) * std::sin(gamma_p) + (y - pose_d.pose.position.y) * std::cos(gamma_p);

  // Time-varying lookahead distance
  double delta_y_e = (delta_max - delta_min) * std::exp(-delta_k * std::pow(y_e, 2)) + delta_min;
  // if turning => small lookahead distance
  // 判断下一个点是否要转弯
  bool isTurning = false;
  if ((closest + 1) != path.poses.end())
  {
    double nextAngle = tf2::getYaw((*(closest + 1)).pose.orientation);
    if (std::fabs(gamma_p - nextAngle) > std::numeric_limits<double>::epsilon())
    {
      delta_y_e = delta_min;
      isTurning = true;
    }
  }

  // velocity-path relative angle
  double chi_r = std::atan(-y_e / delta_y_e);

  // desired course angle
  double chi_d = gamma_p + chi_r;

  // calculate error in heading
  double chi_err = chi_d - psi;
  while (chi_err > M_PI)
  {
    chi_err -= 2 * M_PI;
  }
  while (chi_err < -M_PI)
  {
    chi_err += 2 * M_PI;
  }

  // calculate desired speed
  double u = m_maxSpeed * (1 - std::abs(y_e) / 5 - std::abs(chi_err) / M_PI_2);
  u = std::max(u, m_minSpeed);
  if (isTurning)
  {
    u = m_maxSpeedTurn;
  }
  v = u;
  x_d =chi_d*180/M_PI;
}


geometry_msgs::PoseStamped startPose;
geometry_msgs::PoseStamped goalPose;
void dubinpath(double xx1,double yy1,double an1,double xx2,double yy2,double an2)
{
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
  double yaw = std::atan2(goalPose.pose.position.y - startPose.pose.position.y, goalPose.pose.position.x - 
              startPose.pose.position.x);
  q.setRPY(0, 0, an2);
  goalPose.pose.orientation.x = q.x();
  goalPose.pose.orientation.y = q.y();
  goalPose.pose.orientation.z = q.z();
  goalPose.pose.orientation.w = q.w();
}    


int main(int argc, char **argv)
{
	// ROS节点初始化
	ros::init(argc, argv, "v2run");
	// 创建节点句柄
	ros::NodeHandle n;
	// ros::Publisher attitude_pub = n.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_attitude/attitude", 10);
	// ros::Publisher local_pose = n.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
	//发送航向信息
	ros::Publisher raw_pub = n.advertise<mavros_msgs::AttitudeTarget>("rover2/mavros/setpoint_raw/attitude",10);
	// 发送速度
	// ros::Publisher vel_pub = n.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
	// Pub发布话题simple_dubins_path，消息nav_msgs::Path给guidance
  ros::Publisher pathPub = n.advertise<nav_msgs::Path>("rover2/simple_dubins_path", 1000);
	// 创建一个Subscriber，订阅名为mavros/global_position/global的topic，注册回调函数poseCallback 订阅相经纬度
  ros::Subscriber gps_sub = n.subscribe("rover2/mavros/global_position/global", 10, gpsCallback);
	// 创建一个Subscriber，订阅名为mavros/local_position/pose的topic，注册回调函数poseCallback 订阅相对位置
  ros::Subscriber pose_sub = n.subscribe("rover2/mavros/local_position/pose", 10, poseCallback);
 	// 创建一个Publisher，发布名为/xyzuobiao的topic，消息类型为guidance::my_path，队列长度100
  // ros::Publisher zuobiao_pub = n.advertise<guidance::my_path>("/xyzuobiao", 10);
	// 创建一个Subscriber，订阅名为mavros/imu/data的topic，注册回调函数poseCallback 订阅四元数
  ros::Subscriber angles_sub = n.subscribe("rover2/mavros/imu/data", 10, anglesCallback);
 	// 创建一个Subscriber，订阅名为mavros/local_position/velocity_body的topic，注册回调函数vel_callback订阅四元数
  ros::Subscriber v_sub = n.subscribe("rover2//mavros/local_position/velocity_body", 10, vel_callback);
  //USV位置可视化
  ros::Publisher marker1_pub = n.advertise<visualization_msgs::Marker>("usv_qian", 100);
  //USV位置可视化
  ros::Publisher marker2_pub = n.advertise<visualization_msgs::Marker>("usv_zhongxin", 100);
  //USV位置可视化
  ros::Publisher marker3_pub = n.advertise<visualization_msgs::Marker>("usv_hou", 100);
  //订阅rc输出
  ros::Subscriber sub = n.subscribe("rover2//mavros/rc/out", 10, rcCallback);

	
  visualization_msgs::Marker marker1;
  marker1.header.frame_id = "map";
  marker1.type = visualization_msgs::Marker::SPHERE;
  marker1.action = visualization_msgs::Marker::ADD;
  marker1.scale.x = 0.5;
  marker1.scale.y = 0.5;
  marker1.scale.z = 0.5;
  marker1.color.r = 0.0;
  marker1.color.g = 1.0;
  marker1.color.b = 0.0;
  marker1.color.a = 1.0;

  visualization_msgs::Marker marker2;
  marker2.header.frame_id = "map";
  marker2.type = visualization_msgs::Marker::SPHERE;
  marker2.action = visualization_msgs::Marker::ADD;
  marker2.scale.x = 0.5;
  marker2.scale.y = 0.5;
  marker2.scale.z = 0.5;
  marker2.color.r = 1.0;
  marker2.color.g = 0.0;
  marker2.color.b = 0.0;
  marker2.color.a = 1.0;

  visualization_msgs::Marker marker3;
  marker3.header.frame_id = "map";
  marker3.type = visualization_msgs::Marker::SPHERE;
  marker3.action = visualization_msgs::Marker::ADD;
  marker3.scale.x = 0.5;
  marker3.scale.y = 0.5;
  marker3.scale.z = 0.5;
  marker3.color.r = 0.0;
  marker3.color.g = 0.0;
  marker3.color.b = 1.0;
  marker3.color.a = 1.0;

	float roll_deg = 0;
 	float pitch_deg = 0;
 	float yaw_deg =0;

	// guidance::my_path jilu;
	guaijiao::SimpleDubinsPath my_dubin;

  path.poses.clear();///可能要改
  path2.poses.clear();///可能要改

  int shu = 4;//路经点个数
  for (int i = 0; i < shu; i++)  
  {
    if(i==0)
    {
      double yaw1 = std::atan2(p1[i+1][1] - p1[i][1], p1[i+1][0] - p1[i][0]);
      double yaw2 = yaw1;
      dubinpath(p1[i][0], p1[i][1], yaw1, p1[i+1][0],p1[i+1][1], yaw2);//(startPose,goalPose,path);
      my_dubin.makePath(startPose,goalPose,path);//
      my_dubin.makePath(startPose,goalPose,path2);//
    }
    if(i>0 && i<(shu-1))
    {
      double yaw1 = std::atan2(p1[i][1] - p1[i-1][1], p1[i][0] - p1[i-1][0]);
      double yaw2 = std::atan2(p1[i+1][1] - p1[i][1], p1[i+1][0] - p1[i][0]);
      dubinpath(p1[i][0], p1[i][1], yaw1, p1[i+1][0],p1[i+1][1], yaw2);
      my_dubin.makePath(startPose,goalPose,path);
      my_dubin.makePath(startPose,goalPose,path2);
    }
  }
// 设置循环的频率
  ros::Rate loop_rate(10);

	while (ros::ok())
	{	
		if((pointDistance(x_usv,y_usv,zuihoudian[0][0],zuihoudian[0][1])) < 1.0)
		{
			v=0.0;
		}

    x_qian = x_usv + 1.0*std::cos(angles.yaw);
    y_qian = y_usv + 1.0*std::sin(angles.yaw);

    x_hou = x_usv - (L/2.0)*std::cos(angles.yaw);
    y_hou = y_usv - (L/2.0)*std::sin(angles.yaw);

    dingyue_Hz = dingyue_Hz +1;
    if(dingyue_Hz >= 10)
    {
      dingyue_Hz = 0;
      usv_zhihou_yaw = angles.yaw;
    }
		if(path.poses.size() > 0)
		{
		  Time_LOS(x_usv, y_usv, angles.yaw);//usv_zhihou_yaw
		}
    else
    {
      v=0;
    }
    yaw_deg=x_d;
		from_euler(roll_deg*M_PI/180, pitch_deg*M_PI/180, yaw_deg*M_PI/180);
		mavros_msgs::AttitudeTarget attitude_raw;
		attitude_raw.orientation.w = att_quat.q1;
		attitude_raw.orientation.x = att_quat.q2;
		attitude_raw.orientation.y = att_quat.q3;
		attitude_raw.orientation.z = att_quat.q4;
		attitude_raw.body_rate.z = 1;
		attitude_raw.thrust = v;///油门值
		attitude_raw.type_mask =0b00000111 ;//0b00000111
		raw_pub.publish(attitude_raw);

    // ROS_INFO("usv_zhihou_yaw: %.3f, x_d: %.3f, usv_yaw: %.3f , dingyue_Hz: %d", 
    //           usv_zhihou_yaw      , x_d ,      angles.yaw    , dingyue_Hz);

    ROS_INFO("x:%.3f, y:%.3f", x_usv, y_usv);

		// jilu.y=y_usv;
		// jilu.x=x_usv;
		// jilu.usv_vx=vx;
		// jilu.my_vx=v;
		// jilu.usv_yaw=angles.yaw*180/M_PI;
		// jilu.my_yaw=x_d;
		// jilu.shijian = sj;
		// jilu.wucha = wucha;
		// zuobiao_pub.publish(jilu);

    pathPub.publish(path);
    // pathPub.publish(path2);

    marker1.pose.position.x = x_qian;
    marker1.pose.position.y = y_qian;
    marker1_pub.publish(marker1);

    marker2.pose.position.x = x_usv;
    marker2.pose.position.y = y_usv;
    marker2_pub.publish(marker2);

    marker3.pose.position.x = x_hou;
    marker3.pose.position.y = y_hou;
    marker3_pub.publish(marker3);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
	//ROS_INFO("x:%.3f, y:%.3f ", xx_usv, yy_usv);
/*

	
*/



/*
 for(int i=0;i<2;i++)
	{	

	   loop_rate.sleep();
	}

	//for(int i=0;i<10;i=i+2)
	//{	
		dubinpath(4);
		my_dubin.makePath(startPose,goalPose,path);
 		 	//newpath();
 for(int i=0;i<2;i++)
	{	


	   loop_rate.sleep();
	}
	for (auto it = path.poses.begin(); it != path.poses.end(); it++)//得到的点中离当前位置最近的
  {
    jilu.x=it->pose.position.x;
    jilu.y=it->pose.position.y;
    zuobiao_pub.publish(jilu);
    ROS_INFO("x:%.3f, y:%.3f", it->pose.position.x, it->pose.position.y);
    for(int i=0;i<1;i++)
    { 
      loop_rate.sleep();
    }
  }
//}
	while (ros::ok())
	{
		

		loop_rate.sleep();
	}
*/






