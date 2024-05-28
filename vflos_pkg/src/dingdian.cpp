#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
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
// #include <guidance/guidance.h>
// #include "guidance/my_path.h"
// #include <iostream>

float x_usv=0,y_usv=0;
float yaw=0;
float sudu=0;
float x_u=0;
double x_d=0;
int pao;
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
double delta_max = 4.0;
double delta_min = 1.0;
double delta_k = 1.0;
double DELTA = 0.5;

double m_maxSpeed = 0.5;
double m_maxSpeedTurn = 0.2;
double m_minSpeed = 0.1;
// guidance::my_path jilu;

//////////////////////////////////////////////////////////////////////////////////////
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

//直线段生成
nav_msgs::Path path;
////followdubinPath///////////////////
void followPath(float x, float y,double psi)//
{
    if(flag_lastTime == true)
    {
        flag_lastTime = false;
        deltaTime = 0.1;
    }//当前时间赋值
    else
    {
        deltaTime = (ros::Time::now() - lastTime).toSec();
    }
    lastTime = ros::Time::now();

    if (path.poses.size() <= 1)
    {

    }

    std::vector<geometry_msgs::PoseStamped>::iterator closest;
    double minDist = std::numeric_limits<double>::max();//最大界限
    double dist =0.0;
    for (auto it = path.poses.begin(); it != path.poses.end(); it++)
    {
        dist= std::sqrt(std::pow(x - it->pose.position.x, 2) +
        std::pow(y - it->pose.position.y, 2));
        if (dist < minDist)
        {
            minDist = dist;//更新最短距离
            closest = it;//更新最近  
        }
    }
    // Store closest储存最近点 pose_d
    geometry_msgs::PoseStamped pose_d = *closest;
    // Erase previous elements删除以前的元素
    path.poses.erase(path.poses.begin(), closest);
    // Path tangential angle路径切向角γp 
    double gamma_p = tf2::getYaw(pose_d.pose.orientation);
    // Cross-track error跨轨误差y(e)   (7.7)式
    double y_e = -(x - pose_d.pose.position.x) * std::sin(gamma_p) + (y - pose_d.pose.position.y) * std::cos(gamma_p);
    double angle_ex = std::abs(gamma_p - psi);
    if(angle_ex > M_PI)
    {
        angle_ex = 2 * M_PI - angle_ex;
    }
    double delta_y_e = (angle_ex*std::min(std::exp(std::abs(1/y_e)+0.6), 100.0))*std::abs(delta_max- delta_min) * std::exp(-delta_k * std::pow(y_e, 2)) + delta_min;

    bool isTurning = false;
    if ((closest + 1) != path.poses.end())//如果最近点不是goalPose
    {
        double nextAngle = tf2::getYaw((*(closest + 1)).pose.orientation);
        if (std::fabs(gamma_p - nextAngle) > std::numeric_limits<double>::epsilon() && minDist<0.1)
        {
            delta_y_e = delta_min;
            isTurning = true;//转弯/////
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
    }
    double cur_u, cur_v, U;
    cur_u = delta_x * std::cos(psi) + delta_y * std::sin(psi);
    cur_v = -delta_y * std::cos(psi) + delta_x * std::sin(psi);
    U = std::sqrt(std::pow(cur_u, 2) + std::pow(cur_v, 2));
    double d_cr = std::sqrt(std::pow(x - x_cr, 2) + std::pow(y - y_cr, 2));
    if (isTurning == false)
    {
        D = U * y_e / std::sqrt(std::pow((y_e + sigma*dint), 2) + std::pow(delta_y_e, 2));
    }
    else
    {
        D = U * (d_cr - r_v) / std::sqrt(std::pow((d_cr - r_v + sigma*dint), 2) + std::pow(delta_y_e, 2));
    }
    dintn = dint + D * deltaTime;
    dint = dintn;

    double chi_r, chi_d;
    if (isTurning == true)
    {
        double gamma_cr =
        std::atan2(y - y_cr, x - x_cr);
        if (turningDir == 1) //turn left
        {
            chi_r = std::atan((d_cr - r_v + sigma*dintn) / delta_y_e);
            chi_d = gamma_cr - M_PI_2 - chi_r;//chi_d = gamma_cr + M_PI_2 + chi_r
            //ROS_INFO("zuozuozuo" );
        }
        else
        {
            chi_r = std::atan((-d_cr + r_v + sigma*dintn) / delta_y_e);
            chi_d = gamma_cr + M_PI_2 - chi_r;// chi_d = gamma_cr - M_PI_2 + chi_r;
            // ROS_INFO("youyouyou" );
        }
    }
    else
    {
        chi_r = std::atan((-y_e - sigma*dintn) / delta_y_e);
        chi_d = gamma_p + chi_r;
         // ROS_INFO("wuwuwuwu" );
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
    {
        u = m_maxSpeedTurn;//赋值
    }
    x_u=u;                 //期望速度 0到1

    x_d =chi_d*180/M_PI;   //期望角度 单位度

    if(x_d < 0)
    {
        x_d = 360.0 + x_d;
    }
}

//接收船的位置信息///////////////////////////////////////////////////////////////////////////////////////////////////
void gpsCallback(const sensor_msgs::NavSatFix& msg)
{
    double dx = std::cos(msg.latitude*M_PI/180) * (msg.longitude - lonO) * 111319.9 ;
    double dy = (msg.latitude - latO) * 111319.9 ;
    const double unit = 0.25; //每个单位代表的距离（米）
    x_usv = dx ;
    y_usv = dy ;
    // ROS_INFO("x:%.3f, y:%.3f", dx, dy);

}
void poseCallback(const geometry_msgs::PoseStamped& msg)
{
	//x_usv=msg.pose.position.x;
	//y_usv=msg.pose.position.y;

}
//接收船的角度信息///////////////////////////////////////////////////////////////////////////////////////////////////
void anglesCallback(const sensor_msgs::Imu& msg)
{
	q.x=msg.orientation.x;
	q.y=msg.orientation.y;
	q.z=msg.orientation.z;
	q.w=msg.orientation.w;
	ToEulerAngles(q);//计算出的欧拉角全是弧度
   // ROS_INFO("roll:%.3f, pitch:%.3f,yaw:%.3f", angles.roll*180/M_PI, angles.pitch*180/M_PI, angles.yaw*180/M_PI);
}

int main(int argc, char **argv)
{
  double x_mubiao = 50;       	
  double y_mubiao = 22.5;
  bool varvel_flag = false;
  switch (argc)
  {
  case 2:
    if (atof(argv[1]) == 0)
    {
      varvel_flag = false;
    }
    else
    {
      varvel_flag = true;
    }
    break;

  case 4:
    if (atof(argv[1]) == 0)
    {
      varvel_flag = false;
    }
    else
    {
      varvel_flag = true;
    }
    x_mubiao = atof(argv[2]);
    y_mubiao = atof(argv[3]);
    break;

  default:
    break;
  }
    // ROS节点初始化
    ros::init(argc, argv, "v2juxing");
    // 创建节点句柄
    ros::NodeHandle n;

    // ros::Publisher attitude_pub = n.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_attitude/attitude", 10);

    // ros::Publisher local_pose = n.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    //发送航向信息
    ros::Publisher raw_pub = n.advertise<mavros_msgs::AttitudeTarget>("rover2/mavros/setpoint_raw/attitude",10);

    // 发送速度
    // ros::Publisher vel_pub = n.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);

    // m_pathPub发布话题simple_dubins_path，消息nav_msgs::Path给guidance
    // ros::Publisher pathPub = n.advertise<nav_msgs::Path>("simple_dubins_path", 1000);

    // 创建一个Subscriber，订阅名为mavros/global_position/global的topic，注册回调函数poseCallback 订阅相经纬度
    ros::Subscriber gps_sub = n.subscribe("rover2/mavros/global_position/global", 10, gpsCallback);

    // 创建一个Subscriber，订阅名为mavros/local_position/pose的topic，注册回调函数poseCallback 订阅相对位置
    ros::Subscriber pose_sub = n.subscribe("rover2/mavros/local_position/pose", 10, poseCallback);

    // 创建一个Publisher，发布名为/xyzuobiao的topic，消息类型为guidance::my_path，队列长度100
    // ros::Publisher zuobiao_pub = n.advertise<guidance::my_path>("/xyzuobiao", 100);

    // 创建一个Subscriber，订阅名为mavros/imu/data的topic，注册回调函数poseCallback 订阅四元数
    ros::Subscriber angles_sub = n.subscribe("rover2/mavros/imu/data", 10, anglesCallback);

	// 设置循环的频率
	ros::Rate loop_rate(10);
	
    float roll_deg = 0;
    float pitch_deg = 0;
    float yaw_deg =0;

    float thrust;
	
	for(int i=0;i<5;i++)
	{	
	   loop_rate.sleep();
	}
	ros::Time start_time = ros::Time::now();
	while (ros::ok())
	{	
        // 变速度因子
        ros::Time current_time = ros::Time::now();
        if (varvel_flag)
        {
          double interval = current_time.sec - start_time.sec;
          thrust = (sin(0.07 * interval) + 1) / 2;
          // ROS_INFO("varvel");
        }
        else
        {
          thrust = 1.0;
          // ROS_INFO("no varvel");
        }
        // float interval = current_time.sec - start_time.sec;
        // ROS_INFO("time:%.3f", interval);
        path.poses.clear();///可能要改
        double dx = x_mubiao - x_usv;
        double dy = y_mubiao - y_usv;
        double dx_norm = dx / std::sqrt(dx * dx + dy * dy);
        double dy_norm = dy / std::sqrt(dx * dx + dy * dy);
        tf2::Quaternion q;
        q.setRPY(0, 0, std::atan2(dy, dx));

          // generate straight line segment 生成直线段，path.poses最后没有放入goalPose
        for (double i = 0;
            std::fabs(i * 0.05 * dx_norm - dx) > 2 * 0.05 ||
            std::fabs(i * 0.05 * dy_norm - dy) > 2 * 0.05;
            ++i)
        {
            geometry_msgs::PoseStamped point;
            point.header.stamp = ros::Time::now();
            point.header.frame_id = "map";
            point.pose.position.x = x_usv + i * 0.05 * dx_norm; // 变量 是起始坐标
            point.pose.position.y = y_usv + i * 0.05 * dy_norm; // 变量
            point.pose.orientation.x = q.x();
            point.pose.orientation.y = q.y();
            point.pose.orientation.z = q.z();
            point.pose.orientation.w = q.w();
            path.poses.push_back(point);
        }
        if(path.poses.size() >= 1)//制导 必须加是否有路径生成 不然会核心以转出
        {
            followPath(x_usv, y_usv ,angles.yaw);
        }
        yaw_deg=x_d;
        from_euler(roll_deg*M_PI/180, pitch_deg*M_PI/180, yaw_deg*M_PI/180);

        mavros_msgs::AttitudeTarget attitude_raw;//发布的期望姿态

        attitude_raw.orientation.w = att_quat.q1;
        attitude_raw.orientation.x = att_quat.q2;
        attitude_raw.orientation.y = att_quat.q3;
        attitude_raw.orientation.z = att_quat.q4;
        attitude_raw.body_rate.z = 3;
        attitude_raw.thrust = thrust;//油门值
        attitude_raw.type_mask =0b00000111 ;//0b00000111
        raw_pub.publish(attitude_raw);
        ROS_INFO("x:%.3f, y:%.3f", x_usv, y_usv);
        // ROS_INFO("x:%.3f, y:%.3f , yaw:%.3f", xx_usv, yy_usv,yaw_deg);

        // zuobiao_pub.publish(jilu);

        ros::spinOnce();
        loop_rate.sleep();
	}
	return 0;
}
/*

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
  double y_e = -(x - pose_d.pose.position.x) * std::sin(gamma_p) +
               (y - pose_d.pose.position.y) * std::cos(gamma_p);
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
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
     // ROS_INFO_STREAM("gamma_cr+pi/2: " << (gamma_cr + M_PI_2) << ", gamma_p: " << gamma_p);
    }
    else
    {
      chi_r = std::atan((-d_cr + r_v - sigma*dintn) / delta_y_e);
      chi_d = gamma_cr - M_PI_2 + chi_r;
     // ROS_INFO_STREAM("gamma_cr-pi/2: " << (gamma_cr - M_PI_2) << ", gamma_p: " << gamma_p);
    }
  }
  else
  {
    chi_r = std::atan((-y_e - sigma*dintn) / delta_y_e);
    chi_d = gamma_p + chi_r;
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

 x_d =chi_d*180/M_PI;

/*
//////时变

// Time-varying lookahead distance时变前瞻距离Δy(e)，前瞻距离Δ决定转向的积极性  (7.11)式
  double delta_max = 4.0;
  double delta_min = 1.0;
  double delta_k = 1;
  double delta_y_e =(delta_max - delta_min) * std::exp(-delta_k * std::pow(y_e, 2)) + delta_min;

  // if turning => small lookahead distance如果转弯 => wr小前瞻距离
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

//计算路线
  // velocity-path relative angle速度路径相对角chi_r=arctan(-ye/Δy(e))
  double chi_r = std::atan(-y_e / delta_y_e);
  // desired course angle期望航线角xd=路径切向角γp+chi_r
  double chi_d = gamma_p + chi_r;

 x_d =chi_d*180/M_PI;
*/




	/*
		//角度速度控制
		from_euler(roll_deg*M_PI/180, pitch_deg*M_PI/180, yaw_deg*M_PI/180);
		mavros_msgs::AttitudeTarget attitude_raw;
    		attitude_raw.orientation.w = att_quat.q1;
    		attitude_raw.orientation.x = att_quat.q2;
    		attitude_raw.orientation.y = att_quat.q3;
    		attitude_raw.orientation.z = att_quat.q4;
    		attitude_raw.thrust = 1;//速度
    		attitude_raw.type_mask = 0b00000111;
		local_attitude_pub.publish(attitude_raw);
		*/
	
		/*

		geometry_msgs::TwistStamped vel_msg;
		vel_msg.twist.linear.x = 5.0;
		vel_msg.twist.linear.y = 0.0;
		//vel_pub.publish(vel_msg);
		*/
/* 	
// Identify closest point on path识别路径上的最近点
  	std::vector<geometry_msgs::PoseStamped>::iterator closest;//声明最近点迭代器
  	double minDist = std::numeric_limits<double>::max();//最大界限
  	for (auto it = path.poses.begin(); it != path.poses.end(); it++)//遍历m_path.poses，寻找算法得到的点中离当前位置最近的点
  	{
   	 //double dist = std::sqrt(std::pow(x - it->pose.position.x, 2) +std::pow(y - it->pose.position.y, 2));
    	ROS_INFO("[%0.2f , %0.2f ]", it->pose.position.x, it->pose.position.y);
  	}
	*/

/*
// Finished?
  if (path.poses.size() <= 1)//如果算法计算的位姿数量小于等于1
  {
    
  }

  // Identify closest point on path识别路径上的最近点
  std::vector<geometry_msgs::PoseStamped>::iterator closest;//声明最近点迭代器
  double minDist = std::numeric_limits<double>::max();//最大界限
  for (auto it = path.poses.begin(); it != path.poses.end(); it++)//遍历m_path.poses，寻找算法得到的点中离当前位置最近的点
  {
    double dist = std::sqrt(std::pow(x - it->pose.position.x, 2) +
                            std::pow(y - it->pose.position.y, 2));//距离计算
    if (dist < minDist)
    {
      minDist = dist;//更新最短距离
      closest = it;//更新最近点
    }
  }

  // Store closest储存最近点
  geometry_msgs::PoseStamped pose_d = *closest;
  
  bool isTurning = false;
  double delta_y_e = delta_min;
  if((closest + 1) != path.poses.end())
  {
    geometry_msgs::PoseStamped pose_next = *(closest + 1);
    double gamma_p = tf2::getYaw(pose_d.pose.orientation);
    double gamma_next = tf2::getYaw(pose_next.pose.orientation);
    //ROS_INFO_STREAM("gamma_p: " << gamma_p << ", gamma_next: " << gamma_next);
    //ROS_INFO_STREAM("pose_d: (" << pose_d.pose.position.x << ", " << pose_d.pose.position.y << ")");
    //ROS_INFO_STREAM("pose_next: (" << pose_next.pose.position.x << ", " << pose_next.pose.position.y << ")");
    
    if (std::fabs(gamma_p - gamma_next) > std::numeric_limits<double>::epsilon())
    {
      isTurning = true;//转弯
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
  path.poses.erase(path.poses.begin(), closest);

  // Path tangential angle路径切向角γp
  double gamma_p = tf2::getYaw(pose_d.pose.orientation);

  // Cross-track error跨轨误差y(e)
  double y_e = -(x - pose_d.pose.position.x) * std::sin(gamma_p) +
               (y - pose_d.pose.position.y) * std::cos(gamma_p);

  // Time-varying lookahead distance时变前瞻距离Δy(e)，前瞻距离Δ决定转向的积极性
  //isTurning = false时
  if (isTurning == false)
  {
    double delta_y_e =
      (delta_max - delta_min + 1.0) * std::exp(-delta_k * std::pow(y_e, 2)) +
      delta_min+5.0;
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
    double d_cr = std::sqrt(std::pow(x - x_cr, 2) +
                    std::pow(y - y_cr, 2));
    double gamma_cr =
        std::atan2(y - y_cr, x - x_cr);
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
  ROS_INFO_STREAM("chi_d = " << chi_d << ", chi_r = " << chi_r << ", beta = " << beta << ", -y_e = " << -y_e << ", delta_y_e = " << delta_y_e);

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
  
 x_d =chi_d*180/M_PI;
  //ROS_INFO_STREAM("psi_d: " << chi_d << " psi: " << psi);
  //ROS_INFO_STREAM("u_d: " << u);


*/

//geometry_msgs::TwistStamped vel_msg;
	//vel_msg.twist.linear.x = 5.0;
	//vel_msg.twist.linear.y = 0.0;
	//vel_pub.publish(vel_msg);
