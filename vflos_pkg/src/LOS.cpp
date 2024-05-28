//标准LOS/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void followdubinPath(float x, float y,double psi)
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
  const double epsilon = std::numeric_limits<double>::epsilon(); //ε(极小值)
  // Identify closest point on path识别路径上的最近点 (7.6)式
  std::vector<geometry_msgs::PoseStamped>::iterator closest;//声明最近点迭代器
  double minDist = std::numeric_limits<double>::max();//最大界限
  for (auto it = path.poses.begin(); it != path.poses.end(); it++)//寻找算法得到的点中离当前位置最近的点
  {
    //距离计算  pow()用于返回第一个参数的第二个参数次幂的值
    double dist = std::sqrt(std::pow(x - it->pose.position.x, 2) + std::pow(y - it->pose.position.y, 2));
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
    qian_wucha = minDist;
    wucha = wucha + qian_wucha;
  }

  // Store closest储存最近点 pose_d
  geometry_msgs::PoseStamped pose_d = *closest;
  // Erase previous elements删除以前的元素
  path.poses.erase(path.poses.begin(), closest);
  // Path tangential angle路径切向角γp 
  double gamma_p = tf2::getYaw(pose_d.pose.orientation);
  //ROS_INFO("x:%.6f, y:%.3f", pose_d.pose.orientation.z, gamma_p);////////////
  //Cross-track error跨轨误差y(e)   (7.7)式
  double y_e = -(x - pose_d.pose.position.x) * std::sin(gamma_p) + (y - pose_d.pose.position.y) * std::cos(gamma_p);

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
  double chi_r = std::atan(-y_e / 3.0);//在这非时变
  // desired course angle期望航线角xd=路径切向角γp+chi_r
  double chi_d = gamma_p + chi_r;
  x_d =chi_d*180/M_PI;
}

//时变LOS/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void followdubinPath(float x, float y,double psi)
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
  const double epsilon = std::numeric_limits<double>::epsilon(); //ε(极小值)
  // Identify closest point on path识别路径上的最近点 (7.6)式
  std::vector<geometry_msgs::PoseStamped>::iterator closest;//声明最近点迭代器
  double minDist = std::numeric_limits<double>::max();//最大界限
  for (auto it = path.poses.begin(); it != path.poses.end(); it++)//寻找算法得到的点中离当前位置最近的点
  {
    //距离计算  pow()用于返回第一个参数的第二个参数次幂的值
    double dist = std::sqrt(std::pow(x - it->pose.position.x, 2) + std::pow(y - it->pose.position.y, 2));
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
    qian_wucha = minDist;
    wucha = wucha + qian_wucha;
  }

  // Store closest储存最近点 pose_d
  geometry_msgs::PoseStamped pose_d = *closest;
  // Erase previous elements删除以前的元素
  path.poses.erase(path.poses.begin(), closest);
  // Path tangential angle路径切向角γp 
  double gamma_p = tf2::getYaw(pose_d.pose.orientation);
  //ROS_INFO("x:%.6f, y:%.3f", pose_d.pose.orientation.z, gamma_p);////////////
  //Cross-track error跨轨误差y(e)   (7.7)式
  double y_e = -(x - pose_d.pose.position.x) * std::sin(gamma_p) + (y - pose_d.pose.position.y) * std::cos(gamma_p);

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
}


// VFILOS//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void followdubinPath(float x, float y,double psi)
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
   //误差计算 自己加的 王后可以去掉
  if(minDist < 1.0)
  {
    first = false;
  }
  if(first == false)
  {
    qian_wucha = minDist;
    wucha = wucha + qian_wucha;
  }


  //Store closest储存最近点 pose_d
  geometry_msgs::PoseStamped pose_d = *closest;
  // Erase previous elements删除以前的元素
  path.poses.erase(path.poses.begin(), closest);


  // Path tangential angle路径切向角γp 
  double gamma_p = tf2::getYaw(pose_d.pose.orientation);
  //ROS_INFO("x:%.6f, y:%.3f", pose_d.pose.orientation.z, gamma_p);////////////

  // Cross-track error跨轨误差y(e)   (7.7)式
  double y_e = -(x - pose_d.pose.position.x) * std::sin(gamma_p) + (y - pose_d.pose.position.y) * std::cos(gamma_p);

  //这是什么变量 路径朝向与船的朝向的差吗
  double angle_ex = std::abs(gamma_p - psi);

  if(angle_ex > M_PI)
  {
    angle_ex = 2 * M_PI - angle_ex;
  }
  //这个也要考虑考虑 delta_y_e是个前视距离
  double delta_y_e =(angle_ex*std::min(std::exp(std::abs(1/y_e)+0.6), 100.0))*std::abs(delta_max- delta_min) * std::exp(-delta_k * std::pow(y_e, 2)) + delta_min;

  bool isTurning = false;
  if ((closest + 1) != path.poses.end())//如果最近点不是goalPose
  { 
    //最近点的下一个点的朝向
    double nextAngle = tf2::getYaw((*(closest + 1)).pose.orientation);
    //ε（极小值）是从1到下一个double数字的距离，杜宾路径最近点和下一个点偏航角差值如果大于ε，说明不是直行
    if (std::fabs(gamma_p - nextAngle) > std::numeric_limits<double>::epsilon())
    {
      //转弯用最小前视距离？
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
  //公式1、6、7 cur_u = (delta_x * std::cos(psi) - delta_y * std::sin(psi)) / Cramer;
  cur_u = delta_x * std::cos(psi) + delta_y * std::sin(psi);
  //cur_v = (delta_y * std::cos(psi) - delta_x * std::sin(psi)) / Cramer;
  cur_v = -delta_y * std::cos(psi) + delta_x * std::sin(psi);
  U = std::sqrt(std::pow(cur_u, 2) + std::pow(cur_v, 2));
  // ROS_INFO_STREAM("U: " << U << ", cur_u: " << cur_u << ", cur_v: " << cur_v);
  
  double d_cr = std::sqrt(std::pow(x - x_cr, 2) +
                    std::pow(y - y_cr, 2));
  //ROS_INFO_STREAM("d-r: " << (d_cr - turning_radius) << ", y_e: " << y_e);
  
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
    double gamma_cr = std::atan2(y - y_cr, x - x_cr);
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
    //ROS_INFO("zwuwuwuwu" );
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

//VFALOS///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void followdubinPath(float x, float y,double psi)
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
    qian_wucha = minDist;
    wucha = wucha + qian_wucha;
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

//参数////////////////////////////

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

double m_maxSpeed = 0.4;
double m_maxSpeedTurn = 0.2;
double m_minSpeed = 0.1;

//原始文献中的LOS
void followdubinPath(double x, double y, double psi)
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
  // Store closest
  geometry_msgs::PoseStamped pose_d = *closest;

  // Erase previous elements
  path.poses.erase(path.poses.begin(), closest);

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