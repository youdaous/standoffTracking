#ifndef NEWPATH_H_
#define NEWPATH_H_

// #include <coverage_boustrophedon/DubinInput.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>



namespace guaijiao
{
	  
class SimpleDubinsPath
{
public:
  SimpleDubinsPath();
  ~SimpleDubinsPath();
  void makePath(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal, nav_msgs::Path& dubinsPath);
  bool getTargetHeading(double x_q, double y_q, double theta_q, double x_n,
                        double y_n, double& yawTarget);

  //void followdubinPath(float x, float y,nav_msgs::Path path);
  void followguaijiaoPath(float x, float y,nav_msgs::Path path);

private:


int pao;

int guai=0;

int R=10;
int qianshi=6;
int hangdian=0;

double at;//式（3-5）的变量

  enum Dir
  {
    Left = 1,
    Right = -1
  };
  enum DubinsPathType
  {
    LSL = 0,
    LSR = 1,
    RSL = 2,
    RSR = 3,
    RLR = 4,
    LRL = 5
  };
  enum SegmentType
  {
    L_SEG = 0,
    S_SEG = 1,
    R_SEG = 2
  };
  struct DubinsPath
  {
    /* the initial configuration */
    double qi[3];        
    /* the lengths of the three segments */
    double param[3];     
    /* model forward velocity / model angular velocity */
    double rho;          
    /* the path type described */
    DubinsPathType type; 
  };
  #define EDUBOK        (0)   /* No error */
  #define EDUBCOCONFIGS (1)   /* Colocated configurations */
  #define EDUBPARAM     (2)   /* Path parameterisitation error */
  #define EDUBBADRHO    (3)   /* the rho value is invalid */
  #define EDUBNOPATH    (4)   /* no connection between configurations with this word */

  struct DubinsIntermediateResults
  {
    double alpha;
    double beta;
    double d;
    double sa;
    double sb;
    double ca;
    double cb;
    double c_ab;
    double d_sq;
  };


  void onGoal(const geometry_msgs::PoseStamped& goal);
  // void onInput(const coverage_boustrophedon::DubinInput& input);
  Dir turningDirection(double x_q, double y_q, double theta_q, double x_n,
                       double y_n);
  void turningCenter(double x_q, double y_q, double theta_q, double& x_cr,
                     double& y_cr, Dir dir);
  void tangentLine(double x_n, double y_n, double x_cr, double y_cr,
                   double& beta1, double& beta2);
  void tangentPoint(double x_q, double y_q, double x_n, double y_n, double x_cr,
                    double y_cr, double beta1, double beta2, Dir dir,
                    double& x_lc, double& y_lc);
  void generatePath(double x_q, double y_q, double x_n, double y_n, double x_cr,
                    double y_cr, double x_lc, double y_lc, Dir dir,
                    const geometry_msgs::PoseStamped& goal,
                    nav_msgs::Path& dubinsPath);
  void generateStraightPath(const geometry_msgs::PoseStamped& start,
                            const geometry_msgs::PoseStamped& goal,
                            nav_msgs::Path& dubinsPath);
  
  int dubins_shortest_path(DubinsPath* path, double q0[3], double q1[3], double rho);
  //int DubinsPathSamplingCallback(double q[3], double t, void* user_data);
  double dubins_path_length(DubinsPath* path);
  void dubins_segment( double t, double qi[3], double qt[3], SegmentType type);
  //double dubins_segment_length(DubinsPath* path, int i);
  //double dubins_segment_length_normalized( DubinsPath* path, int i);
  //DubinsPathType dubins_path_type(DubinsPath* path);
  int dubins_path_sample(DubinsPath* path, double t, double q[3]);
  int dubins_path_sample_many(DubinsPath* path, 
                            double stepSize, 
                            nav_msgs::Path& dubinsPath);
  //int dubins_path_endpoint(DubinsPath* path, double q[3]);
  //int dubins_extract_subpath(DubinsPath* path, double t, DubinsPath* newpath);
  //int dubins_word(DubinsIntermediateResults* in, DubinsPathType pathType, double out[3]);
  int dubins_intermediate_results(DubinsIntermediateResults* in, double q0[3], double q1[3], double rho);
  double mod2pi( double theta );
  int dubins_LSL(DubinsIntermediateResults* in, double out[3]);
  int dubins_RSR(DubinsIntermediateResults* in, double out[3]);
  int dubins_LSR(DubinsIntermediateResults* in, double out[3]);
  int dubins_RSL(DubinsIntermediateResults* in, double out[3]);
  int dubins_RLR(DubinsIntermediateResults* in, double out[3]);
  int dubins_LRL(DubinsIntermediateResults* in, double out[3]);
     const double epsilon = std::numeric_limits<double>::epsilon(); //ε(极小值)
  double m_turningRadius = 1.0;////////////
  double m_pathResolution = 0.1;////////
  //ros::Publisher m_pathPub;
  
  /* The segment types for each of the Path types */
  const SegmentType DIRDATA[6][3] = {
    { L_SEG, S_SEG, L_SEG },
    { L_SEG, S_SEG, R_SEG },
    { R_SEG, S_SEG, L_SEG },
    { R_SEG, S_SEG, R_SEG },
    { R_SEG, L_SEG, R_SEG },
    { L_SEG, R_SEG, L_SEG }
  };

};

} // namespace otter_coverage



#endif
