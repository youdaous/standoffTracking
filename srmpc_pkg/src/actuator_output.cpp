#include <ros/ros.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/RCOut.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/Thrust.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/ManualControl.h>
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>

// TEST for topics:
// manual_control/send
// rc/override
// setpoint_accel

// std::vector<uint16_t> servo_output;
// void serve_output_Callback(const mavros_msgs::RCOut &rcout)
// {
//     // servo_output = rcout.channels;
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "actuator_out");
    ros::NodeHandle n;

    // 发布驱动器控制输出
    ros::Publisher actuator_controlPub = n.advertise<mavros_msgs::ManualControl>("/rover1/mavros/manual_control/send", 10);

    // // 发布RC控制输出
    ros::Publisher RC_controlPub = n.advertise<mavros_msgs::OverrideRCIn>("/rover1/mavros/rc/override", 10);

    // 发布setpoint_accel
    ros::Publisher accelPub = n.advertise<geometry_msgs::Vector3Stamped>("/rover1/mavros/setpoint_accel/accel", 10);

    // 发布setpoint_raw
    ros::Publisher rawattitudePub = n.advertise<mavros_msgs::AttitudeTarget>("/rover1/mavros/setpoint_raw/attitude", 10);
    ros::Publisher rawlocalPub = n.advertise<mavros_msgs::PositionTarget>("/rover1/mavros/setpoint_raw/local", 10);

    // // 订阅servo_output(1000-1500-2000)
    // ros::Subscriber servo_outputSub= n.subscribe("/rover1/mavros/rc/out", 10, serve_output_Callback);

    ros::Rate loop_rate(10);

    // mavros_msgs::ActuatorControl ac_output;
    // ac_output.group_mix = 0;
    // ac_output.controls = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0};

    // geometry_msgs::Vector3Stamped control_output;
    // geometry_msgs::Vector3 force;
    // force.x = 1900;
    // force.y = 0.;
    // force.z = 0.;
    // control_output.vector = force;

    // mavros_msgs::ManualControl control_output;
    // control_output.x = 1900;
    // control_output.y = 0;
    // control_output.z = 0;
    // control_output.r = 0;
    // control_output.buttons = 0xFFFF;

    // mavros_msgs::PositionTarget control_output;
    // control_output.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    // control_output.acceleration_or_force.x = 10;
    // control_output.velocity.x = 2;
    // control_output.type_mask = mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY |
    //                 mavros_msgs::PositionTarget::IGNORE_PZ | mavros_msgs::PositionTarget::IGNORE_VX |
    //                 mavros_msgs::PositionTarget::IGNORE_VY | mavros_msgs::PositionTarget::IGNORE_VZ;

    mavros_msgs::AttitudeTarget control_output;
    // control_output.thrust = 0.8;
    control_output.body_rate.z = 1.5;




    

    // mavros_msgs::Thrust Thrust;
    // Thrust.thrust = 0.8;

    while (ros::ok())
    {
        control_output.header.stamp = ros::Time::now();
        rawattitudePub.publish(control_output);
    
        
        // control_output.header.stamp = ros::Time::now();
        // accelPub.publish(control_output);
        // ROS_INFO_STREAM("RCOUT:" << servo_output[0] << servo_output[1] << servo_output[2] << servo_output[3]);
    }
    
    ros::spinOnce();
    loop_rate.sleep();
}