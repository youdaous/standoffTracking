#include <ros/ros.h>
#include <mavros_msgs/PositionTarget.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "attitude_velocity_control");
    ros::NodeHandle nh;

    // Publisher to send commands
    ros::Publisher setpoint_pub = nh.advertise<mavros_msgs::PositionTarget>("rover1/mavros/setpoint_raw/local", 10);

    ros::Rate rate(20); // 20 Hz update rate

    while (ros::ok()) {
        mavros_msgs::PositionTarget setpoint;
        setpoint.header.stamp = ros::Time::now();
        setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;

        // Control mask (indicating which fields are valid)
        setpoint.type_mask =
            mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY | mavros_msgs::PositionTarget::IGNORE_PZ | // Ignore position
            mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ | // Ignore acceleration
            mavros_msgs::PositionTarget::IGNORE_YAW_RATE; // Ignore yaw rate

        // Desired linear velocities (Body NED frame)
        setpoint.velocity.x = 1.0; // Forward velocity
        setpoint.velocity.y = 0.0; // No lateral movement
        setpoint.velocity.z = 0.0; // Maintain altitude

        // Desired yaw angle (in radians)
        setpoint.yaw = 1.57; // 90 degrees yaw

        setpoint_pub.publish(setpoint);

        rate.sleep();
    }

    return 0;
}
