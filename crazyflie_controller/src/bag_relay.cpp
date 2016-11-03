#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
//#include "LinearMath/btMatrix3x3.h"

//#include <crazyflie_teleop/PoseStampedWithTime.h> 
//NTS: added

#include "pid.hpp"

class Controller
{
public:

    Controller()
        : m_pubNav()
        , m_thrust(0)
    {
        ros::NodeHandle nh;
        m_pubNav = nh.advertise<geometry_msgs::Twist>("/cap/cmd_vel", 100);
    }

    void run(double frequency){
        ros::NodeHandle node;
        ros::Timer timer = node.createTimer(ros::Duration(1.0/40.0), &Controller::publish_command, this);
        ros::spin();
    }

private:
    void publish_command(const ros::TimerEvent& e)
    {
        if(m_thrust < 49000){
            m_thrust += 1000;
        }
        geometry_msgs::Twist vel_command;
        vel_command.linear.z = m_thrust;
        m_pubNav.publish(vel_command);
    }



private:
    ros::Publisher m_pubNav;
    float m_thrust;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bag_relay");

  // Read parameters
  Controller controller;

  ros::Duration(5.0).sleep();

  controller.run(80.0);

  return 0;
}
