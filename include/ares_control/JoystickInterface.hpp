#ifndef JOYSTICK_INTERFACE_HPP
#define JOYSTICK_INTERFACE_HPP

#include <mutex>
#include <thread>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

namespace ares_control
{
  class JoystickInterface
  {
    public:
    JoystickInterface() :
      m_linear_scale(0.0),
      m_angular_scale(0.0)    
    {}

    void init()
    {
      ros::param::get("locomotion/speed/linear_scale", m_linear_scale);
      ros::param::get("locomotion/speed/angular_scale", m_angular_scale);
      ros::param::get("general/loop_rate", m_loop_rate);
      m_sub = m_nh.subscribe<sensor_msgs::Joy>("joy", 1, &JoystickInterface::joyCallback, this);
      m_pub = m_nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);    
      m_control_thread = std::thread(&JoystickInterface::controlLoop, this); 
      m_control_thread.detach();
    }

    private:
    ros::NodeHandle m_nh;
    ros::Subscriber m_sub;
    ros::Publisher m_pub;
    std::thread m_control_thread;
    std::mutex m_command_mutex; 
    double m_linear_scale, m_angular_scale, m_loop_rate;
    geometry_msgs::Twist m_twist;
    
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
    {
      std::lock_guard<std::mutex> lock(m_command_mutex);
      m_twist.linear.x = joy->axes[1]*m_linear_scale;
      m_twist.angular.z = joy->axes[0]*m_angular_scale;
    }

    void controlLoop()
    {
      ros::Rate loop_rate(m_loop_rate/2);
      while (!ros::isShuttingDown())
      {
        geometry_msgs::TwistPtr twist = boost::make_shared<geometry_msgs::Twist>();
        {
          std::lock_guard<std::mutex> lock(m_command_mutex);
          twist->linear.x = (m_twist.linear.x *= 0.9875);
          twist->angular.z = (m_twist.angular.z *= 0.9875);
        }
        m_pub.publish(twist);
        loop_rate.sleep();
      }
    }

  }; // JoystickInterface
}; // namespace ares_control

#endif // JOYSTICK_INTERFACE_HPP