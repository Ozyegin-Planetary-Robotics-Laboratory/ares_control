#ifndef JOYSTICK_INTERFACE_HPP
#define JOYSTICK_INTERFACE_HPP

#include <mutex>
#include <thread>
//#include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
//#include <nodelet/nodelet.h>
//#include <sensor_msgs/Joy.h>
#include <sensor_msgs/msg/joy.hpp>
//#include <geometry_msgs/Twist.h>
#include <geometry_msgs/msg/twist.hpp>

namespace ares_control
{
  //class JoystickInterfaceNodelet : public nodelet::Nodelet
  class JoystickInterface : public rclcpp::Node
  {
    public:
    //JoystickInterfaceNodelet() :
    JoystickInterface() : Node("joystick_interface"),
      m_linear_scale(0.0),
      m_angular_scale(0.0),
      m_loop_rate(30.0)    //?newline
    {
      this->declare_parameter<double>("locomotion/speed/linear_scale", 0.0);
      this->declare_parameter<double>("locomotion/speed/angular_scale", 0.0);
      this->declare_parameter<double>("general/loop_rate", 30.0);

      this->get_parameter("locomotion/speed/linear_scale", m_linear_scale);
      this->get_parameter("locomotion/speed/angular_scale", m_angular_scale);
      this->get_parameter("general/loop_rate", m_loop_rate);


      //m_sub = nh.subscribe<sensor_msgs::Joy>("joy", 1, &JoystickInterfaceNodelet::joyCallback, this);
      //m_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1); 

      m_sub = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&JoystickInterface::joyCallback, this, std::placeholders::_1));
      m_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);


      //m_control_thread = std::thread(&JoystickInterfaceNodelet::controlLoop, this);
      m_control_thread = std::thread(&JoystickInterface::controlLoop, this); 
      m_control_thread.detach();
    }

    private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr m_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_pub;
    std::thread m_control_thread;
    std::mutex m_command_mutex; 
    double m_linear_scale, m_angular_scale, m_loop_rate;
    geometry_msgs::msg::Twist m_twist;
    
    //void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
    void joyCallback(const std::shared_ptr<sensor_msgs::msg::Joy> joy)
    {
      std::lock_guard<std::mutex> lock(m_command_mutex);
      m_twist.linear.x = joy->axes[1]*m_linear_scale;
      m_twist.angular.z = joy->axes[0]*m_angular_scale;
      RCLCPP_INFO(this->get_logger(), "axes[0]: %f, axes[1]: %f", joy->axes[0], joy->axes[1]);
    }

    void controlLoop()
    {
      //ros::Rate loop_rate(m_loop_rate/2);
      rclcpp::Rate loop_rate(m_loop_rate / 2.0);
      //while (!ros::isShuttingDown())
      while (rclcpp::ok())
      {
        //geometry_msgs::TwistPtr twist = boost::make_shared<geometry_msgs::Twist>();
        auto twist = std::make_shared<geometry_msgs::msg::Twist>();
        {
          std::lock_guard<std::mutex> lock(m_command_mutex);
          twist->linear.x = (m_twist.linear.x *= 0.9875);
          twist->angular.z = (m_twist.angular.z *= 0.9875);
        }
        m_pub->publish(*twist);
        loop_rate.sleep();
      }
    }

  }; // JoystickInterface
}; // namespace ares_control

#endif // JOYSTICK_INTERFACE_HPP