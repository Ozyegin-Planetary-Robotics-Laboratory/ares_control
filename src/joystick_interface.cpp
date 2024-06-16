#include <mutex>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

namespace ares_control
{
  class JoyStickInterfaceNode : public nodelet::Nodelet
  {
    public:
    JoyStickInterfaceNode() :
      m_linear_scale(0.0),
      m_angular_scale(0.0)    
    {}

    virtual void onInit()
    {
      ros::NodeHandle &nh = getMTNodeHandle();
      ros::NodeHandle &pnh = getMTPrivateNodeHandle();
      pnh.getParam("linear_scale", m_linear_scale);
      pnh.getParam("angular_scale", m_angular_scale);
      ros::param::get("loop_rate", m_loop_rate);
      m_sub = nh.subscribe<sensor_msgs::Joy>("joy", 1, &JoyStickInterfaceNode::joyCallback, this);
      m_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);    
      m_control_thread = std::thread(&JoyStickInterfaceNode::controlLoop, this); 
      m_control_thread.detach();
    }

    private:
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
          twist->linear.x = m_twist.linear.x;
          twist->angular.z = m_twist.angular.z;
        }
        m_pub.publish(twist);
        loop_rate.sleep();
      }
    }

  }; // JoyStickInterfaceNode
}; // namespace ares_control

PLUGINLIB_EXPORT_CLASS(ares_control::JoyStickInterfaceNode, nodelet::Nodelet);