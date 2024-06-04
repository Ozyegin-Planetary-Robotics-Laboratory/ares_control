/**
 * @file loco_interface.cpp
 * @author Toprak Efe Akkılıç (efe.akkilic@ozu.edu.tr)
 * @brief 
 * @version 0.1
 * @date 2024-05-20
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <mutex>
#include <ros/ros.h>
#include <tmotor.hpp>
#include <tmotor/MotorCommand.h>
#include <tmotor/MotorFeedback.h>
#include <geometry_msgs/Twist.h>

/* I am doing it with macros, because in all due honesty,
  our mechanical engineers are too dumb to know robot kinematics
  so I am not supplied with any knowledge beforehand as to the
  physical model of our robot and the simplest of things like
  moment of inertia. No URDF for the rover itself, I don't care
  if our model isn't good, the closed-loop control IS good enough. */
#define WHEEL_RADIUS 0.1425f // cm
#define ROBOT_WIDTH 0.75f // cm

class LocoControllerNode {
public:
  LocoControllerNode() {
    m_active_twist.linear.x = 0.0f;
    m_active_twist.linear.y = 0.0f;
    m_active_twist.linear.z = 0.0f;
    m_active_twist.angular.z = 0.0f;
    m_active_twist.angular.y = 0.0f;
    m_active_twist.angular.z = 0.0f;
    m_pub_fl = m_nh.advertise<tmotor::MotorCommand>("front_right/motor_command", 1);
    m_pub_fr = m_nh.advertise<tmotor::MotorCommand>("front_left/motor_command", 1);
    m_pub_rl = m_nh.advertise<tmotor::MotorCommand>("rear_right/motor_command", 1);
    m_pub_rr = m_nh.advertise<tmotor::MotorCommand>("rear_left/motor_command", 1);
    std::thread control_thread([this]{
      ros::Rate freq(20); // 20hz
      while (ros::ok()) {
      float linear, angular;
      {
        std::lock_guard<std::mutex> lock(m_mutex);
        linear = m_active_twist.linear.x;
        angular = m_active_twist.angular.z;
        m_active_twist.linear.x *= 0.95f;
      }
      if (linear == 0.0f) continue;
      float v_l = (std::abs(linear) - angular * ROBOT_WIDTH)/WHEEL_RADIUS;
      float v_r = (std::abs(linear) + angular * ROBOT_WIDTH)/WHEEL_RADIUS;
      const bool forward = linear > 0.0f;
      v_l = forward ? v_l : -v_l;
      v_r = forward ? v_r : -v_r;
      m_active_cmd[0].velocity = v_r;
      m_active_cmd[1].velocity = v_l;
      m_active_cmd[3].velocity = v_r;
      m_active_cmd[4].velocity = v_l;
      m_pub_fr.publish(m_active_cmd[0]);
      m_pub_fl.publish(m_active_cmd[1]);
      m_pub_rr.publish(m_active_cmd[2]);
      m_pub_rl.publish(m_active_cmd[3]);
      freq.sleep();
    }});
    control_thread.detach();
  }

  void run() {
    ros::spin();
  }

private:
  std::map<std::string, TMotor::AKManager> m_motors;
  ros::NodeHandle m_nh;
  ros::Publisher m_pub_fr;
  ros::Publisher m_pub_fl;
  ros::Publisher m_pub_rr;
  ros::Publisher m_pub_rl;
  std::mutex m_mutex;
  geometry_msgs::Twist m_active_twist;
  tmotor::MotorCommand m_active_cmd[4];

  void controlLoop() {
    
  }

  void cmdVelCallback(const geometry_msgs::TwistConstPtr& msg) {
    m_active_twist.linear.x = msg->linear.x;
    m_active_twist.angular.z = msg->angular.z;
  }
};

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "loco_controller");
  LocoControllerNode node;
  node.run();
}