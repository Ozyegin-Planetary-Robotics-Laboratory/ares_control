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

#include <ros/ros.h>
#include <tmotor.hpp>
#include <geometry_msgs/Twist.h>

/* I am doing it with macros, because in all due honesty,
  our mechanical engineers are too dumb to know robot kinematics
  so I am not supplied with any knowledge beforehand as to the
  physical model of our robot and the simplest of things like
  moment of inertia. No URDF for the rover itself, I don't care
  if our model isn't good, the closed-loop control IS good enough. */
#define WHEEL_RADIUS 0.1425 // cm

class LocoControllerNode {
public:
  LocoControllerNode() :
    m_nh("~"),
    m_motors({
      {"front_left", TMotor::AKManager(1)},
      {"front_right", TMotor::AKManager(2)},
      {"rear_left", TMotor::AKManager(3)},
      {"rear_right", TMotor::AKManager(4)}
    })
  {
    m_sub = m_nh.subscribe("cmd_vel", 1, &LocoControllerNode::cmdVelCallback, this);
  }

  void init() {
    const std::string can_interface = m_nh.param<std::string>("can_interface", "vcan0");
    for (auto& motor : m_motors) {
      motor.second.connect(can_interface.c_str());
    }
  }

private:
  std::map<std::string, TMotor::AKManager> m_motors;
  ros::NodeHandle m_nh;
  ros::Subscriber m_sub;

  void cmdVelCallback(const geometry_msgs::TwistConstPtr& msg) {
    float linear = msg->linear.x;
    float angular = msg->angular.z;
    
  }
};

int main(int argc, char* argv[]) {
  //LocoControllerNode node;
  ros::init(argc, argv, "loco_controller");
  //node.init();
  ros::spin();
}