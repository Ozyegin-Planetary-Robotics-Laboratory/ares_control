/**
 * @file path_controller.cpp
 * @author Toprak Efe Akkılıç (efe.akkilic@ozu.edu.tr)
 * @brief 
 * @version 0.1
 * @date 2024-05-20
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <ros/ros.h>
#include <ares_control/PIDController.hpp>
#include <ares_control/FollowPathAction.h>
#include <ares_control/PathControllerConfig.h>
#include <actionlib/server/simple_action_server.h>
#include <dynamic_reconfigure/server.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>

class PathControllerNode {
public:
  
  PathControllerNode() :
    m_nh("~"),
    m_server("follow_path", boost::bind(&PathControllerNode::followPathActionCallback, this, _1), false),
    m_pid_controller(0.0)
  {
    m_dyn_reconf_callback = boost::bind(&PathControllerNode::reconfigureCallback, this, _1, _2);
    m_dyn_reconf_server.setCallback(m_dyn_reconf_callback);
    m_server.start();
  }

private:
  double m_pid_p, m_pid_i, m_pid_d;
  ros::NodeHandle m_nh;
  actionlib::SimpleActionServer<ares_control::FollowPathAction> m_server;
  dynamic_reconfigure::Server<ares_control::PathControllerConfig> m_dyn_reconf_server;
  dynamic_reconfigure::Server<ares_control::PathControllerConfig>::CallbackType m_dyn_reconf_callback;
  PIDController m_pid_controller;

  void followPathActionCallback(const ares_control::FollowPathGoalConstPtr &goal) {
    // Initialize PID controller

    // PID Loop

    // End of function
  }

  void reconfigureCallback(ares_control::PathControllerConfig &config, uint32_t level) {
    m_pid_p = config.controller_param_p;
    m_pid_i = config.controller_param_i;
    m_pid_d = config.controller_param_d;
  }


};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "path_controller");
  PathControllerNode node;
}