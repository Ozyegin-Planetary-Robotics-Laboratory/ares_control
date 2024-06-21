#ifndef PATH_CONTROLLER_NODELET_HPP
#define PATH_CONTROLLER_NODELET_HPP

#include <mutex>
#include <thread>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <dynamic_reconfigure/server.h>
#include <actionlib/server/simple_action_server.h>
#include <ares_control/FollowPathAction.h>
#include <ares_control/PathControllerConfig.h>
#include <ares_control/PIDController.hpp>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>

namespace ares_control
{
  class PathControllerNodelet : public nodelet::Nodelet
  {
  public:
  void onInit()
  {
    ros::NodeHandle &nh = getNodeHandle();
    ros::NodeHandle &pnh = getPrivateNodeHandle();

    pnh.getParam("controller_param_p", m_p);
    pnh.getParam("controller_param_i", m_i);
    pnh.getParam("controller_param_d", m_d);
    ros::param::get("loop_rate", m_frequency);

    m_pid_controller = PIDController(1.0/m_frequency);
    m_reconf_callback = boost::bind(&PathControllerNodelet::reconfCallback, this, _1, _2);
    m_reconf_server.setCallback(m_reconf_callback);
    m_action_server_ptr = std::make_shared <actionlib::SimpleActionServer<ares_control::FollowPathAction>>(nh, "follow_path", boost::bind(&PathControllerNodelet::actionCallback, this, _1), false);
    m_tf_listener_ptr = std::make_shared<tf2_ros::TransformListener>(m_tf_buffer);
    m_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    m_control_thread = std::thread(&PathControllerNodelet::controlLoop, this);
    m_control_thread.detach();
  }

  private:
  float m_p, m_i, m_d;
  double m_frequency;
  nav_msgs::Path m_path_setpoint;
  geometry_msgs::PoseStamped m_rover_pose;
  PIDController m_pid_controller;
  ros::Publisher m_pub;
  dynamic_reconfigure::Server<ares_control::PathControllerConfig> m_reconf_server;
  dynamic_reconfigure::Server<ares_control::PathControllerConfig>::CallbackType m_reconf_callback;
  std::shared_ptr<actionlib::SimpleActionServer<ares_control::FollowPathAction>> m_action_server_ptr;
  tf2_ros::Buffer m_tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> m_tf_listener_ptr;
  std::thread m_control_thread;
  std::mutex m_path_mutex;
  std::mutex m_rover_pose_mutex;
  std::mutex m_pid_params_mutex;

  void reconfCallback(ares_control::PathControllerConfig &config, uint32_t level)
  {
    std::lock_guard<std::mutex> lock(m_pid_params_mutex);
    m_p = config.controller_param_p;
    m_i = config.controller_param_i;
    m_d = config.controller_param_d;
  }

  void actionCallback(const ares_control::FollowPathGoalConstPtr &goal)
  {
    std::lock_guard<std::mutex> lock(m_path_mutex);
    m_path_setpoint = goal->target_path;
    m_pid_controller.reset();
  }

  void controlLoop()
  { 
    ros::Rate loop_rate(m_frequency);
    while (!ros::isShuttingDown())
    {
      loop_rate.sleep();
      
      if (!m_action_server_ptr->isActive())
      {
        continue;
      }
      
      /* Get the current state. */
      nav_msgs::Path path_setpoint;
      geometry_msgs::PoseStamped rover_pose;
      {
        std::lock_guard<std::mutex> lock(m_path_mutex);
        path_setpoint = m_path_setpoint;
      }
      {
        std::lock_guard<std::mutex> lock(m_rover_pose_mutex);
        rover_pose = m_rover_pose;
      }

      /* Check if the path has been fully followed. */
      geometry_msgs::PoseStamped pose_reference(path_setpoint.poses.back());
      pose_reference.pose.position.x -= rover_pose.pose.position.x;
      pose_reference.pose.position.y -= rover_pose.pose.position.y;
      if (std::pow(pose_reference.pose.position.x, 2) + std::pow(pose_reference.pose.position.y, 2) < 2.0f)
      {
        ares_control::FollowPathResult result;
        result.is_traversed = true;
        m_action_server_ptr->setSucceeded(result);
        std::lock_guard<std::mutex> lock(m_path_mutex);
        m_pid_controller.reset();
        path_setpoint.poses.clear();
        continue;
      }

      /* Get the reference position. */
      geometry_msgs::PoseStamped closest_pose = path_setpoint.poses.front();
      double min_distance2 = std::pow(rover_pose.pose.position.x - closest_pose.pose.position.x, 2) + std::pow(rover_pose.pose.position.y - closest_pose.pose.position.y, 2);
      for (std::vector<geometry_msgs::PoseStamped>::iterator it = path_setpoint.poses.begin(); it != path_setpoint.poses.end(); it++)
      {
        double distance2 = std::pow(rover_pose.pose.position.x - it->pose.position.x, 2) + std::pow(rover_pose.pose.position.y - it->pose.position.y, 2);
        if (distance2 < min_distance2)
        {
          min_distance2 = distance2;
          closest_pose = *it;
        }
      }
      
      /* Compute the control signal. */
      tf2::Vector3 line(1.0f, 0.0f, 0.0f);
      tf2::Vector3 error_vector(rover_pose.pose.position.x - closest_pose.pose.position.x, rover_pose.pose.position.y - closest_pose.pose.position.y, 0.0);
      tf2::Quaternion q;
      tf2::fromMsg(closest_pose.pose.orientation, q);
      line = tf2::quatRotate(q, line);
      tf2::Vector3 cross = line.cross(error_vector);
      float error = cross.getZ();
      float output = m_pid_controller.update(0.0f, error, m_p, m_i, m_d);
      
      /* Publish the control signal. */
      geometry_msgs::TwistPtr twist = boost::make_shared<geometry_msgs::Twist>();
      twist->linear.x = 1.0f;
      twist->angular.z = atan(output);
      m_pub.publish(twist);
    } 
  }

  }; // class PathControllerNodelet
} // namespace ares_control

#endif // PATH_CONTROLLER_NODELET_HPP