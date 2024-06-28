#ifndef MISSION_CONTROLLER_NODELET_HPP
#define MISSION_CONTROLLER_NODELET_HPP

#include <mutex>
#include <queue>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <actionlib/client/simple_action_client.h>
#include <ares_control/GetPath.h>
#include <ares_control/CheckPath.h>
#include <ares_control/GoalSite.h>
#include <ares_control/FollowPathAction.h>
#include <ares_control/MissionControl/LEDController.hpp>
#include <geometry_msgs/PoseStamped.h>

namespace ares_control
{
  class MissionControllerNodelet : public nodelet::Nodelet 
  {
    public:
    virtual void onInit()
    {
      ros::NodeHandle &nh = getNodeHandle();
      m_goal_sub = nh.subscribe<ares_control::GoalSite>("goal_site", 20, &MissionControllerNodelet::goalCallback, this);
      m_pathtrace_action_client_ptr = std::make_shared<actionlib::SimpleActionClient<ares_control::FollowPathAction>>("follow_path", true);
      m_pathfind_service_client = nh.serviceClient<ares_control::GetPath>("get_path");
      m_pathcheck_service_client = nh.serviceClient<ares_control::CheckPath>("check_path");
      m_led_controller.setColor(LedController::BLUE);
    }

    LedController m_led_controller;
    ros::Subscriber m_goal_sub;
    ros::ServiceClient m_pathfind_service_client;
    ros::ServiceClient m_pathcheck_service_client;
    ros::Timer m_nav_check_timer;
    std::shared_ptr <actionlib::SimpleActionClient<ares_control::FollowPathAction>> m_pathtrace_action_client_ptr;
    std::map<ares_control::GoalSite, geometry_msgs::PoseStamped> m_marker_poses_dict;

    void goalCallback(const ares_control::GoalSiteConstPtr &msg)
    {
      if (isInvalidMessage(*msg)) return;
      sendFollowPathAction(msg->marker_poses[0]);
    }

    void goalCompleteCallback(const actionlib::SimpleClientGoalState &state, const ares_control::FollowPathResultConstPtr &result)
    {
      if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        m_led_controller.setFlashing(LedController::GREEN);
      }
      else
      {
        m_led_controller.setFlashing(LedController::RED);
      }
    }

    void goalActiveCallback()
    {
      m_led_controller.setColor(LedController::YELLOW);
    }

    void feedbackCallback(const ares_control::FollowPathFeedbackConstPtr &feedback)
    {
      ares_control::CheckPath check_path_service_msg;
      check_path_service_msg.request.path = feedback->remaining_path;
      m_pathcheck_service_client.call(check_path_service_msg);
      if (check_path_service_msg.response.is_collision_course)
      {
        m_pathtrace_action_client_ptr->cancelGoal();
        sendFollowPathAction(feedback->remaining_path.poses.back());
      }
    }

    bool isInvalidMessage(const ares_control::GoalSite &msg)
    {
      return msg.marker_ids.empty() ||
             msg.marker_poses.empty() ||
             msg.marker_count == 0 ||
             msg.marker_count != msg.marker_ids.size() ||
             msg.marker_count != msg.marker_poses.size();
    }

    void sendFollowPathAction(const geometry_msgs::PoseStamped &goal)
    {
      ares_control::FollowPathGoalPtr new_goal(new ares_control::FollowPathGoal);
      new_goal->target_path = getPath(goal);
      m_pathtrace_action_client_ptr->sendGoal(*new_goal, boost::bind(&MissionControllerNodelet::goalCompleteCallback, this, _1, _2), boost::bind(&MissionControllerNodelet::goalActiveCallback, this), boost::bind(&MissionControllerNodelet::feedbackCallback, this, _1));
    }

    nav_msgs::Path getPath(const geometry_msgs::PoseStamped &pose)
    {
      ares_control::GetPath path_service;
      path_service.request.goal = pose;
      ros::Duration loop_period(2.0);
      while (!m_pathfind_service_client.call(path_service)) loop_period.sleep();
      return path_service.response.path;
    }

  }; // class MissionControllerNodelet
} // namespace ares_control

#endif // MISSION_CONTROLLER_NODELET_HPP