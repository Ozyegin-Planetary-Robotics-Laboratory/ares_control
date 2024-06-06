/**
 * @file wheel_controller.cpp
 * @author Toprak Efe Akkılıç (efe.akkilic@ozu.edu.tr)
 * @brief 
 * @version 0.1
 * @date 2024-06-04
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <mutex>
#include <thread>
#include <ros/ros.h>
#include <functional>
#include <signal.h>
#include <tmotor.hpp>
#include <tmotor/MotorCommand.h>
#include <tmotor/MotorFeedback.h>

sig_atomic_t volatile request_shutdown = 0;

class MotorNode
{
public:
  MotorNode(int id) :
    n_nh("~"),
    motor_manager(id),
    vel_cmd(5.0),
    pose_cur(0.0), vel_cur(0.0), eff_cur(0.0),
    mode_id(TMotor::MotorModeID::VELOCITY)
  {
    std::string can_interface;
    if (!ros::param::get("can_interface", can_interface)) {
      ROS_ERROR("CAN interface not provided, exiting...");
      ros::shutdown();
    }
    ros::param::get("invert", m_invert);
    ROS_INFO("Connecting to CAN interface %s", can_interface.c_str());
    motor_manager.connect(can_interface.c_str());
    n_nh = ros::NodeHandle("~");
    n_sub = n_nh.subscribe("motor_command", 1, &MotorNode::motorCommandCallback, this);
    n_pub = n_nh.advertise<tmotor::MotorFeedback>("motor_feedback", 1);
  } 

  void run() {
    motor_updater = std::thread([this] {
      ros::Rate loop_rate(40); // 40hz
      
      while (!request_shutdown && ros::ok()) {
        tmotor::MotorFeedback msg;
        setMotorStates(motor_manager.getPosition(), motor_manager.getVelocity(), motor_manager.getCurrent());
        msg.position = getPoseState();
        msg.velocity = getVelocityState();
        msg.current = getEffortState();
        msg.temperature = motor_manager.getTemperature();
        msg.motor_fault = motor_manager.getFault();
        if (msg.motor_fault != TMotor::MotorFault::NONE) {
          std::string warning_msg = "Motor fault detected for ID " + std::to_string(motor_manager.getMotorID()) + " with fault: " + fault_to_str(motor_manager.getFault());
          ROS_WARN("%s", warning_msg.c_str());
        }
        n_pub.publish(msg);
        float cmd = getVelocityCommand() * (m_invert ? -1.0 : 1.0);
        motor_manager.sendVelocity(cmd);
        loop_rate.sleep();
      }
    });

    motor_updater.detach();
    ros::spin();
  }

private:
  float vel_cmd;
  float pose_cur, vel_cur, eff_cur;
  std::mutex motor_mutex;
  std::thread motor_updater;
  TMotor::AKManager motor_manager;
  ros::NodeHandle n_nh;
  ros::Subscriber n_sub;
  ros::Publisher n_pub;
  TMotor::MotorModeID mode_id;
  bool m_invert;

  float getVelocityCommand() {
    std::lock_guard<std::mutex> lock(motor_mutex);
    return vel_cmd;
  }

  void setVelocityCommand(float vel) {
    std::lock_guard<std::mutex> lock(motor_mutex);
    vel_cmd = vel;
  }

  float getPoseState() {
    std::lock_guard<std::mutex> lock(motor_mutex);
    return pose_cur;
  }

  float getVelocityState() {
    std::lock_guard<std::mutex> lock(motor_mutex);
    return vel_cur;
  }

  float getEffortState() {
    std::lock_guard<std::mutex> lock(motor_mutex);
    return eff_cur;
  }

  void setMotorStates(float pose, float vel, float eff) {
    std::lock_guard<std::mutex> lock(motor_mutex);
    pose_cur = pose;
    vel_cur = vel;
    eff_cur = eff;
  }

  void motorCommandCallback(const tmotor::MotorCommand::ConstPtr& msg) {
    static const double vel_max = 22.5;
    static const double vel_min = -22.5;

    std::lock_guard<std::mutex> lock(motor_mutex);
    mode_id = static_cast<TMotor::MotorModeID>(msg->type);
    vel_cmd = msg->velocity < vel_max ? msg->velocity : vel_max;
    vel_cmd = msg->velocity > vel_min ? msg->velocity : vel_min;
  }

  std::string fault_to_str(TMotor::MotorFault fault) {
    static std::map<TMotor::MotorFault, std::string> dict({
      {TMotor::MotorFault::NONE,            "NONE"           },
      {TMotor::MotorFault::OVERTEMPERATURE, "OVERTEMPERATURE"},
      {TMotor::MotorFault::OVERCURRENT,     "OVERCURRENT"    },
      {TMotor::MotorFault::OVERVOLTAGE,     "OVERVOLTAGE"    },
      {TMotor::MotorFault::UNDERVOLTAGE,    "UNDERVOLTAGE"   },
      {TMotor::MotorFault::ENCODER,         "ENCODER"        },
      {TMotor::MotorFault::HARDWARE,        "HARDWARE"       }
    });
    return dict[fault];
  }

};

int main(int argc, char** argv) {
  ros::init(argc, argv, "tmotor_node");
  int id;
  ros::param::get("~id", id);
  ROS_INFO("AK motor node started with id %d", id);
  signal(SIGINT, [](int sig) { request_shutdown = 1; });
  MotorNode node(id);
  node.run();
  return 0;
}