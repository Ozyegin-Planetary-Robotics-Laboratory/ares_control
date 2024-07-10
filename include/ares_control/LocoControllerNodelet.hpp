#ifndef LOCO_CONTROLLER_NODELET_HPP
#define LOCO_CONTROLLER_NODELET_HPP

#include <cmath>
#include <mutex>
#include <tmotor.hpp>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <ares_control/WheelCommandArray.h>
#include <tmotor/MotorFeedback.h>
#include <geometry_msgs/Twist.h>

#define WHEEL_RADIUS 0.1425f // cm
#define ROBOT_WIDTH 0.75f // cm
#define RAD_TO_DEG 57.2957795131f

namespace ares_control
{
  class LocoControllerNodelet : public nodelet::Nodelet
  {
    public:
    LocoControllerNodelet() :
      m_wheel_commands{0.0, 0.0, 0.0, 0.0}
    {}

    virtual void onInit()
    {
            
      ros::NodeHandle &nh = getMTNodeHandle();

      std::string control_method;
      std::string can_interface;
      std::vector<int> wheel_ids;

      NODELET_INFO("Initializing parameters from server.");

      ros::param::get("locomotion/wheel_ids", wheel_ids);
      ros::param::get("locomotion/control_method", control_method);
      ros::param::get("general/loop_rate", m_control_freq);
      ros::param::get("general/can_interface", can_interface);

      NODELET_INFO("Locomotion: Control method: %s", control_method.c_str());
      NODELET_INFO("Locomotion: CAN interface: %s", can_interface.c_str());
      NODELET_INFO("Locomotion: Wheel IDs: %d, %d, %d, %d", wheel_ids[0], wheel_ids[1], wheel_ids[2], wheel_ids[3]);
      NODELET_INFO("Locomotion: Control loop rate: %f", m_control_freq);
      
      /* Connect to motors and advertise feedback. */
      for (size_t i = 0; i < 4; i++)
      {
        m_motor_array[i].setMotorID(wheel_ids[i]);
        m_motor_array[i].connect(can_interface.c_str());
      }
      
      m_wheels_pub[0] = nh.advertise <tmotor::MotorFeedback> ("front_right/feedback", 1, false); 
      m_wheels_pub[1] = nh.advertise <tmotor::MotorFeedback> ("front_left/feedback", 1, false); 
      m_wheels_pub[2] = nh.advertise <tmotor::MotorFeedback> ("rear_right/feedback", 1, false); 
      m_wheels_pub[3] = nh.advertise <tmotor::MotorFeedback> ("rear_left/feedback", 1, false); 
            
      /* Subscribe to command topics and initialize the control loop. */
      if (control_method == "velocity")
      {
        NODELET_INFO("Enacting VELOCITY control.");
        m_twist_sub = nh.subscribe <geometry_msgs::Twist> ("cmd_vel", 1, &LocoControllerNodelet::robotTwistVelocityCallback, this); 
        m_wheels_cmd_sub = nh.subscribe <ares_control::WheelCommandArray> ("cmd_arr", 1, &LocoControllerNodelet::wheelArrayVelocityCallback, this); 
        m_control_thread = std::thread(&LocoControllerNodelet::velocityControlLoop, this);
      }
      else // or "torque"
      {
        NODELET_INFO("Enacting TORQUE control.");
        m_twist_sub = nh.subscribe <geometry_msgs::Twist> ("cmd_vel", 1, &LocoControllerNodelet::robotTwistTorqueCallback, this); 
        m_wheels_cmd_sub = nh.subscribe <ares_control::WheelCommandArray> ("cmd_arr", 1, &LocoControllerNodelet::wheelArrayTorqueCallback, this); 
        m_control_thread = std::thread(&LocoControllerNodelet::torqueControlLoop, this);
      }
      m_control_thread.detach();
    }

    private:    
    ros::Subscriber m_twist_sub, m_wheels_cmd_sub; 
    ros::Publisher  m_wheels_pub[4];
    TMotor::AKManager m_motor_array[4];
    float m_wheel_commands[4];
    double m_control_freq;
    std::mutex m_control_mutex;
    std::thread m_control_thread;

    void velocityControlLoop()
    {
      ros::Rate loop_frequency(m_control_freq);
      while (!ros::isShuttingDown())
      {
        loop_frequency.sleep();
        float cmds[4];
        tmotor::MotorFeedbackPtr feedbacks[4] = {
          tmotor::MotorFeedbackPtr(new tmotor::MotorFeedback),
          tmotor::MotorFeedbackPtr(new tmotor::MotorFeedback),
          tmotor::MotorFeedbackPtr(new tmotor::MotorFeedback),
          tmotor::MotorFeedbackPtr(new tmotor::MotorFeedback)
        };
        for (size_t i = 0; i < 4; i++)
        {
          feedbacks[i]->position    = m_motor_array[i].getPosition();
          feedbacks[i]->velocity    = m_motor_array[i].getVelocity();
          feedbacks[i]->current     = m_motor_array[i].getCurrent();
          feedbacks[i]->temperature = m_motor_array[i].getTemperature();
          feedbacks[i]->motor_fault = m_motor_array[i].getFault();
          std::lock_guard <std::mutex> guard(m_control_mutex);
          cmds[i] = m_wheel_commands[i];
          m_wheel_commands[i] *= 0.95;
          m_motor_array[i].sendVelocity(cmds[i]);
        }
        m_wheels_pub[0].publish(feedbacks[0]); 
        m_wheels_pub[1].publish(feedbacks[1]); 
        m_wheels_pub[2].publish(feedbacks[2]); 
        m_wheels_pub[3].publish(feedbacks[3]); 
      }
    }

    void torqueControlLoop()
    {
      ros::Rate loop_frequency(m_control_freq);
      while (!ros::isShuttingDown())
      {
        loop_frequency.sleep();
        float cmds[4];
        tmotor::MotorFeedbackPtr feedbacks[4] = {
          tmotor::MotorFeedbackPtr(new tmotor::MotorFeedback),
          tmotor::MotorFeedbackPtr(new tmotor::MotorFeedback),
          tmotor::MotorFeedbackPtr(new tmotor::MotorFeedback),
          tmotor::MotorFeedbackPtr(new tmotor::MotorFeedback)
        };
        for (size_t i = 0; i < 4; i++)
        {
          feedbacks[i]->position    = m_motor_array[i].getPosition();
          feedbacks[i]->velocity    = m_motor_array[i].getVelocity();
          feedbacks[i]->current     = m_motor_array[i].getCurrent();
          feedbacks[i]->temperature = m_motor_array[i].getTemperature();
          feedbacks[i]->motor_fault = m_motor_array[i].getFault();
          std::lock_guard <std::mutex> guard(m_control_mutex);
          cmds[i] = m_wheel_commands[i];
          m_wheel_commands[i] *= 0.95;
          m_motor_array[i].sendCurrent(cmds[i]);
        }
        m_wheels_pub[0].publish(feedbacks[0]); 
        m_wheels_pub[1].publish(feedbacks[1]); 
        m_wheels_pub[2].publish(feedbacks[2]); 
        m_wheels_pub[3].publish(feedbacks[3]); 
      }
    }

    void robotTwistVelocityCallback(const geometry_msgs::TwistConstPtr &msg)
    {
      float linear(msg->linear.x), angular(msg->angular.z);
      if (linear > 0.0f) {angular *= -1.0f;}
      float v_l = (linear - angular * ROBOT_WIDTH) * WHEEL_RADIUS;
      float v_r = (linear + angular * ROBOT_WIDTH) * WHEEL_RADIUS;
      std::lock_guard <std::mutex> guard(m_control_mutex);
      m_wheel_commands[0] = m_wheel_commands[2] = -v_r*RAD_TO_DEG; // right wheel axes are mirrored       
      m_wheel_commands[1] = m_wheel_commands[3] = v_l*RAD_TO_DEG;       
    }

    void wheelArrayVelocityCallback(const ares_control::WheelCommandArrayConstPtr &msg)
    { 
      if (msg->data.size() != 4) return;  
      for (auto command : msg->data) if (command.type != 2) return;
      std::lock_guard <std::mutex> guard(m_control_mutex);
      m_wheel_commands[0] = msg->data[0].velocity;
      m_wheel_commands[1] = msg->data[1].velocity;
      m_wheel_commands[2] = msg->data[2].velocity;
      m_wheel_commands[3] = msg->data[3].velocity;
    }

    void robotTwistTorqueCallback(const geometry_msgs::TwistConstPtr &msg)
    {
      // TBD
    }

    void wheelArrayTorqueCallback(const ares_control::WheelCommandArrayConstPtr &msg)
    { 
      if (msg->data.size() != 4) return;  
      for (auto command : msg->data) if (command.type != 3) return;
      m_wheel_commands[0] = msg->data[0].current;
      m_wheel_commands[1] = msg->data[1].current;
      m_wheel_commands[2] = msg->data[2].current;
      m_wheel_commands[3] = msg->data[3].current;
    }

  }; // class LocoControllerNodelet
}; // namespace ares_control

#endif // LOCO_CONTROLLER_NODELET_HPP