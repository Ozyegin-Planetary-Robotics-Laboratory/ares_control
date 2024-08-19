#ifndef LOCO_CONTROLLER_NODELET_HPP
#define LOCO_CONTROLLER_NODELET_HPP

#include <cmath>
#include <mutex>
#include <atomic>
#include <tmotor.hpp>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <ares_control/WheelCommandArray.h>
#include <ares_control/MotorFeedback.h>
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
      m_wheel_commands{0.0, 0.0, 0.0, 0.0},
      m_motor_name_map{"front right", "front left", "rear right", "rear left"},
      m_overheating(false)
    {}

    virtual void onInit()
    {
            
      ros::NodeHandle &nh = getMTNodeHandle();

      std::string control_method;
      std::string can_interface;
      std::vector<int> wheel_ids;

      NODELET_INFO("Initializing parameters from server.");

      int temp_limit(50);
      ros::param::get("locomotion/motor_temp_lim", temp_limit);
      ros::param::get("locomotion/control_method", control_method);
      ros::param::get("locomotion/control_degree", m_control_degree);
      ros::param::get("general/loop_rate", m_control_freq);
      ros::param::get("general/can_interface", can_interface);
      ros::param::get("locomotion/wheel_ids", wheel_ids);
      m_maximum_temperature = std::max(INT8_MIN, std::min(INT8_MAX, temp_limit));
      m_can_interface = can_interface;
      m_motor_ids[0] = wheel_ids[0];
      m_motor_ids[1] = wheel_ids[1];
      m_motor_ids[2] = wheel_ids[2];
      m_motor_ids[3] = wheel_ids[3];

      NODELET_INFO("Locomotion: Control method: %s", control_method.c_str());
      NODELET_INFO("Locomotion: CAN interface: %s", can_interface.c_str());
      NODELET_INFO("Locomotion: Wheel IDs: %d, %d, %d, %d", wheel_ids[0], wheel_ids[1], wheel_ids[2], wheel_ids[3]);
      NODELET_INFO("Locomotion: Control loop rate: %f", m_control_freq);

      /* Connect to motors and advertise feedbacks. */
      connectMotors();
      m_wheels_pub[0] = nh.advertise <ares_control::MotorFeedback> ("front_right/feedback", 1, false); 
      m_wheels_pub[1] = nh.advertise <ares_control::MotorFeedback> ("front_left/feedback", 1, false); 
      m_wheels_pub[2] = nh.advertise <ares_control::MotorFeedback> ("rear_right/feedback", 1, false); 
      m_wheels_pub[3] = nh.advertise <ares_control::MotorFeedback> ("rear_left/feedback", 1, false); 
            
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
    uint8_t m_motor_ids[4];
    std::string m_can_interface;
    float m_wheel_commands[4];
    double m_control_freq;
    int m_control_degree;
    std::mutex m_control_mutex;
    std::thread m_control_thread;
    std::atomic<bool> m_reconnect;
    std::atomic<bool> m_overheating;
    std::vector<std::string> m_motor_name_map;
    int8_t m_maximum_temperature;

    void velocityControlLoop()
    {
      ros::Rate loop_frequency(m_control_freq);
      while (!ros::isShuttingDown())
      {
        loop_frequency.sleep();

        float cmds[4];
        ares_control::MotorFeedbackPtr feedbacks[4] = {
          ares_control::MotorFeedbackPtr(new ares_control::MotorFeedback),
          ares_control::MotorFeedbackPtr(new ares_control::MotorFeedback),
          ares_control::MotorFeedbackPtr(new ares_control::MotorFeedback),
          ares_control::MotorFeedbackPtr(new ares_control::MotorFeedback)
        };
        bool overheating = isOverheating();

        for (size_t i = 0; i < 4; i++)
        {
          feedbacks[i]->position    = m_motor_array[i].getPosition();
          feedbacks[i]->velocity    = m_motor_array[i].getVelocity();
          feedbacks[i]->current     = m_motor_array[i].getCurrent();
          feedbacks[i]->temperature = m_motor_array[i].getTemperature();
          feedbacks[i]->motor_fault = m_motor_array[i].getFault();
          
          m_control_mutex.lock();
          cmds[i] = m_wheel_commands[i];
          m_wheel_commands[i] *= 0.95;
          m_control_mutex.unlock();
          
          if (overheating) continue;

          try
          {
            m_motor_array[i].sendVelocity(cmds[i]);
          }
          catch (TMotor::CANSocketException& e)
          {
            connectMotors();
          }
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
        ares_control::MotorFeedbackPtr feedbacks[4] = {
          ares_control::MotorFeedbackPtr(new ares_control::MotorFeedback),
          ares_control::MotorFeedbackPtr(new ares_control::MotorFeedback),
          ares_control::MotorFeedbackPtr(new ares_control::MotorFeedback),
          ares_control::MotorFeedbackPtr(new ares_control::MotorFeedback)
        };
        bool overheating = isOverheating();

        for (size_t i = 0; i < 4; i++)
        {
          feedbacks[i]->position    = m_motor_array[i].getPosition();
          feedbacks[i]->velocity    = m_motor_array[i].getVelocity();
          feedbacks[i]->current     = m_motor_array[i].getCurrent();
          feedbacks[i]->temperature = m_motor_array[i].getTemperature();
          feedbacks[i]->motor_fault = m_motor_array[i].getFault();
          
          m_control_mutex.lock();
          cmds[i] = m_wheel_commands[i];
          m_wheel_commands[i] *= 0.95;
          m_control_mutex.unlock();
          
          if (overheating) continue;

          try
          {
            m_motor_array[i].sendCurrent(cmds[i]);
          }
          catch (TMotor::CANSocketException& e)
          {
            connectMotors();
          }
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
      std::lock_guard <std::mutex> guard(m_control_mutex);
      float v_l = ((linear - angular * ROBOT_WIDTH) * WHEEL_RADIUS * RAD_TO_DEG - m_wheel_commands[0]*m_control_degree)/(m_control_degree + 1);
      float v_r = ((linear + angular * ROBOT_WIDTH) * WHEEL_RADIUS * RAD_TO_DEG + m_wheel_commands[1]*m_control_degree)/(m_control_degree + 1);      
      m_wheel_commands[0] = m_wheel_commands[2] = -v_r;       
      m_wheel_commands[1] = m_wheel_commands[3] = v_l;       
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

    bool isOverheating()
    {
      for (size_t i = 0; i < 4; i++)
      {
        int8_t temperature = m_motor_array[i].getTemperature();
        if (temperature > m_maximum_temperature || m_motor_array[i].getFault() == TMotor::MotorFault::OVERTEMPERATURE)
        {
          NODELET_WARN("Motor %d at %s is overheating at %i", m_motor_array[i].getMotorID(), m_motor_name_map[i].c_str(), temperature);
          m_overheating = true;
          return true;
        }
      }
      return false;
    }

    void connectMotors()
    {
      NODELET_INFO("Connecting to motors through the CAN interface.");
      for (size_t i = 0; i < 4; i++)
      {
        try
        {
          m_motor_array[i].setMotorID(m_motor_ids[i]);
          m_motor_array[i].connect(m_can_interface.c_str());
        }
        catch(TMotor::CANSocketException& e)
        {
          NODELET_WARN("Motor %d at %s unable to reconnect: %s", m_motor_array[i].getMotorID(), m_motor_name_map[i].c_str(), e.what());
          std::this_thread::sleep_for(std::chrono::milliseconds(500));
          i--; // retry
        }
      }
    }
  }; // class LocoControllerNodelet
}; // namespace ares_control

#endif // LOCO_CONTROLLER_NODELET_HPP