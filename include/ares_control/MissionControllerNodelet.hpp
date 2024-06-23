#ifndef MISSION_CONTROLLER_NODELET_HPP
#define MISSION_CONTROLLER_NODELET_HPP

#include <ros/ros.h>
#include <nodelet/nodelet.h>

namespace ares_control
{
  class MissionControllerNodelet : public nodelet::Nodelet 
  {
    public:
    virtual void onInit()
    {
      NODELET_INFO("Initializing Mission Controller Nodelet");
    }
    private:
  }; // class MissionControllerNodelet
} // namespace ares_control


#endif // MISSION_CONTROLLER_NODELET_HPP