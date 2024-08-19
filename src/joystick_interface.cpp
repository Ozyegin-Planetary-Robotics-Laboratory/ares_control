#include <ares_control/JoystickInterface.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joystick_interface");
  ares_control::JoystickInterface joystick_interface;
  joystick_interface.init();
  ros::spin();
  return 0;
}