#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

ros::Publisher pub;
float linear_scale = 1.0;
float angular_scale = 1.0;

void clamp(double& val, double min, double max) {
  if (val < min) val = min;
  if (val > max) val = max;
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
  geometry_msgs::Twist twist;
  twist.linear.x = -joy->axes[0]*linear_scale;
  twist.angular.z = -joy->axes[1]*angular_scale;
  pub.publish(twist);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "joystick_interface");
  ros::NodeHandle nh;
  ros::Subscriber sub;
  nh.getParam("joystick_control/linear_scale", linear_scale);
  nh.getParam("joystick_control/angular_scale", angular_scale);
  pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  sub = nh.subscribe("joy", 1, joyCallback);
  ros::spin();
}