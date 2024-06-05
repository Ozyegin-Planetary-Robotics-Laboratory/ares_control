#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

ros::Publisher pub;

void clamp(double& val, double min, double max) {
  if (val < min) val = min;
  if (val > max) val = max;
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
  geometry_msgs::Twist twist;
  twist.linear.x = joy->axes[1];
  twist.angular.z = joy->axes[0];
  clamp(twist.linear.x, -1.0, 1.0);
  clamp(twist.angular.z, -1.0, 1.0);
  pub.publish(twist);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "joystick_interface");
  ros::NodeHandle nh;
  ros::Subscriber sub;
  pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  sub = nh.subscribe("joy", 1, joyCallback);
  ros::spin();
}