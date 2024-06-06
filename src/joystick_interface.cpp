#include <mutex>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

ros::Publisher pub;
double linear_scale, angular_scale;
geometry_msgs::Twist twist;
std::mutex twist_mutex;

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
  geometry_msgs::Twist twist;
  std::lock_guard<std::mutex> lock(twist_mutex);
  twist.linear.x = joy->axes[1]*linear_scale;
  twist.angular.z = joy->axes[0]*angular_scale;
}

int main(int argc, char** argv) {
  twist.angular.x = twist.angular.y = twist.linear.y = twist.linear.z = 0;
  twist.linear.x = twist.linear.y = twist.linear.z = 0;
  ros::init(argc, argv, "joystick_interface");
  ros::NodeHandle nh;
  nh.getParam("joystick_control/linear_coeff", linear_scale);
  nh.getParam("joystick_control/angular_coeff", angular_scale); 
  ros::Subscriber sub;
  pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  sub = nh.subscribe("joy", 1, joyCallback);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    loop_rate.sleep();
    std::lock_guard<std::mutex> lock(twist_mutex);
    pub.publish(twist);
  }
  spinner.stop();
}