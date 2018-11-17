#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
 
class TeleopTurtleJoyStick
{
public:
  TeleopTurtleJoyStick();
  void keyLoop();
  void callBack(const sensor_msgs::Joy::ConstPtr& joy);
 
private:
  ros::NodeHandle nh_;
  double linear_, angular_, l_scale_, a_scale_;
  ros::Publisher twist_pub_;
  ros::Subscriber twist_sub_;
};
 
TeleopTurtleJoyStick::TeleopTurtleJoyStick():
  linear_(0),
  angular_(0),
  l_scale_(0.8),
  a_scale_(100)
{
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);
 
  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  twist_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopTurtleJoyStick::callBack, this);
}
 
void TeleopTurtleJoyStick::callBack(const sensor_msgs::Joy::ConstPtr& joy)
{
	geometry_msgs::Twist twist;
	twist.linear.x = joy->axes[1]; //speed
	twist.angular.z = joy->axes[0]; //angle
 
    twist_pub_.publish(twist);
}
 
int kfd = 0;
struct termios cooked, raw;
 
int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle_joystick");
  TeleopTurtleJoyStick teleop_turtle;
 
  ros::spin();
}

