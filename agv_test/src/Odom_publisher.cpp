#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odometry_publisher");//初始化节点
  ros::NodeHandle n;//节点实例化为n
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);//定义一个消息发布者来发布“odom”消息
  tf::TransformBroadcaster odom_broadcaster;//定义一个tf广播，来发布tf变换信息
  //默认机器人的起始位置是odom参考系下的0点。（x，y，theta）=（0,0,0）
  double x = 0.0;
  double y = 0.0;
  double th = 0.0;
  //机器人的默认前进速度；让机器人的base_link参考系在odom参考系下以x轴方向0.1m/s，
  //Y轴速度-0.1m/s，角速度0.1rad/s的状态移动，这种状态下，可以让机器人保持圆周运动。
  double vx = 0.1;
  double vy = -0.1;
  double vth = 0.1;
  ros::Time current_time, last_time;

  current_time = ros::Time::now();

  last_time = ros::Time::now();
  // //使用1Hz的频率发布odom消息，当然，在实际系统中，往往需要更快的速度进行发布。
  ros::Rate r(1.0);

  //使用我们设置的速度信息，来计算并更新里程计的信息，包括单位时间内机器人在x轴、
  //y轴的坐标变化和角度的变化。在实际系统中，需要更具里程计的实际信息进行更新。
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages

    current_time = ros::Time::now();
    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();//计算delta_t时间
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;//通过积分方式计算累计里程
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;//对于单舵轮，是根据运动模型来计算的。
    double delta_th = vth * dt;
    x += delta_x;
    y += delta_y;
    th += delta_th;

    //为了兼容二维和三维的功能包，让消息结构更加通用，里程计的偏航角需要转换成四元数才能发布，
    //幸运的是，ROS为我们提供了偏航角与四元数相互转换的功能。
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    //创建一个tf发布需要使用的TransformStamped类型消息，
    geometry_msgs::TransformStamped odom_trans;
    //然后根据消息结构填充当前的时间戳、参考系id、子参考系id，
    //注意两个参考系的id必须要是“odom”和“base_link”。
    odom_trans.header.stamp = current_time;
    //odom为里程计坐标系
    odom_trans.header.frame_id = "odom";
    //base_link是车体基座坐标系
    odom_trans.child_frame_id = "base_link";

    //填充里程计信息，然后发布tf变换的消息。
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform

    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    //我们还要发布nav_msgs/Odometry消息，让导航包获取机器人的速度。
    //创建消息变量，然后填充时间戳。
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;//线速度
    odom.twist.twist.linear.y = vy;//
    odom.twist.twist.angular.z = vth;//角速度

    //publish the message

    odom_pub.publish(odom);
    last_time = current_time;

    r.sleep();

  }

}
