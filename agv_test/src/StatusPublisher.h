#ifndef STATUSPUBLISHER_H
#define STATUSPUBLISHER_H

#include "ros/ros.h"
#include <boost/thread.hpp>
#include <boost/assign/list_of.hpp>
#include <algorithm>
#include "geometry_msgs/Pose2D.h"   
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "tf/transform_broadcaster.h"
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointField.h>
#include "sensor_msgs/Imu.h"

#define PI 3.14159265//宏定义

namespace agv_v3
{
	//定义小车状态结构体，用于接收底层上传上来的各种传感器数据
typedef struct {
    int status;//小车状态，0表示未初始化，1表示正常，-1表示error
    float power;//电源电压【9 13】v
    float theta;//方位角，【0 360）°
    int encoder_ppr;//车轮1转对应的编码器个数
    int encoder_delta_r;//右轮编码器增量， 个为单位
    int encoder_delta_l;//左轮编码器增量， 个为单位
    int encoder_delta_car;//两车轮中心位移，个为单位
    int omga_r;//右轮转速 个每秒
    int omga_l;//左轮转速 个每秒
    float distance1;//第一个超声模块距离值 单位cm
    float distance2;//第二个超声模块距离值 单位cm
    float distance3;//第三个超声模块距离值 单位cm
    float distance4;//第四个超声模块距离值 单位cm
    float IMU[9];//mpu9250 9轴数据;三轴陀螺仪，三轴加速度计，三轴磁力计
    unsigned int time_stamp;//时间戳
}UPLOAD_STATUS;//上传状态

class StatusPublisher//定义状态发布类
{
public:
    StatusPublisher();//构造函数
    StatusPublisher(double separation,double radius);//构造函数
    void Refresh();//状态刷新函数
    void Update(const char *data, unsigned int len);//更新函数
    double get_wheel_separation();//获取车轮的间距
    double get_wheel_radius();//获取车轮的半径
    double get_current_theta();
    int get_wheel_ppr();//获取车轮的个数
    int get_status();//获取状态
    geometry_msgs::Pose2D get_CarPos2D();//获取小车二维位姿信息
    void get_wheel_speed(double speed[2]);//获取小车实际轮子转速
    geometry_msgs::Twist get_CarTwist();//获取车的速度
    std_msgs::Float64 get_power();//获取电量
    nav_msgs::Odometry get_odom();//获取里程计信息
    UPLOAD_STATUS car_status;//定义结构体对象car_status

private:
    //封装

    //Wheel separation, wrt the midpoint of the wheel width: meters
    double wheel_separation;//小车轮子宽度的中点的距离

    //Wheel radius (assuming it's the same for the left and right wheels):meters
    double wheel_radius;//左右轮子的半径一样

    geometry_msgs::Pose2D CarPos2D;//小车开始启动原点坐标系
    geometry_msgs::Twist  CarTwist;//小车自身坐标系
    std_msgs::Float64 CarPower;// 小车电池信息
    nav_msgs::Odometry CarOdom;// 小车位置和速度信息
    ros::NodeHandle mNH;//实例化节点mH
    ros::Publisher mPose2DPub;//定义二维位姿发布器
    ros::Publisher mTwistPub;//定义速度发布器
    ros::Publisher mStatusFlagPub;//定义状态标志发布器
    ros::Publisher mPowerPub;//定义电量发布器
    ros::Publisher mOdomPub;//定义里程计消息发布器
    ros::Publisher pub_barpoint_cloud_;//定义红外信息发布器
    ros::Publisher pub_clearpoint_cloud_;//定义点云发布器

    bool mbUpdated;//声明布尔型变量

    boost::mutex mMutex;//声明线程变量
    double base_time_;

    ros::Publisher mIMUPub;//定义一个IMU发布器
    sensor_msgs::Imu CarIMU;//定义一个九轴传感器变量sensor_msgs/Imu.msg
};

} //namespace xqserial_server


#endif // STATUSPUBLISHER_H
