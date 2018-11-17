#ifndef DIFFDRIVERCONTROLLER_H
#define DIFFDRIVERCONTROLLER_H
#include <ros/ros.h>
#include "StatusPublisher.h"
#include "AsyncSerial.h"
#include <std_msgs/Bool.h>

namespace agv_v3
{

class DiffDriverController
{
public:
    DiffDriverController();//构造函数，相当于python里的__init__初始化函数。
	//需要加获取current_angle的类函数的参数
    DiffDriverController(double max_speed_,std::string cmd_topic_,StatusPublisher* xq_status_,CallbackAsyncSerial* cmd_serial_);
    void run();//以下均为订阅话题的消息回调函数
    void sendcmd(const geometry_msgs::Twist& command);
    void imuCalibration(const std_msgs::Bool& calFlag);
    void setStatusPtr(StatusPublisher& status);
    void updateMoveFlag(const std_msgs::Bool& moveFlag);//回调函数声明
    void updateBarDetectFlag(const std_msgs::Bool& DetectFlag);
	
    //private里的为封装的数据，类的对象可以使用
private:
    double max_wheelspeed;//单位为转每秒,只能为正数
    std::string cmd_topic;
    StatusPublisher* xq_status;
    CallbackAsyncSerial* cmd_serial;
    boost::mutex mMutex;//线程变量
    bool MoveFlag;//布尔型运动标志
};

}
#endif // DIFFDRIVERCONTROLLER_H
