
#include "AsyncSerial.h"//serial port yibu conmmunication

#include <iostream>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include "DiffDriverController.h"
#include "StatusPublisher.h"

using namespace std;

int main(int argc, char **argv)
{
    cout<<"welcome to AGV serial server,please feel free at home!"<<endl;

    ros::init(argc, argv, "agv_v3");//initial the node
    ros::start();

    //获取串口参数
    std::string port;//define variable
    ros::param::param<std::string>("~port", port, "/dev/ttyUSB001");//assign port value
    int baud;//define baud
    ros::param::param<int>("~baud", baud, 115200);//assign baud value
    cout<<"port:"<<port<<" baud:"<<baud<<endl;//print the port and baud value
    //获取小车机械参数
    double separation=0,radius=0;//define the wheels distance and radius
    bool DebugFlag = false;
    ros::param::param<double>("~wheel_separation", separation, 0.37);//ros方式设置参数的值
    ros::param::param<double>("~wheel_radius", radius, 0.09);
    ros::param::param<bool>("~debug_flag", DebugFlag, false);
    agv_v3::StatusPublisher xq_status(separation,radius);
    //获取小车控制参数
    double max_speed;
    string cmd_topic;
    ros::param::param<double>("~max_speed", max_speed, 2.0);
    ros::param::param<std::string>("~cmd_topic", cmd_topic, "cmd_vel");

    try {
        CallbackAsyncSerial serial(port,baud);//以实参port和baud调用创建的串口对象serial
        serial.setCallback(boost::bind(&agv_v3::StatusPublisher::Update,&xq_status,_1,_2));//串口设置回调函数
        agv_v3::DiffDriverController xq_diffdriver(max_speed,cmd_topic,&xq_status,&serial);
        boost::thread cmd2serialThread(& agv_v3::DiffDriverController::run,&xq_diffdriver);
        // send test flag
        char debugFlagCmd[] = {(char)0xcd, (char)0xeb, (char)0xd7, (char)0x01, 'T'};
        if(DebugFlag){
          std::cout << "Debug mode Enabled" << std::endl;
          serial.write(debugFlagCmd, 5);
        }
        // send reset cmd
        char resetCmd[] = {(char)0xcd, (char)0xeb, (char)0xd7, (char)0x01, 'I'};
        serial.write(resetCmd, 5);

        ros::Duration(0.5).sleep();
        ros::Rate r(50);
        while (ros::ok())
        {
            if(serial.errorStatus() || serial.isOpen()==false)
            {
                cerr<<"Error: serial port closed unexpectedly"<<endl;
                break;
            }
            ros::spinOnce();
            xq_status.Refresh();//定时发布状态
            r.sleep();
            //cout<<"run"<<endl;
        }

        quit:
        serial.close();

    } catch (std::exception& e) {
        cerr<<"Exception: "<<e.what()<<endl;
    }

    ros::shutdown();
    return 0;
}
