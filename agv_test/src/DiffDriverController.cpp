/*
 *ROS codes to communicate with base driver through serial-port
 *从cmd_vel上订阅的速度话题，默认的线速度属性不变，将默认的角速度换成角度。从消息定义处改变其属性和大小。
 *即command.linear.x=Vx,command.angular.z=angle。
 *需要一个从底层获取当前舵机转角current_angle的.cpp文件
 */
#include "DiffDriverController.h"
#include <time.h>

namespace agv_v3
{

DiffDriverController::DiffDriverController()//无参数构造函数
{
    max_wheelspeed=2.0;//轮子最大转速，单位是转/秒
    cmd_topic="cmd_vel";//速度话题名
    xq_status=new StatusPublisher();//小强的状态函数
    cmd_serial=NULL;//串口对象
    MoveFlag=true;//运动标志
}

//带参数的构造函数，函数重载,可以以相应的实参来调用该函数
DiffDriverController::DiffDriverController(double max_speed_,std::string cmd_topic_,StatusPublisher* xq_status_,CallbackAsyncSerial* cmd_serial_)
{
    MoveFlag=true;
    max_wheelspeed=max_speed_;
    cmd_topic=cmd_topic_;
    xq_status=xq_status_;
    cmd_serial=cmd_serial_;
}

void DiffDriverController::run()//驱动执行函数
{
    ros::NodeHandle nodeHandler;//创建节点对象（实例化节点）
    ros::Subscriber sub = nodeHandler.subscribe(cmd_topic, 1, &DiffDriverController::sendcmd, this);//从速度话题上订阅速度消息，经过处理后发给串口
    ros::Subscriber sub2 = nodeHandler.subscribe("/imu_cal", 1, &DiffDriverController::imuCalibration,this);//惯性测量单元标定
    ros::Subscriber sub3 = nodeHandler.subscribe("/globalMoveFlag", 1, &DiffDriverController::updateMoveFlag,this);//全局移动标志
    ros::Subscriber sub4 = nodeHandler.subscribe("/barDetectFlag", 1, &DiffDriverController::updateBarDetectFlag,this);//红外检测标志
    ros::spin();//保证节点一直运行

}
void DiffDriverController::updateMoveFlag(const std_msgs::Bool& moveFlag)//订阅globalMoveFlag后的回调函数，moveFlag是该话题上的消息数据。在.h中有声明。参数为bool类型
{
  boost::mutex::scoped_lock lock(mMutex);//锁定线程
  MoveFlag=moveFlag.data;//bool类型data属性

}
void DiffDriverController::imuCalibration(const std_msgs::Bool& calFlag)//订阅/imu_cal后的回调函数，calFlag是从话题上接收到的消息数据
{
  if(calFlag.data)//如果calFlag=true
  {
    //下发底层ｉｍｕ标定命令
    char cmd_str[5]={(char)0xcd,(char)0xeb,(char)0xd7,(char)0x01,(char)0x43};
    if(NULL!=cmd_serial)//串口对象不为空
    {
        cmd_serial->write(cmd_str,5);//向串口写标定IMU的命令数据
    }
  }
}
void DiffDriverController::updateBarDetectFlag(const std_msgs::Bool& DetectFlag)//订阅/barDetectFlag后的消息回调函数
{
  if(DetectFlag.data)
  {
    //下发底层红外开启命令
    char cmd_str[6]={(char)0xcd,(char)0xeb,(char)0xd7,(char)0x02,(char)0x44,(char)0x01};//最后一个字节为开启命令
    if(NULL!=cmd_serial)//串口对象不为空
    {
        cmd_serial->write(cmd_str,6);//向串口写开启红外的命令数据
    }
  }
  else
  {
    //下发底层红外禁用命令
    char cmd_str[6]={(char)0xcd,(char)0xeb,(char)0xd7,(char)0x02,(char)0x44,(char)0x00};//最后一个字节为关闭命令
    if(NULL!=cmd_serial)
    {
        cmd_serial->write(cmd_str,6);//向串口写关闭红外的命令数据
    }
  }
}


//----------------------   ros send speed command to serial-port--------------------------------
void DiffDriverController::sendcmd(const geometry_msgs::Twist &command)//订阅速度消息后的回调函数（处理后发给串口的操作）
{
    static time_t t1=time(NULL),t2;
    int i=0,wheel_ppr=1;
    double separation=0,radius=0,speed_lin=0,goal_angle=0,lmax_angle=0,rmax_angle=0,speed_temp[2];
    char speed[2]={0,0};//直行一旋转二
    double current_angle=0.0;
    //电机控制命令的格式
    char cmd_str[13]={(char)0xcd,(char)0xeb,(char)0xd7,(char)0x09,(char)0x74,(char)0x53,(char)0x53,(char)0x53,(char)0x53,(char)0x00,(char)0x00,(char)0x00,(char)0x00};

    std::cout << "this function is being called!!!!!!!!!" << std::endl;
    //if(xq_status->get_status()==0) return;//底层还在初始化
    separation=xq_status->get_wheel_separation();
    radius=xq_status->get_wheel_radius();
    wheel_ppr=xq_status->get_wheel_ppr();
    current_angle = xq_status->get_current_theta();
    //current_angle = 10.0;
    std::cout << "the current theta is:" << current_angle << std::endl;

    //command.linear.x=front and back speed;command.angular.speed=left and right speed;
    //convert the float command data [-1.0,1.0] into integer data between -100 to 100;
    //the + and - contained in direction bit in command
    speed_lin =command.linear.x/(2.0*PI*radius);//front-back,zhuan/s
    goal_angle=command.angular.z;//left-right,zhuan/s，给的目标转角是个带方向的数【-135,+135】
    //转出最大速度百分比,并进行限幅
    //make sure the two max_wheelspeeds are same
    //straight motor
    speed_temp[0]=(speed_lin/max_wheelspeed)*100.0;
    speed_temp[0]=std::min(speed_temp[0],100.0);
    speed_temp[0]=std::max(-100.0,speed_temp[0]);
    //rotate motor+max_angle最大可转角（lmax_angle，rmax_angle）,current_angle当前转角,angle目标转角
   //定义逆时针旋转角度为-，顺时针旋转角度为+,则写入串口的角度数范围为[0,120]
    lmax_angle= -135-current_angle;//向左最大可旋转角度
    rmax_angle=  135-current_angle;//向右最大可旋转角度
    speed_temp[1]= std::min(goal_angle,rmax_angle);//正的目标角度与向右最大可旋转角度比较取最小值
    speed_temp[1]= std::max(goal_angle,lmax_angle);//负的目标角度与向左最大可旋转角度比较取最大值
    //std::cout<<"radius "<<radius<<std::endl;
    //std::cout<<"ppr "<<wheel_ppr<<std::endl;
    //std::cout<<"pwm "<<speed_temp[0]<<std::endl;
    //command.linear.x
    //three move methods of motor
    for(i=0;i<2;i++)
    {
    speed[i]=speed_temp[i];//speed[0]right & left wheel,speed[1]forward&back wheel
    if(speed[i]<0)
    {
       cmd_str[5+i]=(char)0x42;//B,backward,negtive data,command type bit
       cmd_str[9+i]=-speed[i];//command data bit,all the data sent to the serial-port are positive.
    }
    else if(speed[i]>0)
    {
       cmd_str[5+i]=(char)0x46;//F,forward,positive data,command type bit
       cmd_str[9+i]=speed[i];//command data bit,the data means the total rounds the wheel should rotate
    }
    else
    {
       cmd_str[5+i]=(char)0x53;//S,stop
       cmd_str[9+i]=(char)0x00;//zero round
    }
    }

    /*
    boost::mutex::scoped_lock lock(mMutex);//线程锁
    if(!MoveFlag)
    {
      cmd_str[5]=(char)0x53;//motor1 stop
      cmd_str[6]=(char)0x53;//motor2 stop
    }
    */

    if(NULL!=cmd_serial)
    {
      std::cout <<  "write to cmd_serial!!!" << std::endl;
      cmd_serial->write(cmd_str,13);//write the vel_command to serial-port.
    }


   // command.linear.x
}


}
