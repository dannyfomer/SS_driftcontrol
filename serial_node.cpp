#include <iostream>
#include <string>
#include <sstream>
using namespace std;

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <serial/serial.h>

#include "serial_node/my_msg.h"
#include "serial_node/control_msg.h"
#define sBUFFER_SIZE 1024
#define rBUFFER_SIZE 1024
unsigned char s_buffer[sBUFFER_SIZE];
unsigned char r_buffer[rBUFFER_SIZE];

serial::Serial ser;
typedef union
{
	float data;
	unsigned char data8[4];
} data_u;
data_u pitch;
data_u roll;
data_u yaw;
data_u delta;
data_u throttle;
//订阅的回调函数  ，订阅发布的控制量消息

void doMsg(const serial_node::control_msg::ConstPtr& msg_p){

    ROS_INFO(" 我订阅:%f 和%f ",msg_p->delta,msg_p->throttle);
	delta.data=msg_p->delta;
	throttle.data=msg_p->throttle;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_port");
    //创建句柄（虽然后面没用到这个句柄，但如果不创建，运行时进程会出错）
    ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe<serial_node::control_msg>("control",1000,&doMsg);
    //创建一个serial对象
    serial::Serial ser;
    //创建timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    //设置要打开的串口名称
    ser.setPort("/dev/ttyRplidar");
    //设置串口通信的波特率
    ser.setBaudrate(9600);
    //串口设置timeout
    ser.setTimeout(to);
 
    try
    {
        //打开串口
        ser.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }
    
    //判断串口是否打开成功
    if(ser.isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
    }
    else
    {
        return -1;
    }
    //发送时间 Hz
    ros::Rate loop_rate(10);
	
	int cnt = 0;
	//数据包 格式先转角后油门见文档，控制下位机驱动舵机和电机
	uint8_t stm32_msg[10] = {0xA0,0xB0,0x00,0x00,0x20,0x41,0x00,0x00,0x20,0x41};
	ros::Publisher imu_pub = nh.advertise<serial_node::my_msg>("imu", 1000);
	while(ros::ok())
	{
		//发送串口消息
		for(int i=0;i<4;i++)
		{
			stm32_msg[i+2]=delta.data8[i];
			stm32_msg[i+6]=throttle.data8[i];
		}
		ser.write(stm32_msg, 10);


		serial_node::my_msg msg;
		//ser.write(stm32_msg, 10);
		if(ser.available())
		{
			// 读取串口数据
			size_t bytes_read = ser.read(r_buffer, ser.available());
	
			// 使用状态机处理读取到的数据，通信协议与STM32端一致
			int state = 0;
			unsigned char buffer[12] = {0};
			
			for(int i = 0; i < bytes_read && i < rBUFFER_SIZE; i++)
			{
				
				if(state == 0 && r_buffer[i] == 0x0a)
				{
					state++;
					
				}
				else if(state == 1 && r_buffer[i] == 0x0b)
				{
					state++;
					
				}
				else if(state >= 2 && state < 14)
				{
					
					buffer[state-2]=r_buffer[i];
					state ++;
					if(state == 14)
					{
						for(int k = 0; k < 4; k++)
						{
							roll.data8[k] = buffer[k];
							pitch.data8[k] = buffer[4 + k];
							yaw.data8[k] = buffer[8 + k];
						}						
						ROS_INFO("%f, %f, %f, %d", roll.data*180/3.1415, pitch.data*180/3.1415, yaw.data*180/3.1415, cnt);
						
						state = 0;
						
					}
				}

				
				else 
					state = 0;
			}
			
		}
		// 发布接收到的imu数据
		msg.roll = roll.data;
		msg.pitch = pitch.data;
		msg.yaw = yaw.data;
		imu_pub.publish(msg);
		 
		loop_rate.sleep();
		ros::spinOnce();
		cnt++;
	}
	// 关闭串口
	ser.close();
	return 0;
}

//      rosrun serial_node serial_node
//      catkin_make
