#include <iostream>
#include <string>
#include <sstream>
using namespace std;

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "serial_node/control_msg.h"


typedef union
{
	float data;
	unsigned char data8[4];
} data_u;
data_u delta;
data_u throttle;

int main(int argc, char *argv[])
{

    //2.初始化 ROS 节点:命名(唯一)
    // 参数 1 和参数 2 后期为节点传值会使用
    // 参数 3 是节点名称,是一个标识符,需要保证运行后,在 ROS 网络拓扑中唯一
    ros::init(argc, argv, "control_publisher");
 
    //3.实例化 ROS 句柄
    ros::NodeHandle nh;  //该类封装了 ROS 中的一些常用功能
 
    //4.实例化 发布者 对象
    //泛型: 发布的消息类型
    //参数 1: 要发布到的话题
    //参数 2: 队列中最大保存的消息数,超出此阈值时,先进的先销毁(时间早的先销毁)
    ros::Publisher control_pub = nh.advertise<serial_node::control_msg>("control",100);
 
    //逻辑(一秒 10 次)
    ros::Rate loop_rate(10);
 
    //节点不死;
    while(ros::ok())
    {
		serial_node::control_msg msg;
		//编写消息
		msg.delta = delta.data;
		msg.throttle = throttle.data;
        //发布消息
        control_pub.publish(msg);

		loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}


//      rosrun serial_node control_publish
//       rostopic echo 
//      sudo apt-get install ros-noetic-teleop-twist-keyboard
/*
#安装键盘控制节点
sudo apt-get install ros-melodic-teleop-twist-keyboard
#运行
rosrun teleop_twist_keyboard teleop_twist_keyboard.py

或者源码安装
cd catkin_ws/src
git clone https://github.com/ros-teleop/teleop_twist_keyboard
cd ..
catkin_make

*/

