#include <iostream>
#include <string>
#include <sstream>
using namespace std;
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
//订阅的回调函数，订阅键盘节点发布的消息
void doMsg(const geometry_msgs::Twist& cmd_vel){

    ROS_INFO("Received a /cmd_vel message!");
	ROS_INFO("Linear Components:[%f,%f,%f]",cmd_vel.linear.x,cmd_vel.linear.y,cmd_vel.linear.z);
	ROS_INFO("Angular Components:[%f,%f,%f]",cmd_vel.angular.x,cmd_vel.angular.y,cmd_vel.angular.z);

}
 
int main(int argc, char *argv[])
{

    ros::init(argc,argv,"subscriber");
    //3.实例化 ROS 句柄
    ros::NodeHandle nh;
    //4.实例化 订阅者 对象
    ros::Subscriber sub = nh.subscribe("cmd_vel",1000,&doMsg);
    //5.处理订阅的消息(回调函数)
    ros::Rate loop_rate(10);
    //6.设置循环调用回调函数
    while (ros::ok())
        {

               loop_rate.sleep();
                ros::spinOnce();
        }
    ros::spin(); //循环读取接收的数据,并调用回调函数处理
    return 0;

}


//    rosrun serial_node key_Subcriber
//    rosrun teleop_twist_keyboard teleop_twist_keyboard.py

