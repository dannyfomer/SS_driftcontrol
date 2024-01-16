#include <iostream>
#include <string>
#include <sstream>
using namespace std;
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "serial_node/control_msg.h"
void doMsg(const serial_node::control_msg::ConstPtr& msg_p){

    ROS_INFO("我订阅:%f 和%f",msg_p->delta,msg_p->throttle);
}
 
int main(int argc, char *argv[])
{

    ros::init(argc,argv,"subscriber");
    //3.实例化 ROS 句柄
    ros::NodeHandle nh;
    //4.实例化 订阅者 对象
    ros::Subscriber sub = nh.subscribe<serial_node::control_msg>("control",1000,&doMsg);
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


//    rosrun serial_node Subcriber
//    rosrun teleop_twist_keyboard teleop_twist_keyboard.py


