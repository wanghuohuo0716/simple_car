#include <ros/ros.h>
#include <serial/serial.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/String.h>
#include "Decode_frame.hpp"
serial::Serial ser;

int main (int argc, char** argv)
{
    //初始化节点
    ros::init(argc, argv, "serial_example_node");
    //声明节点句柄
    ros::NodeHandle nh;
    ros::Publisher pub_imu = nh.advertise<geometry_msgs::Vector3>("turtle1/imu_angle", 1000);
    geometry_msgs::Vector3 imu_data;
    std::string data; // 接收串口的单个字节
    try
    {
        //设置串口属性，并打开串口
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    //检测串口是否已经打开，并给出提示信息
    if(ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else
    {
        return -1;
    }
    ros::Rate loop_rate(1000);
    // 根据协议接收到完成的消息就发topic
    while(ros::ok())
    {
        if(ser.available()){
            data = ser.read();
            Decode_frame(data[0]);	//进入解帧程序,IMU更新速率不超过300Hz
        };

        imu_data.x = yaw;
        imu_data.y = pitch;
        imu_data.z = roll;
        ROS_INFO("yaw=%f,pitch=%f,roll=%f", yaw, pitch, roll);

        pub_imu.publish(imu_data);
        loop_rate.sleep();
    }
}
