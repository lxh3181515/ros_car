#ifndef __BASE_CONTROL_H
#define __BASE_CONTROL_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "serial/serial.h"
#include "function_code.h"
#include "queue.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"

using namespace ros;
using namespace std_msgs;

#define PI 3.14159265

class BaseControl
{
private:
    // get param
    uint8_t ID; 
    std::string odomID;
    std::string baseID;
    std::string imuID;
    std::string device_port;
    uint32_t baudrate;
    uint16_t odom_freq;
    uint16_t imu_freq;
    Publisher odom_topic;
    Publisher vel_ack_topic;
    Publisher imu_topic;

    // define param
    Time current_time = Time::now();
    Time previous_time = current_time;
    geometry_msgs::Twist info_vel_ack;
    geometry_msgs::Twist cmd_vel_ack;
    nav_msgs::Odometry info_odom;
    sensor_msgs::Imu info_imu_raw;
    float yaw;
    serial::Serial sp;
    uint8_t serialIDLE_flag;
    queue Circleloop;
    Timer timer_communication;
    Timer timer_odom;
    Timer timer_imu;
    tf::TransformBroadcaster tf_broadcaster;

public:
    BaseControl(NodeHandle n);

    uint8_t crc8_MAXIM(uint8_t *data, uint8_t len);
    void serialSend(uint8_t fc, uint8_t *data, uint8_t len);
    void serialSend(uint8_t fc);
    
    void timerCommunicationCB(const TimerEvent& event);
    void timerOdomCB(const TimerEvent& event);
    void timerIMUCB(const TimerEvent& event);

    void getInfo();
};


#endif
