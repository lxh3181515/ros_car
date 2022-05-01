#include "base_control.h"


BaseControl::BaseControl(NodeHandle n)
{
    /* topic init */
    odom_topic = n.advertise<geometry_msgs::Twist>("/odom", 10);
    imu_topic = n.advertise<sensor_msgs::Imu>("/imu", 10);
    /* serial init */
    ID = 0x01;
    device_port = "/dev/ttyTHS1";
    baudrate = 115200;
    serial::Timeout to = serial::Timeout::simpleTimeout(10);
    sp.setPort(device_port);
    sp.setBaudrate(baudrate);
    sp.setTimeout(to);
    ROS_INFO("Opening Serial");
    try
    {
        sp.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR("Can not open Serial %s", device_port.c_str());
        return;
    }
    ROS_INFO("Serial Open Succeed %s", device_port.c_str());
    /* others init */
    odom_freq = 50;
    imu_freq = 50;
    odomID = "odom";
    baseID = "base";
    imuID = "imu";
    serialIDLE_flag = 0;
    /* timer init */
    timer_communication = n.createTimer(Duration(1.0/500), &BaseControl::timerCommunicationCB, this);
    timer_odom = n.createTimer(Duration(1.0/odom_freq), &BaseControl::timerOdomCB, this);
    timer_imu = n.createTimer(Duration(1.0/imu_freq), &BaseControl::timerIMUCB, this);
}


uint8_t BaseControl::crc8_MAXIM(uint8_t *data, uint8_t len)
{
    uint8_t crc, i;
    crc = 0x00;

    while(len--)
    {
        crc ^= *data++;
        for(i = 0; i < 8; i++)
        {
            if(crc & 0x01)
            {
                crc = (crc >> 1) ^ 0x8c;
            }
            else crc >>= 1;
        }
    }
    return crc;
}


void BaseControl::serialSend(uint8_t fc, uint8_t *data, uint8_t len)
{
    uint8_t buff[256], i = 0;

    buff[0] = 0x5a;
    buff[1] = 6 + len;
    buff[2] = ID;
    buff[3] = fc;
    while (i < len)
    {
        buff[4 + i] = data[i];
        i++;
    }
    buff[4 + i] = 0x00;
    buff[5 + i] = crc8_MAXIM(buff, 5 + len);
    try
    {
        sp.write(buff, buff[1]);
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("serial send failed");
    }
}

void BaseControl::serialSend(uint8_t fc)
{
    uint8_t buff[6];

    buff[0] = 0x5a;
    buff[1] = 0x06;
    buff[2] = ID;
    buff[3] = fc;
    buff[4] = 0x00;
    buff[5] = crc8_MAXIM(buff, 5);
    try
    {
        sp.write(buff, buff[1]);
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("serial send failed");
    }
}


void BaseControl::timerCommunicationCB(const TimerEvent& event)
{
    /* read data */
    if (sp.available())
    {
        int len;
        std::string buff;
        buff = sp.read();
        len = buff.size();
        for (int i = 0; i < len; i++)
        {
            if (!Circleloop.enqueue((uint8_t)buff[i])) break; // ---check it out---
        }
    }
    /* handle data */
    if (!Circleloop.is_empty())
    {
        uint8_t data = Circleloop.get_front();
        if (data == 0x5a)
        {
            uint8_t len = Circleloop.get_front_second();
            if (len <= Circleloop.get_queue_length() && len > 1)
            {
                uint8_t databuf[256];
                for (int i = 0; i < len; i++)
                {
                    databuf[i] = Circleloop.get_front();
                    Circleloop.dequeue();
                }
                // crc check
                if (databuf[len - 1] != crc8_MAXIM(databuf, len - 1))
                {
                    ROS_WARN("crc check failed");
                }
                // function code
                switch(databuf[3])
                {
                    case FC_REP_SET_VEL:
                        ROS_WARN("velocity set failed!");
                        break;
                    // case FC_REP_VEL:
                    //     info_vel_ack.linear.x = float((uint16_t(databuf[4])<<8) + databuf[5]) / 1000.0;
                    //     info_vel_ack.linear.y = float((uint16_t(databuf[6])<<8) + databuf[7]) / 1000.0;
                    //     info_vel_ack.angular.z = float((uint16_t(databuf[8])<<8) + databuf[9]) / 1000.0;
                    //     break;
                    // case FC_REP_YAW:
                    //     yaw = float((uint16_t(databuf[8])<<8) + databuf[9]) / 100.0;
                    //     break;
                    case FC_REP_IMU:
                        info_imu_raw.angular_velocity.x = double((((databuf[4]&0xff)<<24)|((databuf[5]&0xff)<<16)|((databuf[6]&0xff)<<8)|(databuf[7]&0xff))/100000.0);
                        info_imu_raw.angular_velocity.y = double((((databuf[8]&0xff)<<24)|((databuf[9]&0xff)<<16)|((databuf[10]&0xff)<<8)|(databuf[11]&0xff))/100000.0);
                        info_imu_raw.angular_velocity.z = double((((databuf[12]&0xff)<<24)|((databuf[13]&0xff)<<16)|((databuf[14]&0xff)<<8)|(databuf[15]&0xff))/100000.0);
                        
                        info_imu_raw.linear_acceleration.x = double((((databuf[16]&0xff)<<24)|((databuf[17]&0xff)<<16)|((databuf[18]&0xff)<<8)|(databuf[19]&0xff))/100000.0);
                        info_imu_raw.linear_acceleration.y = double((((databuf[20]&0xff)<<24)|((databuf[21]&0xff)<<16)|((databuf[22]&0xff)<<8)|(databuf[23]&0xff))/100000.0);
                        info_imu_raw.linear_acceleration.z = double((((databuf[24]&0xff)<<24)|((databuf[25]&0xff)<<16)|((databuf[26]&0xff)<<8)|(databuf[27]&0xff))/100000.0);
                        
                        info_imu_raw.orientation.x = double(((databuf[28]&0xff)<<8|databuf[29])/10000.0);
                        info_imu_raw.orientation.y = double(((databuf[30]&0xff)<<8|databuf[31])/10000.0);
                        info_imu_raw.orientation.z = double(((databuf[32]&0xff)<<8|databuf[33])/10000.0);
                        info_imu_raw.orientation.w = double(((databuf[34]&0xff)<<8|databuf[35])/10000.0);
                        break;
                    case FC_REP_ODOM:
                        info_vel_ack.linear.x = float((uint16_t(databuf[4])<<8) + databuf[5]) / 1000.0;
                        yaw = float((uint16_t(databuf[6])<<8) + databuf[7]) / 100.0;
                        yaw = yaw * PI / 180.0;
                        info_vel_ack.angular.z = float((uint16_t(databuf[8])<<8) + databuf[9]) / 1000.0;
                        break;
                    default:
                        break;
                }
            }
            
        }
        else 
        {
            Circleloop.dequeue();
        }
    }
}



void BaseControl::timerOdomCB(const TimerEvent& event)
{
    /* get move base velocity data */
    while (serialIDLE_flag)
    {
        Duration(0.01).sleep();
    }
    serialIDLE_flag = 1;
    // serialSend(FC_GET_YAW);
    // serialSend(FC_GET_VEL);
    serialSend(FC_GET_ODOM);
    serialIDLE_flag = 0;

    /* publish odom data */
    tf::Transform trans;
    current_time = Time::now();
    double dt = (current_time - previous_time).toSec();
    previous_time = current_time;
    info_odom.header.stamp = current_time;
    info_odom.header.frame_id = odomID;
    info_odom.child_frame_id = baseID;

    // cal odom data
    info_odom.pose.pose.position.x += info_vel_ack.linear.x * cos(yaw) * dt - info_vel_ack.linear.y * sin(yaw) * dt;
    info_odom.pose.pose.position.y += info_vel_ack.linear.x * sin(yaw) * dt + info_vel_ack.linear.y * cos(yaw) * dt;
    info_odom.pose.pose.position.z = 0;
    trans.setOrigin(tf::Vector3(info_odom.pose.pose.position.x, info_odom.pose.pose.position.y, 0));

    info_odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    trans.setRotation(tf::createQuaternionFromYaw(yaw));

    info_odom.twist.twist.linear.x = info_vel_ack.linear.x;
    info_odom.twist.twist.linear.y = info_vel_ack.linear.y;
    info_odom.twist.twist.angular.z = info_vel_ack.angular.z;

    odom_topic.publish(info_odom);
    tf_broadcaster.sendTransform(tf::StampedTransform(trans, current_time, baseID, odomID));
}


void BaseControl::timerIMUCB(const TimerEvent& event)
{
    /* get IMU data */
    while (serialIDLE_flag)
    {
        Duration(0.01).sleep();
    }
    serialIDLE_flag = 3;
    serialSend(FC_GET_IMU);
    serialIDLE_flag = 0;

    /* publish IMU data */
    current_time = Time::now();
    info_imu_raw.header.stamp = current_time;
    info_imu_raw.header.frame_id = imuID;
    imu_topic.publish(info_imu_raw);
    tf_broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0.0, 0.0, 0.0, 1.0), tf::Vector3(0.0, 0.0, 0.09)), current_time, imuID, baseID));
}
