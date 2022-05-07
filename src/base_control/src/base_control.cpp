#include "base_control.h"


BaseControl::BaseControl(NodeHandle n)
{
    /* topic init */
    odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 1);
    path_pub = n.advertise<nav_msgs::Path>("/path", 1);
    imu_pub = n.advertise<sensor_msgs::Imu>("/imu", 1);
    vel_ack_pub = n.advertise<geometry_msgs::TwistStamped>("/vel/info", 1);
    vel_ack_sub = n.subscribe("/vel/cmd", 1, &BaseControl::ackermannCmdCB, this);
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
    /* wait for imu init - 2s */
    getInfo();
    ROS_INFO("init done.");
}

void BaseControl::getInfo()
{
    while(serialIDLE_flag)
    {
        Duration(0.01).sleep();
    }
    serialIDLE_flag = 1;
    serialSend(FC_GET_VERSION);
    Duration(0.01).sleep();
    serialSend(FC_GET_SN);
    Duration(0.01).sleep();
    serialSend(FC_GET_CHASIS_INFO);
    serialIDLE_flag = 0;
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
    // ROS_INFO("queue len = %d", Circleloop.get_queue_length());
    /* read data */
    if (sp.available())
    {
        int len;
        std::string buff;
        buff = sp.read(sp.available());
        len = buff.size();
        for (int i = 0; i < len; i++)
        {
            Circleloop.enqueue(buff[i]);
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
                if (databuf[3] == FC_REP_IMU)
                {
                    temp_info_imu_raw.angular_velocity.x = int(((databuf[4]&0xff)<<24)|((databuf[5]&0xff)<<16)|((databuf[6]&0xff)<<8)|(databuf[7]&0xff));
                    temp_info_imu_raw.angular_velocity.y = int(((databuf[8]&0xff)<<24)|((databuf[9]&0xff)<<16)|((databuf[10]&0xff)<<8)|(databuf[11]&0xff));
                    temp_info_imu_raw.angular_velocity.z = int(((databuf[12]&0xff)<<24)|((databuf[13]&0xff)<<16)|((databuf[14]&0xff)<<8)|(databuf[15]&0xff));
                    
                    temp_info_imu_raw.linear_acceleration.x = int(((databuf[16]&0xff)<<24)|((databuf[17]&0xff)<<16)|((databuf[18]&0xff)<<8)|(databuf[19]&0xff));
                    temp_info_imu_raw.linear_acceleration.y = int(((databuf[20]&0xff)<<24)|((databuf[21]&0xff)<<16)|((databuf[22]&0xff)<<8)|(databuf[23]&0xff));
                    temp_info_imu_raw.linear_acceleration.z = int(((databuf[24]&0xff)<<24)|((databuf[25]&0xff)<<16)|((databuf[26]&0xff)<<8)|(databuf[27]&0xff));
                    
                    temp_info_imu_raw.orientation.x = int((databuf[28]&0xff)<<8|databuf[29]);
                    temp_info_imu_raw.orientation.y = int((databuf[30]&0xff)<<8|databuf[31]);
                    temp_info_imu_raw.orientation.z = int((databuf[32]&0xff)<<8|databuf[33]);
                    temp_info_imu_raw.orientation.w = int((databuf[34]&0xff)<<8|databuf[35]);
                } else if (databuf[3] == FC_REP_ODOM_EX)
                {
                    temp_info_vel_ack.linear.x = short((databuf[4]&0xff)<<8|databuf[5]);
                    temp_info_vel_ack.linear.y = short((databuf[6]&0xff)<<8|databuf[7]);
                    temp_yaw = short((databuf[8]&0xff)<<8|databuf[9]);
                    temp_info_vel_ack.angular.z = short((databuf[10]&0xff)<<8|databuf[11]);
                } else if (databuf[3] == FC_REP_SET_VEL)
                {
                    ROS_WARN("velocity set failed!");
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
    serialSend(FC_GET_ODOM_EX);
    serialIDLE_flag = 0;

    /* publish */
    static long long cnt = 0;
    double dt;
    std_msgs::Header header;
    current_time = Time::now();Duration(0.01).sleep();
    dt = (current_time - previous_time).toSec();
    previous_time = current_time;
    header.frame_id = odomID;
    header.seq = cnt;
    header.stamp = current_time;
    
    // restore data
    info_vel_ack.linear.x = temp_info_vel_ack.linear.x / 1000.0;
    info_vel_ack.linear.y = temp_info_vel_ack.linear.y / 1000.0;
    info_vel_ack.angular.z = temp_info_vel_ack.angular.z / 1000.0;
    yaw = temp_yaw * PI / 18000.0;

    // cal speed in world frame && pub twist
    geometry_msgs::TwistStamped twist;
    twist.header = header;
    twist.twist.linear.x = info_vel_ack.linear.x * cos(yaw) * dt - info_vel_ack.linear.y * sin(yaw) * dt;
    twist.twist.linear.y = info_vel_ack.linear.x * sin(yaw) * dt + info_vel_ack.linear.y * cos(yaw) * dt;
    vel_ack_pub.publish(twist);

    // cal pos in world frame
    info_odom.header = header;
    info_odom.pose.pose.position.x += twist.twist.linear.x;
    info_odom.pose.pose.position.y += twist.twist.linear.y;
    info_odom.pose.pose.position.z = 0;
    info_odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    info_odom.twist.twist.linear.x = info_vel_ack.linear.x;
    info_odom.twist.twist.linear.y = info_vel_ack.linear.y;
    info_odom.twist.twist.angular.z = info_vel_ack.angular.z;
    // odom_pub.publish(info_odom);

    // cal transform && broadcast
    tf::Transform trans;
    trans.setOrigin(tf::Vector3(info_odom.pose.pose.position.x, info_odom.pose.pose.position.y, 0));
    trans.setRotation(tf::createQuaternionFromYaw(yaw));
    tf_broadcaster.sendTransform(tf::StampedTransform(trans, current_time, baseID, odomID));

    // pub path
    info_path.header = header;
    info_path.header.frame_id = baseID;
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.header = header;
    this_pose_stamped.pose.position = info_odom.pose.pose.position;
    this_pose_stamped.pose.orientation = info_odom.pose.pose.orientation;
    info_path.poses.push_back(this_pose_stamped);
    path_pub.publish(info_path);

    cnt++;
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
    info_imu_raw.angular_velocity.x = temp_info_imu_raw.angular_velocity.x / 100000.0;
    info_imu_raw.angular_velocity.y = temp_info_imu_raw.angular_velocity.y / 100000.0;
    info_imu_raw.angular_velocity.z = temp_info_imu_raw.angular_velocity.z / 100000.0;
    info_imu_raw.linear_acceleration.x = temp_info_imu_raw.linear_acceleration.x / 100000.0;
    info_imu_raw.linear_acceleration.y = temp_info_imu_raw.linear_acceleration.y / 100000.0;
    info_imu_raw.linear_acceleration.z = temp_info_imu_raw.linear_acceleration.z / 100000.0;
    info_imu_raw.orientation.x = temp_info_imu_raw.orientation.x / 10000.0;
    info_imu_raw.orientation.y = temp_info_imu_raw.orientation.y / 10000.0;
    info_imu_raw.orientation.z = temp_info_imu_raw.orientation.z / 10000.0;
    info_imu_raw.orientation.w = temp_info_imu_raw.orientation.w / 10000.0;

    current_time = Time::now();
    info_imu_raw.header.stamp = current_time;
    info_imu_raw.header.frame_id = imuID;
    imu_pub.publish(info_imu_raw);
    tf_broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0.0, 0.0, 0.0, 1.0), tf::Vector3(0.0, 0.0, 0.09)), current_time, imuID, baseID));
}


void BaseControl::ackermannCmdCB(const geometry_msgs::TwistStamped& msg)
{
    uint8_t data[6];
    float theta, speed;
    theta = atan2(msg.twist.linear.y, msg.twist.linear.x) - yaw;
    speed = sqrt(pow(msg.twist.linear.x, 2) + pow(msg.twist.linear.y, 2));
    data[0] = (short(speed * 1000.0)>>8)&0xff;
    data[1] = short(speed * 1000.0)&0xff;
    data[2] = data[3] = 0;
    data[4] = (short(theta * 1000.0)>>8)&0xff;
    data[5] = short(theta * 1000.0)&0xff;

    while (serialIDLE_flag)
    {
        Duration(0.01).sleep();
    }
    serialIDLE_flag = 4;
    serialSend(FC_SET_ACK_VEL, data, 6);
    serialIDLE_flag = 0;
}
