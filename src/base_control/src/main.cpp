#include "base_control.h"


int main(int argc, char** argv)
{
    init(argc, argv, "base_control");
    NodeHandle n;
    BaseControl base_control(n);


    // uint8_t data[6] = {0x00, 0xCB, 0x00, 0x00, 0x00, 0xCB};
    // uint8_t read_data[256], fc, len;

    // Rate loop_rate(10);
    // while (ok())
    // {
    //     ROS_INFO("Send.");
    //     base_control.serialSend(FC_SET_ACK_VEL, data, 6);    
    //     if (base_control.checkSerial())
    //     {
    //         ROS_INFO("Get response.");
    //         base_control.serialRead(fc, read_data, len);
    //     }
    //     spinOnce();
    //     loop_rate.sleep();
    // }

    spin();

    return 0;
}
