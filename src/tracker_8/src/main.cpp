#include "iostream"
#include "../include/tracker_8/circle8_vel.h"
//#include "../include/tracker_8/tracker_8_follow.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "circle8_vel");
  circle8_vel circle8;
  //tracker_8_follow follow;
  //circle8.ros_velocity_control();
  ros::MultiThreadedSpinner spinner(4); // Use 4 threads
  spinner.spin(); // spin() will not return until the node has been shutdown
  return 0;
}
