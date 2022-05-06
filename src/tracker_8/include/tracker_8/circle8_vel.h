#ifndef CIRCLE8_VEL_H
#define CIRCLE8_VEL
#include "iostream"
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/TwistStamped.h>
#include "ros/time.h"
#include "pthread.h"
#include <queue>
//#include <mavros_msgs/CommandBool.h>
//#include <mavros_msgs/SetMode.h>
//#include <mavros_msgs/State.h>
#include "eigen3/Eigen/Dense"
#include "sensor_msgs/Range.h"
#include "std_msgs/String.h"
#include "nlink_parser/LinktrackNodeframe0.h"
#include "nlink_parser/LinktrackNodeframe2.h"
#include <math.h>
#include <algorithm>
#include <string.h>
//#include <uwb_common/uwb_common.h>

class circle8_vel
{
public:
    circle8_vel();
    //static void *run(void *args);
    
    //void UAV0_phi(const geometry_msgs::PoseStamped::ConstPtr& msg);

   // void ros_velocity_control(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void vel_callback(const geometry_msgs::TwistStamped::ConstPtr& msg);
    //void height_callback(const sensor_msgs::Range::ConstPtr& msg);
    void uwb_distance_callback(const nlink_parser::LinktrackNodeframe2::ConstPtr& msg);
    
private:
    //std::string _uav1TwistSubTopic;
   // std::string _uav0PhiSubTopic;
    //std::string _uav1SetVelPubTopic;
    //std::string _uav1HeighSubTopic;
    std::string _uwbDistanceSubTopic;
   // std::string _msgToUwbTopic;
std::string _carSetVelPubTopic;
std::string _carTwistSubTopic;
  //  ros::Subscriber _uav0TwistSub;
  //  ros::Subscriber _uav0PhiSub;
   // ros::Publisher _uav1SetVelPub;
   // ros::Publisher  _msgToUwbPub;

ros::Publisher  data_matlab;
ros::Publisher  carSetVelPub;

ros::Subscriber carTwist_sub;

	//ros::Subscriber flow0_sub;
	//ros::Subscriber flow1_sub;
	//ros::Subscriber height_sub;
	ros::Subscriber distance_sub ;



    float ref_velocity_x;
    float ref_velocity_y;
    float ref_velocity_z;

    Eigen:: Matrix<double, 2, 1>  q_est;
    Eigen:: Matrix<double, 2, 1>  q_est_old;
     Eigen:: Matrix<double, 2, 2>  bgamak;

    std::queue<double> UAV0_x;
    std::queue<double> UAV0_y;
    std::queue<double> UAV1_x;
    std::queue<double> UAV1_y;
    std::vector<double> distance;

    double UAV0_phi_x;
    double UAV0_phi_y;
float car_phi_x;
    float car_phi_y;
    
 	double UAV_0_high_sensor_value;
	double UAV1_high_sensor_value;
	double UAV1_phi_x;
    double UAV1_phi_y;
    double num;
    double k;
    double flow_delta_time_begin;
    double flow_delta_time_end;
    double flow_delta_t ;
    double vel_x;
    double vel_y;
    pthread_t run_thread;

    int get_times;

    int64_t data_start;

    bool is_uav1_data_ok;
    bool init;
};

#endif // VELOCITY_CONTROL_H

