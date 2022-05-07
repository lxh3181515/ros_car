#include "../include/tracker_8/circle8_vel.h"
#include "velcal.cpp"

#include <tracker_8/data_get.h>

using namespace std;
circle8_vel::circle8_vel()
{
    ros::NodeHandle nh1("~");
    // nh1.getParam("Uav1TwistSubTopic",_uav1TwistSubTopic);
    // nh1.getParam("Uav0PhiSubTopic",_uav0PhiSubTopic);
    // nh1.getParam("CarSetVelPubTopic",_carSetVelPubTopic);
    // nh1.getParam("Uav1HeighSubTopic",_uav1HeighSubTopic);
    nh1.getParam("CarSetVelPubTopic",_carSetVelPubTopic);  //find the car vel topic
    nh1.getParam("UwbDistanceSubTopic",_uwbDistanceSubTopic); // find the distance topic
    nh1.getParam("CarTwistSubTopic",_carTwistSubTopic); //feedback the car vel topic
    

    // nh1.getParam("UwbMessagePubTopic",_msgToUwbTopic);



      data_matlab= nh1.advertise<tracker_8::data_get>("/data_analysis_matlab",1);

     carSetVelPub = nh1.advertise<geometry_msgs::TwistStamped>(_carSetVelPubTopic, 1);

    // flow1_sub = nh1.subscribe<geometry_msgs::TwistStamped>(_uav1TwistSubTopic, 1, &circle8_vel::vel_callback,this);//flow1
    // height_sub = nh1.subscribe<sensor_msgs::Range>(_uav1HeighSubTopic, 1, &circle8_vel::height_callback, this); //#height
    distance_sub = nh1.subscribe<nlink_parser::LinktrackNodeframe2>(_uwbDistanceSubTopic, 1,&circle8_vel::uwb_distance_callback, this); //dis
    carTwist_sub = nh1.subscribe<geometry_msgs::TwistStamped>(_carTwistSubTopic, 1, &circle8_vel::vel_callback,this);
   // _uav0PhiSub  = nh1.subscribe<geometry_msgs::PoseStamped>(_uav0PhiSubTopic,1, &circle8_vel::ros_velocity_control,this); //#flow0
    // _msgToUwbPub = nh1.advertise<std_msgs::String>(_msgToUwbTopic,1);
    num = 0;
    k = 0;
    get_times = 0;

    distance.resize(2);
    distance.at(0) = 0;
    distance.at(1) = 0;

    
    data_start=(ros::Time::now().toSec()*1000);
    is_uav1_data_ok = false;
    init=true;
    q_est<<0,0;
    q_est_old<<0,0;
	bgamak << 0.1,0,0,0.1;

}


//void circle8_vel::ros_velocity_control(const geometry_msgs::PoseStamped::ConstPtr& msg)
//{
//}
void circle8_vel::vel_callback(const geometry_msgs::TwistStamped::ConstPtr& msg)///
{  
    // vel_x =msg->twist.linear.x;
    // vel_y = msg->twist.linear.y;
    // if(init)
    // {
    //     flow_delta_time_begin = ros::Time::now().toSec();
    //     flow_delta_time_end   = ros::Time::now().toSec();
    //     init = false;
    // }
    // else
    // {
    //     flow_delta_time_end   = ros::Time::now().toSec();
    //     flow_delta_t = flow_delta_time_end-flow_delta_time_begin;//单位：秒
    //     flow_delta_time_begin = ros::Time::now().toSec();
        
    //     car_phi_x  += vel_x * flow_delta_t;
    //     car_phi_y  += vel_y * flow_delta_t;//E
    //     ROS_INFO("UAV1_phi_y=%f",UAV1_phi_y);
    //     is_uav1_data_ok=true;
    // }
    car_phi_x=msg->twist.linear.x;
    car_phi_y=msg->twist.linear.y;
    is_uav1_data_ok=true;
    num+=1.0;
}
// void circle8_vel::height_callback(const sensor_msgs::Range::ConstPtr& msg)
// {
//     UAV1_high_sensor_value = msg.get()->range;
// }
void circle8_vel::uwb_distance_callback(const nlink_parser::LinktrackNodeframe2::ConstPtr& msg)
{  // if(!is_uav1_data_ok)
    {
        if(msg.get()->nodes.size()>=1)
        {
            for(int i= 0;i<msg.get()->nodes.size();i++)
            {
                if(msg.get()->nodes.at(i).id == 0)
                {
                    distance.at(1) =(msg.get()->nodes.at(i).dis);
                    
                    
                    if(is_uav1_data_ok)
    {
   // ROS_INFO("come in1");
     UAV0_phi_x=0;
     UAV0_phi_y=0;
    // UAV_0_high_sensor_value = 0;
    
        Eigen:: Matrix<double, 2, 1>  v_est(0.0,0.0);
        Eigen:: Matrix<double, 2, 1>  phi_xy(0.0,0.0);
        Eigen:: Matrix<double, 2, 1>  vk(0.0,0.0);
        Eigen:: Matrix<double, 2, 4>   getretu;
        // k= k + 1.0;
        // if (num==0)
        // { UAV0_phi_x=0;
        //     UAV0_phi_y=0;
        // }
        // else{
        //     UAV0_phi_x=UAV0_phi_x;
        //     UAV0_phi_y=UAV0_phi_y;
        // }
        
        // cout<<UAV0_phi_x<<endl;
        // cout<<UAV0_x.front()<<endl;
        cout<<distance.size()<<endl;
        
        ROS_INFO("num==%f",num);
        //double d_now=sqrt(pow(UAV1_x.back()-UAV0_x.back(),2)+pow(UAV1_y.back()-UAV0_y.back(),2));//做一个取代
        //double d_old=sqrt(pow(UAV1_x.front()-UAV0_x.front(),2)+pow(UAV1_y.front()-UAV0_y.front(),2));
        double d_now=sqrt(abs(pow(distance.at(1),2)));
        double d_old=sqrt(abs(pow(distance.at(0),2)));

        
        // phi_xy(0,0)=(UAV1_x.back()-UAV1_x.front())-(UAV0_phi_x);
        //phi_xy(1,0)=(UAV1_y.back()-UAV1_y.front())-(UAV0_phi_y);
        phi_xy(0,0)=car_phi_x;
        phi_xy(1,0)=car_phi_y;
ROS_INFO("vel_get_seccess%f,%f",phi_xy(0,0) ,phi_xy(1,0) );
        // if (phi_xy(0,0)==0)
        // { phi_xy(0,0)=0.00001;
        //     phi_xy(1,0)=0.00001;
        // }
        vk(0,0)=UAV0_phi_x;
        vk(1,0)=UAV0_phi_y;
        //  cout<<"data"<<endl;
        // cout<<q_est<<endl;
        
        cout<<d_now<<endl;
        cout<<d_old<<endl;
        cout<<phi_xy<<endl;
        // cout<<vk<<endl;
        //ROS_INFO("w%f,%f,%f,%f,%f,%f,%f,%f,%f", q_est(0,0),q_est(1,0),d_now,d_old,phi_xy(0,0),phi_xy(1,0),vk(0,0),vk(1,0),k);
        getretu  = velcal(bgamak ,q_est_old, q_est,d_now,d_old,phi_xy,vk,k);
        q_est_old=q_est;
        q_est<<getretu(0,0),getretu(1,0);
        v_est<<getretu(0,1),getretu(1,1);
	    bgamak<< getretu(0,2),getretu(0,3),getretu(1,2),getretu(1,3);
        // cout<<"data2"<<endl;
        cout<<q_est<<endl;
        //ROS_INFO("计算速1度%f,%f",v_est(0,0),v_est(1,0));
        // double x1 = UAV1_x.front()+v_est(0,0)*0.096;
        // double y1 = UAV1_y.front()+v_est(1,0)*0.096;
        //UAV1_x.push(x1);
        // UAV1_y.push(y1);
        ref_velocity_x=v_est(0,0);
        ref_velocity_y=v_est(1,0);
        //get matlab data

        tracker_8::data_get data;
        data.time=int64_t((ros::Time::now().toSec())*1000)-data_start;
        data.dis_now=d_now;
        data.dis_old=d_old;
        data.phi_x=phi_xy(0,0);
        data.phi_y=phi_xy(1,0);
        data.qest_x=getretu(0,0);
        data.qest_y=getretu(1,0);
        data.u_cal_x=getretu(0,1);
        data.u_cal_y=getretu(1,1);
        data.u_real_x=ref_velocity_x;
        data.u_real_y=ref_velocity_y;
        data_matlab.publish(data);

        //
        //pose1.pose.position.x = x1;
        //pose1.pose.position.y = y1;
        geometry_msgs::TwistStamped vel1;
        vel1.twist.linear.x=ref_velocity_x ;
        vel1.twist.linear.y=ref_velocity_y ;
        //vel1.twist.linear.z=ref_velocity_z ;
ROS_INFO("vel_seccess%f,%f",ref_velocity_x ,ref_velocity_y );
        carSetVelPub.publish(vel1);

        // std_msgs::String commitPubMsg;
        // LightFlowStateEstMessage flowMsg;
        // flowMsg.p_inc_x = UAV1_phi_x;
        // flowMsg.p_inc_y = UAV1_phi_y;
        // flowMsg.p_inc_z = UAV1_high_sensor_value;
        // flowMsg.e_x     = getretu(0,0);
        // flowMsg.e_y     = getretu(1,0);
        // commitPubMsg.data = x2struct::X::tojson(flowMsg);
        // _msgToUwbPub.publish(commitPubMsg);
       
        // UAV1_phi_x=0;
        // UAV1_phi_y=0;
        num=0;
        car_phi_x=0;
        car_phi_x=0;
        distance.at(0) = distance.at(1);
        is_uav1_data_ok = false;
    }

                   
                    
                }
            }
        }

    }

}

