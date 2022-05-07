#include "eigen3/Eigen/Dense"
#include <math.h>
#include <algorithm>

#include<string.h>
#include <ros/ros.h>

using namespace std;

//  Eigen::Matrix<double,2,2>   velcal( Eigen::Matrix<double,2,1>  q_est_old,double d_now,double  d_old, Eigen::Matrix<double,2,1>  phi_xy,Eigen::Matrix<double,2,1> vk,int k)
// {
//         double zeta = 0.5*(pow(d_now,2)-pow(d_old,2)- phi_xy.squaredNorm());
//         //ROS_INFO("计算zeta%f",d_now.square()-d_old.square());
//         double eps =zeta-phi_xy.transpose() *q_est_old;
//             //ROS_INFO("计算eps%f",eps);
//         double r=0.50;
//         double Gama = 2.0;
//         double v=1.5743;
//         double pi=3.1415926;
//          double U=0.25;
//          double B=2.5;
//          double tau=0.096;
//          //double tau=0.02;
//         // double tau=0.12;
//          Eigen::Matrix<double,2,1> qS1(-2.0,-2.0);
//         Eigen::Matrix<double,2,1>  q_est= q_est_old + phi_xy + (Gama*phi_xy*eps)/(max(v,(Gama*phi_xy).squaredNorm())/v);
//          Eigen::Matrix<double,2,1>  sigma;
//          sigma(0,0)=r*(1.5*sin(k*pi/48.0));   
//          sigma(1,0)= r*(-1.5*(3.0*sin(k*pi/24.0))/4.0);
//          Eigen::Matrix<double,2,1> detasigma;
//          detasigma(0,0)= r*(1.5*sin(k*pi/48.0)-1.5*sin((k-1.0)*pi/48.0));
//          detasigma(1,0)= r*(-1.5*(3.0*sin(k*pi/24.0))/4.0+1.5*(3.0*sin((k-1.0)*pi/24.0))/4.0);
//          Eigen::Matrix<double,2,1>  u_est = - B*(q_est- qS1 - sigma);
//          //ROS_INFO("计算detasigma%f,%f",detasigma(0,0),detasigma(1,0));
//           Eigen::Matrix<double,2,1> u_op=(U/(max(U,u_est.norm()))*u_est+detasigma/(tau) + vk) ;
//         //   ROS_INFO("估计位置%f,%f",q_est(0,0),q_est(0,1));
//                // ROS_INFO("计算qest%f,%f",q_est(0,0),q_est(1,0));
//         Eigen::Matrix<double,2,2>   Output_Matrix;
//         Output_Matrix <<q_est,u_op;
//         return Output_Matrix;
// }
//static Eigen::Matrix<double,2,2>  bgamak;
 
Eigen::Matrix<double,2,4>   velcal(  Eigen::Matrix<double,2,2>  bgamak,Eigen::Matrix<double,2,1>  q_est_old_old,Eigen::Matrix<double,2,1>  q_est_old,double d_now,double  d_old, Eigen::Matrix<double,2,1>  phi_xy,Eigen::Matrix<double,2,1> vk,int k)
{        
        //static Eigen::Matrix<double,2,1>  q_est_old_old;
        double zeta = 0.5*(pow(d_now,2)-pow(d_old,2)+ phi_xy.squaredNorm());
        //ROS_INFO("计算bgamak%f",bgamak);
        double eps =zeta-phi_xy.transpose() *q_est_old;
            ROS_INFO("计算eps%f",eps);
        //double r=0.50;
        //double Gama = 2.0;
       // double v=1.5743;
        double pi=3.1415926;
         double U=0.45;
         double B=5.5;
         //double tau=0.096;
         float  gamaf=0.95;
         double mu=0.95;
         Eigen::Matrix<double,2,2> eye;
         eye << 1,0,0,1;
         Eigen::Matrix<double,2,2> bgamak1;
        bgamak1=(bgamak-(bgamak*phi_xy*phi_xy.transpose()*bgamak)/(gamaf+(phi_xy.transpose()*bgamak*phi_xy)))/gamaf;
         bgamak=((bgamak1).inverse()+((1-gamaf)*mu/gamaf)*eye ).inverse();
         cout<<bgamak<<endl;
         //double tau=0.02;
        // double tau=0.12;
        double thea=0;
        Eigen::Matrix<double,2,2> rathea;
        rathea << cos(thea),-sin(thea),sin(thea),cos(thea);
         Eigen::Matrix<double,2,1> qS1;
        Eigen::Matrix<double,2,1>  q_est= q_est_old + (bgamak1*phi_xy*eps)+gamaf*mu*bgamak1*(q_est_old-q_est_old_old);
        q_est_old_old=q_est_old;
        if (q_est(1, 0)< (-4.5)){
            qS1<<0,-4.5;
           }
        else if (q_est(1, 0)>4.5){
            qS1<<(0,5);
        }
        else {
            qS1<<0,q_est(1, 0)+1;
            qS1=rathea*qS1;
        }
         Eigen::Matrix<double,2,1>  sigma;
         sigma(0,0)=(sin(k*pi/12));   
         sigma(1,0)= (cos(k*pi/12));
        // Eigen::Matrix<double,2,1> detasigma;
         //detasigma(0,0)= r*(1.5*sin(k*pi/48.0)-1.5*sin((k-1.0)*pi/48.0));
         //detasigma(1,0)= r*(-1.5*(3.0*sin(k*pi/24.0))/4.0+1.5*(3.0*sin((k-1.0)*pi/24.0))/4.0);
        // Eigen::Matrix<double,2,1>  u_est = - B*(q_est- qS1 - 0.5*sigma)-0.1*(q_est_old-q_est);
Eigen::Matrix<double,2,1>  u_est = - B*(q_est );
         //ROS_INFO("计算detasigma%f,%f",detasigma(0,0),detasigma(1,0));
          Eigen::Matrix<double,2,1> u_op=u_est*U/(max(U,u_est.norm())) ;
        //   ROS_INFO("估计位置%f,%f",q_est(0,0),q_est(0,1));
               // ROS_INFO("计算qest%f,%f",q_est(0,0),q_est(1,0));
        Eigen::Matrix<double,2,4>   Output_Matrix;
       // Eigen::Matrix<double,2,2>   Output_bgamak;
        Output_Matrix <<q_est,u_op,bgamak;
        //Output_bgamak << bgamak;
        return Output_Matrix ;
}
// int mainm(int argc, char **argv)
// {
// //         Eigen::Matrix<double,2,1>  arg1(0.0073,-0.016);
// //         double arg2=3.7136;
// //         double agr3=3.6222;
// //         Eigen::Matrix<double,2,1> arg4(0.0666,0.1379);
// //         Eigen::Matrix<double,2,1>arg5(0.0038,0.0296);
// //         int arg6=8;
// //       //  cout<<Eigen::Matrix<double,2,1>   velcal( arg1,arg2,arg3,arg4,arg5,arg6)<<endl;
// //         ROS_INFO("vealcapp");
//         return 0;
// }
