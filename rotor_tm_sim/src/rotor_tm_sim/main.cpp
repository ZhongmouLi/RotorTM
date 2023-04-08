#include <iostream>
// #include <ros/ros.h>
// #include <ros/subscribe_options.h>

#include "rotor_tm_sim/lib_ros_simulator.hpp"

// #include "rotor_tm_sim/lib_quadrotor_dynamic_simulator.hpp"
int main(int argc, char** argv)
{
    ros::init(argc, argv, "rotorTM_sim");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    double mass =1;

    Eigen::Matrix3d m_inertia = Eigen::Matrix3d::Identity(3,3);
    m_inertia(0,0)=0.1;
    m_inertia(1,1)=0.1;
    m_inertia(2,2)=0.2;      

    const double dt = 0.01;
    double t = 3*dt;

    // QuadrotorDynamicSimulator quadrotor_dynmaic(mass, m_inertia, dt);

    ROSQuadrotorDynamicSimulator* rotorTM_simulator = new ROSQuadrotorDynamicSimulator(nh, nh_private, mass, m_inertia, dt);

    ros::spin();

}