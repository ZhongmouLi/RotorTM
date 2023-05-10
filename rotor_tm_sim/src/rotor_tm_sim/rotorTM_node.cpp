#include <iostream>

#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <rotor_tm_msgs/FMCommand.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>

// #include "rotor_tm_sim/lib_ros_simulator.hpp"
#include "rotor_tm_sim/lib_quadrotor_dynamic_simulator.hpp"

// functions for converting data type
Eigen::Vector3d vector3MsgToEigen(const geometry_msgs::Vector3& msg);
geometry_msgs::Vector3 EigenToVector3Msg(const Eigen::Vector3d& msg);
geometry_msgs::Point EigenToPointMsg(const Eigen::Vector3d& msg);
geometry_msgs::Quaternion EigenQuadnToGeomQuadn(const Eigen::Quaterniond& q);



// input for quadrotor dynamic simulator
// thrust: norm of thrust force
static double thrust;       // static Eigen::Vector3d thrust;
// torque : torque vector in body frame
static Eigen::Vector3d torque;


// callback function to take input wrench
void fmCmdCallback(const rotor_tm_msgs::FMCommand::ConstPtr& msg)
{
    
    // thrust = vector3MsgToEigen(msg->rlink_thrust);
    thrust = static_cast<double>(msg->thrust);
    torque = vector3MsgToEigen(msg->moments);
    // ROS_INFO_STREAM("receive input wrench "<< thrust<< " "<<torque.transpose());
}




int main(int argc, char** argv)
{
    // 1. init ros nodes
    ros::init(argc, argv, "rotorTM_sim");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    // 2.define ros frequency to be 100
    const int ROS_FREQ = 100;
    ros::Rate loop_rate(ROS_FREQ);

    // 3. define Subscriber to receve input wrench and Publisher to send odom
    ros::Subscriber fm_command_sub = nh.subscribe<rotor_tm_msgs::FMCommand>("reference/fm_cmd", 1, fmCmdCallback);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("outputs/odom", 1);

    // 4. set drone para from ros para
    double mass;
    double Ixx, Iyy, Izz;
    // using arg in lanch file
    // nh_private.param<double>("drone_mass", mass, 1);
    // nh_private.param<double>("drone_Ixx", Ixx, 0.01);
    // nh_private.param<double>("drone_Iyy", Iyy, 0.01);
    // nh_private.param<double>("drone_Izz", Izz, 0.02);
    // using yaml file rotor_tm_config)/config/uav_params/race.yaml
    nh_private.getParam("/mass", mass);
    nh_private.getParam("/inertia/Ixx", Ixx);
    nh_private.getParam("/inertia/Iyy", Iyy);
    nh_private.getParam("/inertia/Izz", Izz);

    // ROS_INFO_STREAM("input mass "<< mass << "input Ixx "<< Ixx << "input Iyy "<< Iyy << "input Izz "<< Izz);

    Eigen::Matrix3d m_inertia = Eigen::Matrix3d::Identity(3,3);
    // m_inertia(0,0)=0.01;
    // m_inertia(1,1)=0.01;
    // m_inertia(2,2)=0.02;      
    m_inertia(0,0)=Ixx;
    m_inertia(1,1)=Iyy;
    m_inertia(2,2)=Izz; 


    // 5. define quadrotor simulator
    // set int step size to be same as ros step
    const double dt = 1.0/ROS_FREQ;

    
    // instance quadrotor simulator with mass, initeria and int size
    std::unique_ptr<QuadrotorDynamicSimulator> ptr_drone = std::make_unique<QuadrotorDynamicSimulator>(mass, m_inertia, dt);

    //6. odom_msg output message 
    nav_msgs::Odometry odom_msg;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";


    // 7. vars to get from quadrotro simulators to output messages
    Eigen::Vector3d mav_position;
    Eigen::Vector3d mav_vel;
    Eigen::Vector3d mav_bodyrate;
    Eigen::Quaterniond mav_attitude;

    // 8. quadrotor dynamic simulation
    // (1) quadrotor's initial state (position, vel, attitude, bodyrate)  is set as 0s
    // (2) simulation step is defined in ptr_drone
    // (3) bug: callback result is received after several running of int loops
    //     TODO: for a single drone, it can be put all the loop contents into callback function

    while (ros::ok())
    {   
        ros::spinOnce();

        loop_rate.sleep();   

        // step 1. input thrust and torque to quadrotor    
        // ROS_INFO_STREAM("input thrust "<< thrust);
        // ROS_INFO_STREAM("input torque "<< torque.transpose());

        // ptr_drone->inputThurstForce(thrust);
        ptr_drone->inputThurst(thrust);
        ptr_drone->inputTorque(torque);

        // step 2. do one step int with the step size being dt
        //         the obtained drone state (position, vel, attitude, bodyrate) is saved for the next int.
        ptr_drone->doOneStepInt();

        // setp 3. get drone position (Eigen::Vector3d), vel(Eigen::Vector3d), atttude (Eigen::Quaterniond), bodyrate (Eigen::Vector3d)
        ptr_drone->getPosition(mav_position);
        // ROS_INFO_STREAM("drone position is" << mav_position.transpose());

        ptr_drone->getVel(mav_vel); 
        // ROS_INFO_STREAM("drone vel is" << mav_vel.transpose());

        ptr_drone->getAttitude(mav_attitude);  
        // ROS_INFO_STREAM("drone attitude is" << mav_attitude.coeffs());

        ptr_drone->getBodyrate(mav_bodyrate);
        //  ROS_INFO_STREAM("drone body rate is" << mav_bodyrate.transpose());

        // setp 4. assigen drone state infor (position, vel, attitude, bodyrate) to odom_msg
        odom_msg.header.stamp = ros::Time::now();

        odom_msg.pose.pose.position = EigenToPointMsg(mav_position);
        odom_msg.pose.pose.orientation = EigenQuadnToGeomQuadn(mav_attitude);
        odom_msg.twist.twist.linear = EigenToVector3Msg(mav_vel);
        odom_msg.twist.twist.angular = EigenToVector3Msg(mav_bodyrate);

        // setp 5.publish odom_msg
        odom_pub.publish(odom_msg);     
    }
    

}

Eigen::Vector3d vector3MsgToEigen(const geometry_msgs::Vector3& msg) {
    return Eigen::Vector3d(msg.x, msg.y, msg.z);
}

geometry_msgs::Vector3 EigenToVector3Msg(const Eigen::Vector3d& eigen_vector)
{
      geometry_msgs::Vector3 msg_v3;
      msg_v3.x = eigen_vector.x();
      msg_v3.y = eigen_vector.y();
      msg_v3.z = eigen_vector.z();
      return msg_v3;
};

geometry_msgs::Point EigenToPointMsg(const Eigen::Vector3d& eigen_vector)
{
    geometry_msgs::Point msg_point;
    msg_point.x = eigen_vector.x();
    msg_point.y = eigen_vector.y();
    msg_point.z = eigen_vector.z();
    return msg_point;
};

geometry_msgs::Quaternion EigenQuadnToGeomQuadn(const Eigen::Quaterniond& q)
{
    geometry_msgs::Quaternion q_geom;
    q_geom.w = q.w();
    q_geom.x = q.x();
    q_geom.y = q.y();
    q_geom.z = q.z();
    return q_geom;
}
