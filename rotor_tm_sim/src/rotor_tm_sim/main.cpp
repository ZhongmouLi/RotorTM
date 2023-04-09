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

Eigen::Vector3d vector3MsgToEigen(const geometry_msgs::Vector3& msg);
geometry_msgs::Vector3 EigenToVector3Msg(const Eigen::Vector3d& msg);
geometry_msgs::Point EigenToPointMsg(const Eigen::Vector3d& msg);
geometry_msgs::Quaternion EigenQuadnToGeomQuadn(const Eigen::Quaterniond& q);

static Eigen::Vector3d thrust;
static Eigen::Vector3d torque;



void fmCmdCallback(const rotor_tm_msgs::FMCommand::ConstPtr& msg)
{
    thrust = vector3MsgToEigen(msg->rlink_thrust);
    torque = vector3MsgToEigen(msg->moments);
}




int main(int argc, char** argv)
{
    ros::init(argc, argv, "rotorTM_sim");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");


    const int ROS_FREQ = 100;
    ros::Rate loop_rate(ROS_FREQ);
    ros::Subscriber fm_command_sub = nh.subscribe<rotor_tm_msgs::FMCommand>("reference/fm_cmd", 1, fmCmdCallback);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 1);

    double mass =1;

    Eigen::Matrix3d m_inertia = Eigen::Matrix3d::Identity(3,3);
    m_inertia(0,0)=0.1;
    m_inertia(1,1)=0.1;
    m_inertia(2,2)=0.2;      

    const double dt = 1.0/ROS_FREQ;

    // QuadrotorDynamicSimulator quadrotor_sim(mass, m_inertia, dt);

    std::unique_ptr<QuadrotorDynamicSimulator> ptr = std::make_unique<QuadrotorDynamicSimulator>(mass, m_inertia, dt);

    nav_msgs::Odometry odom_msg;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    Eigen::Vector3d mav_position;
    Eigen::Vector3d mav_vel;
    Eigen::Vector3d mav_bodyrate;
    Eigen::Quaterniond mav_attitude;

    thrust<<1,2,3;
    torque<<0,0,0.1;

    while (ros::ok())
    {       
        // quadrotor_sim.inputThurstForce(thrust);

        // quadrotor_sim.inputTorque(torque);

        // quadrotor_sim.doOneStepInt();

        // quadrotor_sim.getPosition(mav_position);
        ptr->inputThurstForce(thrust);
        ptr->inputTorque(torque);
        ptr->doOneStepInt();
        ptr->getPosition(mav_position);

        
        ROS_INFO_STREAM("drone position is" << mav_position.transpose());

        ptr->getVel(mav_vel); 

        ROS_INFO_STREAM("drone vel is" << mav_vel.transpose());

        ptr->getAttitude(mav_attitude);  
        ROS_INFO_STREAM("drone attitude is" << mav_attitude.coeffs());

        ptr->getBodyrate(mav_bodyrate);
         ROS_INFO_STREAM("drone body rate is" << mav_bodyrate.transpose());

        odom_msg.header.stamp = ros::Time::now();

        odom_msg.pose.pose.position = EigenToPointMsg(mav_position);
        odom_msg.pose.pose.orientation = EigenQuadnToGeomQuadn(mav_attitude);
        odom_msg.twist.twist.linear = EigenToVector3Msg(mav_vel);
        odom_msg.twist.twist.angular = EigenToVector3Msg(mav_bodyrate);

        odom_pub.publish(odom_msg);

        ros::spinOnce();

        loop_rate.sleep();        
    }
    

    // ROSQuadrotorDynamicSimulator* rotorTM_simulator = new ROSQuadrotorDynamicSimulator(nh, nh_private, mass, m_inertia, dt);

    // 

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
