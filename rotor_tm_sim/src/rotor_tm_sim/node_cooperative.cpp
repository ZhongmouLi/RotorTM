#include <iostream>
// headers for ros
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
// headers for rotorTM
#include <rotor_tm_msgs/FMCommand.h>

#include <vector>

// headers for rotor_sim class
#include "rotor_tm_sim/lib_cooperative.hpp"



// headers of asio for parallel programming
#include <boost/asio.hpp>


struct RobotState {
    Eigen::Vector3d position;
    Eigen::Vector3d vel= Eigen::Vector3d::Zero();
    Eigen::Vector3d acc= Eigen::Vector3d::Zero();
    Eigen::Quaterniond attitude;        
    Eigen::Vector3d bodyrate = Eigen::Vector3d::Zero();
    Eigen::Vector3d bodyrate_acc = Eigen::Vector3d::Zero();
};



// functions for converting data type
Eigen::Vector3d vector3MsgToEigen(const geometry_msgs::Vector3& msg);
geometry_msgs::Vector3 EigenToVector3Msg(const Eigen::Vector3d& msg);
geometry_msgs::Point EigenToPointMsg(const Eigen::Vector3d& msg);
geometry_msgs::Quaternion EigenQuadnToGeomQuadn(const Eigen::Quaterniond& q);


// controller inputs:
// vector of thrust (scalar) with a size of 4
Eigen::VectorXd v_mavs_thrusts = Eigen::MatrixXd::Constant(4,1,3.0625);
// vector of torque (Eigen vector 3 X1) with a size of 4
std::vector<Eigen::Vector3d> v_mavs_torques(4, Eigen::Vector3d::Zero());


// callback function to take input wrench for mav0
void fmCmdCallback0(const rotor_tm_msgs::FMCommand::ConstPtr& msg)
{
    
    // thrust = vector3MsgToEigen(msg->rlink_thrust);
    // thrust = static_cast<double>(msg->thrust);
    v_mavs_thrusts[0] = static_cast<double>(msg->thrust);

    // torque = vector3MsgToEigen(msg->moments);
    v_mavs_torques[0] = vector3MsgToEigen(msg->moments);
    // ROS_INFO_STREAM("receive input torque in Eigen"<<torque.transpose());
    // ROS_INFO_STREAM("receive input torque "<<msg->moments);
    // ROS_INFO_STREAM("receive input wrench "<< thrust<< " "<<torque.transpose());
}


// callback function to take input wrench for mav1
void fmCmdCallback1(const rotor_tm_msgs::FMCommand::ConstPtr& msg)
{
    
    // thrust = vector3MsgToEigen(msg->rlink_thrust);
    // thrust = static_cast<double>(msg->thrust);
    v_mavs_thrusts[1] = static_cast<double>(msg->thrust);

    // torque = vector3MsgToEigen(msg->moments);
    v_mavs_torques[1] = vector3MsgToEigen(msg->moments);
    // ROS_INFO_STREAM("receive input torque in Eigen"<<torque.transpose());
    // ROS_INFO_STREAM("receive input torque "<<msg->moments);
    // ROS_INFO_STREAM("receive input wrench "<< thrust<< " "<<torque.transpose());
}

// callback function to take input wrench for mav2
void fmCmdCallback2(const rotor_tm_msgs::FMCommand::ConstPtr& msg)
{
    
    // thrust = vector3MsgToEigen(msg->rlink_thrust);
    // thrust = static_cast<double>(msg->thrust);
    v_mavs_thrusts[2] = static_cast<double>(msg->thrust);

    // torque = vector3MsgToEigen(msg->moments);
    v_mavs_torques[2] = vector3MsgToEigen(msg->moments);
    // ROS_INFO_STREAM("receive input torque in Eigen"<<torque.transpose());
    // ROS_INFO_STREAM("receive input torque "<<msg->moments);
    // ROS_INFO_STREAM("receive input wrench "<< thrust<< " "<<torque.transpose());
}

// callback function to take input wrench for mav3
void fmCmdCallback3(const rotor_tm_msgs::FMCommand::ConstPtr& msg)
{
    
    // thrust = vector3MsgToEigen(msg->rlink_thrust);
    // thrust = static_cast<double>(msg->thrust);
    v_mavs_thrusts[3] = static_cast<double>(msg->thrust);

    // torque = vector3MsgToEigen(msg->moments);
    v_mavs_torques[3] = vector3MsgToEigen(msg->moments);
    // ROS_INFO_STREAM("receive input torque in Eigen"<<torque.transpose());
    // ROS_INFO_STREAM("receive input torque "<<msg->moments);
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

    // 3. define subscriber and publisher
    // subscriber to receve input wrench for mavs
    ros::Subscriber fm_mav0_sub = nh.subscribe<rotor_tm_msgs::FMCommand>("/controller_1/dragonfly1/fm_cmd", 1, fmCmdCallback0);
    ros::Subscriber fm_mav1_sub = nh.subscribe<rotor_tm_msgs::FMCommand>("/controller_2/dragonfly2/fm_cmd", 1, fmCmdCallback1);
    ros::Subscriber fm_mav2_sub = nh.subscribe<rotor_tm_msgs::FMCommand>("/controller_3/dragonfly3/fm_cmd", 1, fmCmdCallback2);
    ros::Subscriber fm_mav3_sub = nh.subscribe<rotor_tm_msgs::FMCommand>("/controller_4/dragonfly4/fm_cmd", 1, fmCmdCallback3);

    // publisher to send dyanmic simulation output to mavs and payload
    // mavs
    ros::Publisher mav0_odom_pub = nh.advertise<nav_msgs::Odometry>("/dragonfly1/odom", 1);
    ros::Publisher mav1_odom_pub = nh.advertise<nav_msgs::Odometry>("/dragonfly2/odom", 1);
    ros::Publisher mav2_odom_pub = nh.advertise<nav_msgs::Odometry>("/dragonfly3/odom", 1);
    ros::Publisher mav3_odom_pub = nh.advertise<nav_msgs::Odometry>("/dragonfly4/odom", 1);
    // payload
    ros::Publisher payload_odom_pub = nh.advertise<nav_msgs::Odometry>("/payload/odom", 1);



    // TOModifyLater
    // 4. paras for mavs and payload
    // mav
    // snapdragonfly
    const double mav_mass = 0.25;
    Eigen::Matrix3d mav_inertia = Eigen::Matrix3d::Identity(3,3);
    mav_inertia(0,0)= 0.000601;
    mav_inertia(1,1)= 0.000589;
    mav_inertia(2,2)= 0.001076; 

    // payload
    // load_params/fedex_box_payload.yaml
    // 1. set payload param
    double payload_mass = 0.250;
    Eigen::Matrix3d payload_inertia = Eigen::Matrix3d::Identity(3,3);    
    payload_inertia(0,0)= 0.000601;
    payload_inertia(1,1)= 0.000589;
    payload_inertia(2,2)= 0.01076; 
    // 2. set cable length
    const double cable_length = 0.5;
 
    // attach point
    const std::vector<Eigen::Vector3d> v_attach_point_post{{0.3048, -0.3048, 0.2286},{0.3048, 0.3048, 0.2286}, {-0.3048, 0.3048, 0.2286}, {-0.3048,-0.3048, 0.2286}};


    // 5. define quadrotor simulator
    // set int step size to be same as ros step
    const double dt = 1.0/ROS_FREQ;

    
    // 6. define instance of single quadrotor +  a point mass payload 
    // auto ptr_rotorTM = std::make_shared<rotorTMQuadrotorPointMass>(drone_mass, m_inertia, payload_mass, cable_length, dt);
    std::shared_ptr<Cooperative> ptr_Cooperative = std::make_shared<Cooperative>(v_attach_point_post, mav_mass, mav_inertia, cable_length, payload_mass, payload_inertia, dt);

    // 7. vars to get from quadrotro simulators to output messages
    std::vector<RobotState> v_mavs_states;
    v_mavs_states.reserve(4);

    // vars to get payload states
    // Eigen::Vector3d payload_position;
    // Eigen::Vector3d payload_vel;
    // Eigen::Vector3d payload_bodyrate(0,0,0);
    // Eigen::Quaterniond payload_attitude(1,0,0,0);    
    RobotState payload_state;
   
    nav_msgs::Odometry mav0_odom_msg;
    mav0_odom_msg.header.frame_id = "odom";
    mav0_odom_msg.child_frame_id = "base_link";

    nav_msgs::Odometry mav1_odom_msg;
    mav1_odom_msg.header.frame_id = "odom";
    mav1_odom_msg.child_frame_id = "base_link";

    nav_msgs::Odometry mav2_odom_msg;
    mav2_odom_msg.header.frame_id = "odom";
    mav2_odom_msg.child_frame_id = "base_link";

    nav_msgs::Odometry mav3_odom_msg;
    mav3_odom_msg.header.frame_id = "odom";
    mav3_odom_msg.child_frame_id = "base_link";

    nav_msgs::Odometry payload_odom_msg;
    payload_odom_msg.header.frame_id = "odom";
    payload_odom_msg.child_frame_id = "base_link";


    // initial condition
    // set initial position for robot
    // payload's initial position is at origin
    // drone is above payload by cable length
    ptr_Cooperative->SetPayloadInitPost();


    while (ros::ok())
    {   


        ROS_INFO_STREAM("ROS Function Point 1 in loop");

        // step 1 input control sigals for mavs
        ptr_Cooperative->InputControllerInput4MAVs(v_mavs_thrusts, v_mavs_torques);

        // step compute interation wrenches and vars for MAVs and payload
        ptr_Cooperative->ComputeInteractWrenches();

        // call one step dynamic simulation for MAVs and payload
        ptr_Cooperative->DoOneStepInt4Robots();
        // ROS_INFO_STREAM("input thrust is "<< thrust);


        // step 3.1 obtain mav states
        std::vector<UAVCable> v_drone_cable = ptr_Cooperative->v_drone_cable_;
        for (size_t i = 0; i < 4; i++)
        {  
            v_drone_cable.at(0).mav_.GetPosition(v_mavs_states[i].position);

            v_drone_cable.at(0).mav_.GetVel(v_mavs_states[i].position);    

            // v_drone_cable.at(0).mav_.GetAcc(v_mavs_states[i].acc);

            v_drone_cable.at(0).mav_.GetBodyrate(v_mavs_states[i].bodyrate);

            v_drone_cable.at(0).mav_.GetAttitude(v_mavs_states[i].attitude);
        
            // v_drone_cable.at(0).mav_.GetBodyRateAcc(v_mavs_states[i].bodyrate_acc);
        }
        

        // step 3.2 obtain payload states
       ptr_Cooperative->payload_.GetPosition(payload_state.position);
       ptr_Cooperative->payload_.GetVel(payload_state.vel);    
       ptr_Cooperative->payload_.GetBodyrate(payload_state.bodyrate);  
       ptr_Cooperative->payload_.GetAttitude(payload_state.attitude);   


    
        // setp 5. Publish simulation results to topics
        // setp 5.1 assigen drone state infor (position, vel, attitude, bodyrate) to odom_msg
        mav0_odom_msg.header.stamp = ros::Time::now();
        mav0_odom_msg.pose.pose.position = EigenToPointMsg(v_mavs_states[0].position);
        mav0_odom_msg.pose.pose.orientation = EigenQuadnToGeomQuadn(v_mavs_states[0].attitude);
        mav0_odom_msg.twist.twist.linear = EigenToVector3Msg(v_mavs_states[0].vel);
        mav0_odom_msg.twist.twist.angular = EigenToVector3Msg(v_mavs_states[0].bodyrate);

        mav1_odom_msg.header.stamp = ros::Time::now();
        mav1_odom_msg.pose.pose.position = EigenToPointMsg(v_mavs_states[1].position);
        mav1_odom_msg.pose.pose.orientation = EigenQuadnToGeomQuadn(v_mavs_states[1].attitude);
        mav1_odom_msg.twist.twist.linear = EigenToVector3Msg(v_mavs_states[1].vel);
        mav1_odom_msg.twist.twist.angular = EigenToVector3Msg(v_mavs_states[1].bodyrate);

        mav2_odom_msg.header.stamp = ros::Time::now();
        mav2_odom_msg.pose.pose.position = EigenToPointMsg(v_mavs_states[2].position);
        mav2_odom_msg.pose.pose.orientation = EigenQuadnToGeomQuadn(v_mavs_states[2].attitude);
        mav2_odom_msg.twist.twist.linear = EigenToVector3Msg(v_mavs_states[2].vel);
        mav2_odom_msg.twist.twist.angular = EigenToVector3Msg(v_mavs_states[2].bodyrate);

        mav3_odom_msg.header.stamp = ros::Time::now();
        mav3_odom_msg.pose.pose.position = EigenToPointMsg(v_mavs_states[3].position);
        mav3_odom_msg.pose.pose.orientation = EigenQuadnToGeomQuadn(v_mavs_states[3].attitude);
        mav3_odom_msg.twist.twist.linear = EigenToVector3Msg(v_mavs_states[3].vel);
        mav3_odom_msg.twist.twist.angular = EigenToVector3Msg(v_mavs_states[3].bodyrate);                        

        // payload
        payload_odom_msg.header.stamp = ros::Time::now();

        payload_odom_msg.pose.pose.position = EigenToPointMsg(payload_state.position);
        payload_odom_msg.pose.pose.orientation = EigenQuadnToGeomQuadn(payload_state.attitude);
        payload_odom_msg.twist.twist.linear = EigenToVector3Msg(payload_state.vel);
        payload_odom_msg.twist.twist.angular = EigenToVector3Msg(payload_state.bodyrate);

        // setp 5.publish odom_msgs of drone and payload
        mav0_odom_pub.publish(mav0_odom_msg);    
        mav1_odom_pub.publish(mav1_odom_msg);   
        mav2_odom_pub.publish(mav2_odom_msg);   
        mav3_odom_pub.publish(mav3_odom_msg);   
        payload_odom_pub.publish(payload_odom_msg);  

        // ROS_INFO_STREAM("payload position "<< payload_position.transpose());

        // run ros loop
        ros::spinOnce();

        loop_rate.sleep();   
    }
    

}



























Eigen::Vector3d vector3MsgToEigen(const geometry_msgs::Vector3& msg) {
    return Eigen::Vector3d(static_cast<double>(msg.x), static_cast<double>(msg.y), static_cast<double>(msg.z));
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
