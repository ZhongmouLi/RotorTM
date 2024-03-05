#include <iostream>
#include <string>

#include <ros/ros.h>
#include <gtest/gtest.h>
#include "rotor_tm_sim/lib_cooperative.hpp"

#include <ros/subscribe_options.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
// headers for rotorTM
#include <rotor_tm_msgs/FMCommand.h>

#include <vector>
#include <array>




Eigen::Vector3d vector3MsgToEigen(const geometry_msgs::Vector3& msg);
geometry_msgs::Vector3 EigenToVector3Msg(const Eigen::Vector3d& msg);
geometry_msgs::Point EigenToPointMsg(const Eigen::Vector3d& msg);
geometry_msgs::Quaternion EigenQuadnToGeomQuadn(const Eigen::Quaterniond& q);



class ROSCooperative : public ::testing::Test
{
public:

ROSCooperative(){
    double mav_mass =1;
    double payload_mass = 1.5;
    Eigen::Matrix3d mav_inertia = Eigen::Matrix3d::Identity(3,3);
    Eigen::Matrix3d payload_inertia = Eigen::Matrix3d::Identity(3,3);
    double cable_length =1;
    double step_size = 0.01;
    std::vector<Eigen::Vector3d> v_attach_point_post{{1,1,0},{-1,1,0}, {-1,-1,0}, {1,-1,0}};

    ptr_Cooperative = std::make_shared<Cooperative>(v_attach_point_post, mav_mass, mav_inertia, cable_length, payload_mass, payload_inertia, step_size);
}

~ROSCooperative(){
}

protected:
    std::shared_ptr<Cooperative> ptr_Cooperative;
};


// controller inputs:
// vector of thrust (scalar) with a size of 4
Eigen::VectorXd v_mavs_thrusts = Eigen::MatrixXd::Constant(4,1,3.0625);
// vector of torque (Eigen vector 3 X1) with a size of 4
std::vector<Eigen::Vector3d> v_mavs_torques(4, Eigen::Vector3d::Zero());



// callback function to take input wrench for mav0
void fmCmdCallback0(const rotor_tm_msgs::FMCommand::ConstPtr& msg)
{
    std::cout<<static_cast<double>(msg->thrust)<<std::endl;
    // thrust = vector3MsgToEigen(msg->rlink_thrust);
    // thrust = static_cast<double>(msg->thrust);
    v_mavs_thrusts[0] = static_cast<double>(msg->thrust);

    // torque = vector3MsgToEigen(msg->moments);
    v_mavs_torques[0] = vector3MsgToEigen(msg->moments);
    // ROS_INFO_STREAM("receive input torque in Eigen"<<torque.transpose());
    ROS_INFO_STREAM("mav0 input torque "<<v_mavs_torques[0]);
    ROS_INFO_STREAM("mav0 input thrust "<<  v_mavs_thrusts[0]);
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

    ROS_INFO_STREAM("mav1 input torque "<<v_mavs_torques[1]);
    ROS_INFO_STREAM("mav1 input thrust "<<  v_mavs_thrusts[1]);    
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
    ROS_INFO_STREAM("mav2 input torque "<<v_mavs_torques[2]);
    ROS_INFO_STREAM("mav2 input thrust "<<  v_mavs_thrusts[2]);    
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
    ROS_INFO_STREAM("mav3 input torque "<<v_mavs_torques[3]);
    ROS_INFO_STREAM("mav3 input thrust "<<  v_mavs_thrusts[3]);    
}



// test if gTest is well integrated
TEST_F(ROSCooperative, checkGTest){
    ASSERT_TRUE(true);
}


TEST_F(ROSCooperative, checkGReveive){
    ros::NodeHandle nh;
  
ros::Publisher pub0 = nh.advertise<rotor_tm_msgs::FMCommand>("/controller_1/dragonfly1/fm_cmd", 0);
ros::Publisher pub1 = nh.advertise<rotor_tm_msgs::FMCommand>("/controller_2/dragonfly2/fm_cmd", 0);
ros::Publisher pub2 = nh.advertise<rotor_tm_msgs::FMCommand>("/controller_3/dragonfly3/fm_cmd", 0);
ros::Publisher pub3 = nh.advertise<rotor_tm_msgs::FMCommand>("/controller_4/dragonfly4/fm_cmd", 0);

//   mav_hovering_input.header.stamp = ros::Time::now();  
ros::Subscriber fm_mav0_sub = nh.subscribe<rotor_tm_msgs::FMCommand>("/controller_1/dragonfly1/fm_cmd", 1, fmCmdCallback0);
ros::Subscriber fm_mav1_sub = nh.subscribe<rotor_tm_msgs::FMCommand>("/controller_2/dragonfly2/fm_cmd", 1, fmCmdCallback1);
ros::Subscriber fm_mav2_sub = nh.subscribe<rotor_tm_msgs::FMCommand>("/controller_3/dragonfly3/fm_cmd", 1, fmCmdCallback2);
ros::Subscriber fm_mav3_sub = nh.subscribe<rotor_tm_msgs::FMCommand>("/controller_4/dragonfly4/fm_cmd", 1, fmCmdCallback3); 


    rotor_tm_msgs::FMCommand mav_hovering_input;
    mav_hovering_input.thrust = 3.0625;
    mav_hovering_input.moments.x =0;
    mav_hovering_input.moments.y =0;
    mav_hovering_input.moments.z =0;

    for (size_t i = 0; i < 2; i++)
    {
       pub0.publish(mav_hovering_input);    
       pub1.publish(mav_hovering_input);    
       pub2.publish(mav_hovering_input);    
       pub3.publish(mav_hovering_input);     
       ros::spinOnce();
       std::cout<<i<<std::endl;
    }
    
    
}




int main(int argc, char **argv) {
//   testing::InitGoogleTest(&argc, argv);
//   ros::init(argc, argv, "ros_cooperative_test");
//   ros::NodeHandle nh;
//   return RUN_ALL_TESTS();

  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ros_cooperative_test");

  return RUN_ALL_TESTS();
//   ros::AsyncSpinner spinner(0);
//   spinner.start();

//   pub0.publish(mav_hovering_input);
//   int ret = RUN_ALL_TESTS();
//   spinner.stop();
//   ros::shutdown();
//   return ret;

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
