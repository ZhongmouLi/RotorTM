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
// #include "rotor_tm_sim/lib_ros_simulator.hpp"
#include "rotor_tm_sim/lib_quadrotor_pointmass.hpp"



// headers of asio for parallel programming
#include <boost/asio.hpp>


// functions for converting data type
Eigen::Vector3d vector3MsgToEigen(const geometry_msgs::Vector3& msg);
geometry_msgs::Vector3 EigenToVector3Msg(const Eigen::Vector3d& msg);
geometry_msgs::Point EigenToPointMsg(const Eigen::Vector3d& msg);
geometry_msgs::Quaternion EigenQuadnToGeomQuadn(const Eigen::Quaterniond& q);



// input for quadrotor dynamic simulator
// thrust: norm of thrust force
static double thrust = (0.07 + 0.95)* 9.8;       // static Eigen::Vector3d thrust;
// torque : torque vector in body frame
static Eigen::Vector3d torque(0,0,0);


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
    ros::Subscriber fm_command_sub = nh.subscribe<rotor_tm_msgs::FMCommand>("/controller_1/dragonfly1/fm_cmd", 1, fmCmdCallback);
    ros::Publisher drone_odom_pub = nh.advertise<nav_msgs::Odometry>("/dragonfly1/odom", 1);
    ros::Publisher payload_odom_pub = nh.advertise<nav_msgs::Odometry>("/payload/odom", 1);

    // 4. set drone para from ros para
    double drone_mass;
    double Ixx, Iyy, Izz;
    // using arg in lanch file
    // nh_private.param<double>("drone_mass", mass, 1);
    // nh_private.param<double>("drone_Ixx", Ixx, 0.01);
    // nh_private.param<double>("drone_Iyy", Iyy, 0.01);
    // nh_private.param<double>("drone_Izz", Izz, 0.02);
    // using yaml file rotor_tm_config)/config/uav_params/race.yaml
    nh_private.getParam("/mass", drone_mass);
    nh_private.getParam("/inertia/Ixx", Ixx);
    nh_private.getParam("/inertia/Iyy", Iyy);
    nh_private.getParam("/inertia/Izz", Izz);

    /*TO DO*/
    // 1. set payload param
    double payload_mass;
    // nh_private.getParam("/inertia/Izz", Izz);
    payload_mass = 0.07;
    // 2. set cable length
    std::vector<double> cable_length_list;
    double cable_length;
    // cable_length = 1.0;
    nh.getParam("/cable_length", cable_length_list);

    cable_length = cable_length_list.at(0);
    ROS_INFO_STREAM("cable length is " << cable_length);

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

    
    // 6. define instance of single quadrotor +  a point mass payload 
    auto ptr_rotorTM = std::make_shared<rotorTMQuadrotorPointMass>(drone_mass, m_inertia, payload_mass, cable_length, dt);

    // 7. vars to get from quadrotro simulators to output messages
    Eigen::Vector3d mav_position;
    Eigen::Vector3d mav_vel;
    Eigen::Vector3d mav_bodyrate;
    Eigen::Quaterniond mav_attitude;

    // vars to get payload states
    Eigen::Vector3d payload_position;
    Eigen::Vector3d payload_vel;
    Eigen::Vector3d payload_bodyrate(0,0,0);
    Eigen::Quaterniond payload_attitude(1,0,0,0);    

   
    nav_msgs::Odometry drone_odom_msg;
    drone_odom_msg.header.frame_id = "odom";
    drone_odom_msg.child_frame_id = "base_link";

    nav_msgs::Odometry payload_odom_msg;
    payload_odom_msg.header.frame_id = "odom";
    payload_odom_msg.child_frame_id = "base_link";


    // initial condition
    // set initial position for robot
    // payload's initial position is at origin
    // drone is above payload by cable length
    ptr_rotorTM->setInitPost(Eigen::Vector3d::Zero());

    while (ros::ok())
    {   
        // run ros loop
        ros::spinOnce();

        loop_rate.sleep();   

        // step 1 input control sigal
        ptr_rotorTM->inputMAVThrust(thrust);

        ptr_rotorTM->inputMAVTorque(torque);


        // step 2 call integration
        ptr_rotorTM->doOneStepint();

        // step 3 obtain odom infor from robot
        // drone
        ptr_rotorTM->quadrotor->getPosition(mav_position);
        ptr_rotorTM->quadrotor->getVel(mav_vel);
        ptr_rotorTM->quadrotor->getAttitude(mav_attitude);
        ptr_rotorTM->quadrotor->getBodyrate(mav_bodyrate);       

        // payload
        ptr_rotorTM->pm_payload->getPosition(payload_position); 
        ptr_rotorTM->pm_payload->getVel(payload_vel); 
        
       
        // setp 5. Publish simulation results to topics
        // setp 5.1 assigen drone state infor (position, vel, attitude, bodyrate) to odom_msg
        drone_odom_msg.header.stamp = ros::Time::now();

        drone_odom_msg.pose.pose.position = EigenToPointMsg(mav_position);
        drone_odom_msg.pose.pose.orientation = EigenQuadnToGeomQuadn(mav_attitude);
        drone_odom_msg.twist.twist.linear = EigenToVector3Msg(mav_vel);
        drone_odom_msg.twist.twist.angular = EigenToVector3Msg(mav_bodyrate);

        // payload
        payload_odom_msg.header.stamp = ros::Time::now();

        payload_odom_msg.pose.pose.position = EigenToPointMsg(payload_position);
        payload_odom_msg.pose.pose.orientation = EigenQuadnToGeomQuadn(payload_attitude);
        payload_odom_msg.twist.twist.linear = EigenToVector3Msg(payload_vel);
        payload_odom_msg.twist.twist.angular = EigenToVector3Msg(payload_bodyrate);

        // setp 5.publish odom_msgs of drone and payload
        drone_odom_pub.publish(drone_odom_msg);    
        payload_odom_pub.publish(payload_odom_msg);  
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



    // // instance quadrotor simulator with mass, initeria and int size
    // std::shared_ptr<QuadrotorDynamicSimulator> ptr_drone = std::make_shared<QuadrotorDynamicSimulator>(mass, m_inertia, dt);

    // // instance payload with mass
    // std::shared_ptr<PointMassDynamicSimulator> ptr_payload = std::make_shared<PointMassDynamicSimulator>(payload_mass, dt);


    // // creat a thread pool from boost/asio 
    // // it assigns each quadrotor and payload to an indepdendent thread such that they are in parallel
    // // arg here in pool_rotorTM(2) means create a pool with 2 threads (one for quadrotor and one for payload)
    // // in the future, set this arg to be the no of objs of rotorTM
    // boost::asio::thread_pool pool_rotorTM(2);

    // auto quadrotor_func = std::bind(&QuadrotorDynamicSimulator::doOneStepInt, ptr_drone);
    // auto payload_func = std::bind(&PointMassDynamicSimulator::doOneStepInt, ptr_payload);

    // //6. odom_msg output message 
    // nav_msgs::Odometry odom_msg;
    // odom_msg.header.frame_id = "odom";
    // odom_msg.child_frame_id = "base_link";


 // vars to get cable states
    // xi is a unit vector from the robot’s center of mass to the payload.
    // Eigen::Vector3d xi;
    // Eigen::Vector3d xi_dot; // derivative of xi
    // Eigen::Vector3d xi_omega; 

    // // tension force
    // Eigen::Vector3d tension;
    // // quadrotor thrust force in world frame
    // Eigen::Vector3d thrust_force;
    // // centrifugal force
    // double centrif;



        // // step 1 obtain xi, xi_dot and xi_omega

        // // step 1.1. get positions and vels of quadrotors and payload's position from quadrotor and pointmass
        // // note each state consists of pos(Eigen::Vector3d), vel(Eigen::Vector3d), atttude (Eigen::Quaterniond), bodyrate (Eigen::Vector3d)
        // ptr_drone->getPosition(mav_position);
        // ptr_payload->getPosition(payload_position);

        // ptr_drone->getVel(mav_vel); 
        // ptr_payload->getVel(payload_vel);

        // // step 1.2 compute xi and xi_dot
        // xi = (payload_position - mav_position)/cable_length;
        // xi_dot = (payload_vel - mav_vel)/cable_length;

        // // step 1.3 compute xi_omega
        // xi_omega = xi.cross(xi_dot);


        // // step 2 compute thrust force expressed in world frame, centrifugal force and tension vector

        // // step 2.1 thrust force
        // ptr_drone->getAttitude(mav_attitude); 
        // thrust_force = thrust * (mav_attitude.toRotationMatrix()*Eigen::Vector3d::UnitZ());

        // // step 2.2 centrifugal force
        // // note transpose is not necessary for Eigen vector
        // centrif = xi_omega.dot(xi_omega) * payload_mass * cable_length;

   
        // // step 2.3 tension vector
        // // note that use -xi.dot(thrust_force) instead of -xi.transpose * thrust_force, as the second means matrix manipulation for a 1X3 vector and a 3X1 vector and it is the same to apply` dot product to two vectors in Eigen
        // tension = payload_mass * (-xi.dot(thrust_force) + centrif) * xi / (mass + payload_mass);
        // // tension = payload_mass * (-xi.transpose() * thrust_force + centrif) * xi / (mass + payload_mass);


        // // step 3. dynamic simu for quadrotor + pointmass using parallel programming

        // // step 3.1 input force and torque for quadrotor and pointmass
        // // Note that quadrotor attitude dynamics is decoupled from  payload position and attitude dynamics,  
        // // payload is a pointmass object so its attitude dynamics is not considered.

        // // quadrotor
        // ptr_drone->inputForce(thrust_force+tension);
        // ptr_drone->inputTorque(torque);
        // // point mass
        // ptr_payload->inputForce(-tension);

        // // step 3.2 submit dynamic simulator of quadrotors and payload to the thread pool
        // //          it does one step int for quadrotor and payload with the step size being dt and orginal way to call one step int: ptr_drone->doOneStepInt()
        // //          for each instance, the obtained state (position, vel, attitude, bodyrate) is saved for the next int.
        // //         parallel programming is implemented using a thread pool; quadrotor_func (binded with ptr_drone->doOneStepInt()) and payload_func (binded with ptr_payload->doOneStepInt()) is submitted into this pool
        // boost::asio::post(pool_rotorTM, quadrotor_func);
        // boost::asio::post(pool_rotorTM, payload_func);
        
        
        // // step 3.3 wait for all obj (quadrotors +  payload) to finish their dynamic int
        // pool_rotorTM.join();


        // // setp 4. Dynamic modification on robot and payload's vel caused by collision
        // // 1. determine tension of cable
        //     // compute distance between quadrotor and payload
        // double distance = (mav_position - payload_position).norm();

        // //  ## state integration
        // //             if self.cable_is_slack:
        // //                 print("Cable is slack")
        // //                 #print(x)
        // //                 sol = scipy.integrate.solve_ivp(self.hybrid_ptmass_pl_transportationEOM, t_span, x, method= 'RK45', t_eval=t_span, events=ptmassslackToTaut)
        // //             else:
        // //                 #print("Cable is taut")
        // //                 #print(x)
        // //                 sol = scipy.integrate.solve_ivp(self.hybrid_ptmass_pl_transportationEOM, t_span, x, method= 'RK45', t_eval=t_span, events=ptmasstautToSlack)
                        
        // /*TO DO*/
        // // 2. collision from RotorTM developed by Guanrui
        // // TO DO add vel check that is computed with cable_norm_vel = np.transpose(pl_pos - robot_pos) @ (pl_vel - robot_vel)/np.linalg.norm 
        // if(cable_length-1e-03<=distance<=cable_length+1e-03 )
        // {// 
        //     // The cables are in tension, and the system dynamics doesn’t change, continue
        // }
        // else
        // {
        //    // The cables are in tension, and the system dynamics doesn’t change, continue
        //    // payload_vel, robot_vel = collision(robot_vel, payload_vel)     
        //     Eigen::Vector3d cable_dir = (mav_position - payload_position)/distance;

        //     double cable_dir_projmat = cable_dir.dot(cable_dir);

        //     Eigen::Vector3d payload_vel_pre = cable_dir_projmat * payload_vel;

        //     Eigen::Vector3d mav_vel_pre = cable_dir_projmat * mav_vel;

        //     Eigen::Vector3d v = (payload_mass * payload_vel_pre + mass * mav_vel)/ (payload_mass + mass);

        //     // compute new vels
        //     payload_vel = v + payload_vel - payload_vel_pre;
        //     mav_vel = v + mav_vel - mav_vel_pre;

        //     // update vels of quadrotor and payload
        //     ptr_drone->setVel(mav_vel);
        //     ptr_payload->setVel(payload_vel);

        // }