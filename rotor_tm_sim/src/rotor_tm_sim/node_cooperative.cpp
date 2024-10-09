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
#include <array>
#include <mutex>

// headers for rotor_sim class
#include "rotor_tm_sim/lib_payload.hpp"
#include "rotor_tm_sim/lib_uav_cable.hpp"
#include "rotor_tm_sim/lib_cooperative.hpp"

// headers of asio for parallel programming
#include <boost/asio.hpp>


const size_t num_robots = 4;

int counter_simulation = 0;

// functions for converting data type
Eigen::Vector3d vector3MsgToEigen(const geometry_msgs::Vector3& msg);
geometry_msgs::Vector3 EigenToVector3Msg(const Eigen::Vector3d& msg);
geometry_msgs::Point EigenToPointMsg(const Eigen::Vector3d& msg);
geometry_msgs::Quaternion EigenQuadnToGeomQuadn(const Eigen::Quaterniond& q);
Eigen::Matrix3d GeomQuadnToEigenRotMatrix(const geometry_msgs::Quaternion& q_geom);

double thrust_all = 3.0625 +  0.01;

// controller inputs:
// vector of thrust (scalar) with a size of 4
// Eigen::VectorXd v_mavs_thrusts = Eigen::MatrixXd::Constant(4,1,3.0625);

std::vector<double> v_mavs_thrusts(4, 3.0625);

// vector of torque (Eigen vector 3 X1) with a size of 4
std::vector<Eigen::Vector3d> v_mavs_torques(4, Eigen::Vector3d::Zero());


// std::mutex mavs_mutex;

// callback function to take input wrench for mav0
void fmCmdCallback0(const rotor_tm_msgs::FMCommand::ConstPtr& msg)
{
    // std::lock_guard<std::mutex> lock(mavs_mutex);
    // thrust = vector3MsgToEigen(msg->rlink_thrust);
    // thrust = static_cast<double>(msg->thrust);
    v_mavs_thrusts.at(0) = static_cast<double>(msg->thrust);
    //  v_mavs_thrusts.at(0) = thrust_all;

    // torque = vector3MsgToEigen(msg->moments);
    v_mavs_torques.at(0) = vector3MsgToEigen(msg->moments);

    // v_mavs_torques.at(0) = Eigen::Vector3d::Zero();
    // ROS_INFO_STREAM("receive input torque in Eigen"<<torque.transpose());
    ROS_DEBUG_STREAM("mav0 call back");
    ROS_DEBUG_STREAM("mav0 input torque "<<v_mavs_torques.at(0).transpose());
    ROS_DEBUG_STREAM("mav0 input thrust "<<  v_mavs_thrusts.at(0));

    counter_simulation++;
}


// callback function to take input wrench for mav1
void fmCmdCallback1(const rotor_tm_msgs::FMCommand::ConstPtr& msg)
{
    
    //  std::lock_guard<std::mutex> lock(mavs_mutex);
    // thrust = vector3MsgToEigen(msg->rlink_thrust);
    // thrust = static_cast<double>(msg->thrust);
    v_mavs_thrusts.at(1) = static_cast<double>(msg->thrust);
    // v_mavs_thrusts.at(1) = thrust_all;
    // v_mavs_thrusts.at(1) = v_mavs_thrusts.at(1) + 0.001;

    // torque = vector3MsgToEigen(msg->moments);
    v_mavs_torques.at(1) = vector3MsgToEigen(msg->moments);
    // v_mavs_torques.at(1) = Eigen::Vector3d::Zero();
    // ROS_INFO_STREAM("receive input torque in Eigen"<<torque.transpose());
    // ROS_INFO_STREAM("receive input torque "<<msg->moments);
    // ROS_INFO_STREAM("receive input wrench "<< thrust<< " "<<torque.transpose());

    ROS_DEBUG_STREAM("mav1 input torque "<<v_mavs_torques.at(1).transpose());
    ROS_DEBUG_STREAM("mav1 input thrust "<<  v_mavs_thrusts.at(1));    
}

// callback function to take input wrench for mav2
void fmCmdCallback2(const rotor_tm_msgs::FMCommand::ConstPtr& msg)
{
    //  std::lock_guard<std::mutex> lock(mavs_mutex);
    // thrust = vector3MsgToEigen(msg->rlink_thrust);
    // thrust = static_cast<double>(msg->thrust);
    v_mavs_thrusts.at(2) = static_cast<double>(msg->thrust);
    // v_mavs_thrusts.at(2) = thrust_all;

    // torque = vector3MsgToEigen(msg->moments);
    v_mavs_torques.at(2) = vector3MsgToEigen(msg->moments);
    // v_mavs_torques.at(2) = Eigen::Vector3d::Zero();
    // ROS_INFO_STREAM("receive input torque in Eigen"<<torque.transpose());
    // ROS_INFO_STREAM("receive input torque "<<msg->moments);
    // ROS_INFO_STREAM("receive input wrench "<< thrust<< " "<<torque.transpose());
    ROS_DEBUG_STREAM("mav2 input torque "<<v_mavs_torques.at(2).transpose());
    ROS_DEBUG_STREAM("mav2 input thrust "<<  v_mavs_thrusts.at(2));    
}

// callback function to take input wrench for mav3
void fmCmdCallback3(const rotor_tm_msgs::FMCommand::ConstPtr& msg)
{
    //  std::lock_guard<std::mutex> lock(mavs_mutex);
    // thrust = vector3MsgToEigen(msg->rlink_thrust);
    // thrust = static_cast<double>(msg->thrust);
    v_mavs_thrusts.at(3) = static_cast<double>(msg->thrust);
    // v_mavs_thrusts.at(3) = thrust_all;

    // torque = vector3MsgToEigen(msg->moments);
    v_mavs_torques.at(3) = vector3MsgToEigen(msg->moments);
    // v_mavs_torques.at(3) = Eigen::Vector3d::Zero();
    // ROS_INFO_STREAM("receive input torque in Eigen"<<torque.transpose());
    // ROS_INFO_STREAM("receive input torque "<<msg->moments);
    // ROS_INFO_STREAM("receive input wrench "<< thrust<< " "<ptr_Cooperative->InputControllerInput4MAVs(v_mavs_thrusts, v_mavs_torques);<torque.transpose());
    ROS_DEBUG_STREAM("mav3 input torque "<<v_mavs_torques.at(3).transpose());
    ROS_DEBUG_STREAM("mav3 input thrust "<<v_mavs_thrusts.at(3));    
}


int main(int argc, char** argv)
{
    ROS_INFO_STREAM("FUCKKKK v_mavs_states");
    // 1. init ros nodes
    ros::init(argc, argv, "rotorTM_sim");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
     ros::console::notifyLoggerLevelsChanged();}

    // 2.define ros frequency to be 100
    const int ROS_FREQ = 100;
    ros::Rate loop_rate(ROS_FREQ);

    // set int step size to be same as ros step
    const double dt = 1.0/ROS_FREQ;
    ROS_INFO_STREAM("step size is " << dt);
    // Next check frequency of payload odom and fm_cmd
    // 


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

    ROS_INFO_STREAM("FUUUUCK point 1");

    // TOModifyLater
    // 4. paras for mavs and payload
    // mav
    // snapdragonfly
    const double mav_mass = 0.25;
    Eigen::Matrix3d mav_inertia = Eigen::Matrix3d::Zero(3,3);
    mav_inertia(0,0)= 0.000601;
    mav_inertia(1,1)= 0.000589;
    mav_inertia(2,2)= 0.001076; 

    MassProperty mav_mass_property = {mav_mass, mav_inertia};

    // payload
    // load_params/fedex_box_payload.yaml
    // 1. set payload param
    double payload_mass = 0.250;
    Eigen::Matrix3d payload_inertia = Eigen::Matrix3d::Zero(3,3);    
    payload_inertia(0,0)= 0.000601;
    payload_inertia(1,1)= 0.000589;
    payload_inertia(2,2)= 0.01076; 

    MassProperty payload_mass_property = {payload_mass, payload_inertia};
    // 2. set cable length
    const double cable_length = 0.5;
 
    ROS_DEBUG_STREAM("FUUUUCK point 2");
    // joint post in body frame
    const std::vector<Eigen::Vector3d> v_attach_point_post{{0.3048, -0.3048, 0.2286},{0.3048, 0.3048, 0.2286}, {-0.3048, 0.3048, 0.2286}, {-0.3048,-0.3048, 0.2286}};
    //  const std::vector<Eigen::Vector3d> v_attach_point_post{{1, 0, 0},{0, 1, 0}, {-1, 0, 0}, {0,-1, 0}};



    ROS_DEBUG_STREAM("FUUUUCK point 3");
    // define in stance of joint with attach point post in body frame
    // 
    std::vector<std::shared_ptr<Joint>> v_ptr_joints;
    v_ptr_joints.reserve(num_robots);
    // develop a vector of uav cable
    std::vector<std::shared_ptr<UAVCable>> v_ptr_uavcables;
    v_ptr_uavcables.reserve(num_robots);

    ROS_DEBUG_STREAM("FUUUUCK point 4");
    // resrve 4 slots of v_ptr_joints for 4 joints
    for (size_t i = 0; i < num_robots; i++)
    {
        auto ptr_joint = std::make_shared<Joint>(v_attach_point_post.at(i));

        v_ptr_joints.push_back(ptr_joint);
        // use v_ptr_uavcables[i] or use pus_back

        auto ptr_uav_cable= std::make_shared<UAVCable>(mav_mass_property, cable_length, v_ptr_joints.at(i), dt); 

        v_ptr_uavcables.push_back(ptr_uav_cable);
    }

    for (size_t i = 0; i < num_robots; i++)
    {
        // link join with uav cable
        v_ptr_joints.at(i)->LinkUAVCable(v_ptr_uavcables.at(i));
    }
    

    ROS_DEBUG_STREAM("FUUUUCK point 5");
    // 3. define instance of payload
    auto ptr_payload = std::make_shared<Payload>(payload_mass_property, v_ptr_joints, dt);


    // 5. define quadrotor simulator


    
    // 6. define instance of single quadrotor +  a point mass payload 
    // auto ptr_rotorTM = std::make_shared<rotorTMQuadrotorPointMass>(drone_mass, m_inertia, payload_mass, cable_length, dt);


    // 7. ros output
    std::array<nav_msgs::Odometry, 4> v_mavs_odoms;

    for (size_t i = 0; i < 4; i++)
    {
        v_mavs_odoms.at(i).header.frame_id = "odom";
        v_mavs_odoms.at(i).child_frame_id = "base_link";
    }
    
    nav_msgs::Odometry payload_odom_msg;
    payload_odom_msg.header.frame_id = "odom";
    payload_odom_msg.child_frame_id = "base_link";


    // set initial post for payload and mavs
    Eigen::Vector3d payload_init_post{0,0,0};
    // set initial post for payload
    // ptr_payload->SetInitialPost(payload_init_post);

    // // set initial post for each mav-cable
    // for (size_t i = 0; i < 4; i++)
    // {
    //     // get ith join post in body frame with the vector of shared pointer of joint
    //     Eigen::Vector3d attach_point_post_body_frame;
    //     attach_point_post_body_frame = v_ptr_joints.at(i)->post_body_frame();

    //     // set initial post for each mav-cable
    //     v_ptr_uavcables[i]->SetMAVInitPostCableTautWithAttachPointPost(payload_init_post + attach_point_post_body_frame);
    // }
    


    // 8. define cooperative instance
    std::shared_ptr<Cooperative> ptr_Cooperative = std::make_shared<Cooperative>(ptr_payload, v_ptr_joints, v_ptr_uavcables);

    ptr_Cooperative->SetPayloadInitPost(payload_init_post);

    // 9. run simulation
    while (ros::ok()  ) // && (counter_simulation < 20)
    {   

        // thrust_all  = thrust_all +0.0001;
        ROS_DEBUG_STREAM(counter_simulation << "iteration");

        ROS_DEBUG_STREAM("Input controller input for mavs");
        ptr_Cooperative->InputControllerInput4MAVs(v_mavs_thrusts, v_mavs_torques);

        ROS_DEBUG_STREAM("update kinematcs of joints");
        ptr_Cooperative->UpdateJointAndCableStatus();  

        ROS_DEBUG_STREAM("collision");
        ptr_Cooperative->UpdateVelsCollidedUAVsPayload();

        ROS_DEBUG_STREAM("compute interaction wrenches");
        ptr_Cooperative->ComputeInteractWrenches();

        ROS_DEBUG_STREAM("do one step dynamic simulation for mavs and payload");
        ptr_Cooperative->DoOneStepInt4Robots();

     
        // setp 5. Publish simulation results to topics
        // setp 5.1 assigen drone state infor (position, vel, attitude, bodyrate) to odom_msg
        for (size_t i = 0; i < 4; i++)
        {
            const UAVCable& uavcable = *v_ptr_uavcables[i];
            v_mavs_odoms.at(i).header.stamp = ros::Time::now();
            v_mavs_odoms.at(i).pose.pose.position = EigenToPointMsg(uavcable.mav_.pose().post);
            v_mavs_odoms.at(i).pose.pose.orientation = EigenQuadnToGeomQuadn(uavcable.mav_.pose().att);
            v_mavs_odoms.at(i).twist.twist.linear = EigenToVector3Msg(uavcable.mav_.vels().linear_vel);
            v_mavs_odoms.at(i).twist.twist.angular = EigenToVector3Msg(uavcable.mav_.vels().bodyrate);

            ROS_DEBUG_STREAM(i <<"th mav post is "<< v_mavs_odoms.at(i).pose.pose.position.x <<" "<< v_mavs_odoms.at(i).pose.pose.position.y <<" "<< v_mavs_odoms.at(i).pose.pose.position.z); 
            ROS_DEBUG_STREAM(i <<"th mav linear vel is "<< v_mavs_odoms.at(i).twist.twist.linear.x <<" "<< v_mavs_odoms.at(i).twist.twist.linear.y <<" "<< v_mavs_odoms.at(i).twist.twist.linear.z);

            // get mav euler angles
            auto m_mav_rot = GeomQuadnToEigenRotMatrix(v_mavs_odoms.at(i).pose.pose.orientation);
            auto mav_euler_angle = m_mav_rot.eulerAngles(2,1,0);
            ROS_DEBUG_STREAM(i<<" th mavEuler angles are " << " roll " << mav_euler_angle[2]<< " pitch " <<mav_euler_angle[1] << " yaw " << mav_euler_angle[0]);   

            ROS_DEBUG_STREAM(i <<"th mav bodyrate is "<< v_mavs_odoms.at(i).twist.twist.angular.x<< " "<< v_mavs_odoms.at(i).twist.twist.angular.y <<" "<< v_mavs_odoms.at(i).twist.twist.angular.z);
        }
        
        // payload
        payload_odom_msg.header.stamp = ros::Time::now();

        payload_odom_msg.pose.pose.position = EigenToPointMsg(ptr_payload->pose().post);
        // ROS_INFO_STREAM("payload post "<< payload_state.position.transpose());
        payload_odom_msg.pose.pose.orientation = EigenQuadnToGeomQuadn(ptr_payload->pose().att);
        // ROS_INFO_STREAM("payload attitude "<< payload_state.attitude);
        payload_odom_msg.twist.twist.linear = EigenToVector3Msg(ptr_payload->vels().linear_vel);
        // ROS_INFO_STREAM("payload vel "<< payload_state.vel.transpose());
        payload_odom_msg.twist.twist.angular = EigenToVector3Msg(ptr_payload->vels().bodyrate);
        // ROS_INFO_STREAM("payload bodyrate "<< payload_state.bodyrate.transpose());

        ROS_DEBUG_STREAM("payload post is "<< payload_odom_msg.pose.pose.position.x <<" "<< payload_odom_msg.pose.pose.position.y <<" "<< payload_odom_msg.pose.pose.position.z); 

        ROS_DEBUG_STREAM("payload linear vel is "<<payload_odom_msg.twist.twist.linear.x <<" " <<payload_odom_msg.twist.twist.linear.y << " "<<payload_odom_msg.twist.twist.linear.z);

        // get payload euler angles
        auto m_payload_rot = GeomQuadnToEigenRotMatrix(payload_odom_msg.pose.pose.orientation);
        auto payload_euler_angle = m_payload_rot.eulerAngles(2,1,0);
        ROS_DEBUG_STREAM("payload Euler angles are "  << " roll " << payload_euler_angle[2]<< " pitch " <<payload_euler_angle[1] << " yaw " << payload_euler_angle[0]);
        ROS_DEBUG_STREAM("payload bodyrate is "<< payload_odom_msg.twist.twist.angular.x<< " " << payload_odom_msg.twist.twist.angular.y << " "<< payload_odom_msg.twist.twist.angular.z);

        // setp 5.publish odom_msgs of drone and payload
        mav0_odom_pub.publish(v_mavs_odoms.at(0));    
        mav1_odom_pub.publish(v_mavs_odoms.at(1));   
        mav2_odom_pub.publish(v_mavs_odoms.at(2));   
        mav3_odom_pub.publish(v_mavs_odoms.at(3));   
        payload_odom_pub.publish(payload_odom_msg);  

       
        //  ROS_INFO_STREAM("ROS loop end");
        // run ros loop
        ros::spinOnce();

        loop_rate.sleep();  

        ROS_DEBUG_STREAM("-----End of iteration-----");
        ROS_DEBUG_STREAM("-----End of iteration-----");
        // fuck++;
    }
    

}












        // ROS_INFO_STREAM("ROS loop begin");
        // std::vector<UAVCable> &v_drone_cable = ptr_Cooperative->v_drone_cable_;
        // Eigen::Vector3d mav0_pst{1,2,3};
        // v_drone_cable.at(0).mav_.GetPosition(mav0_pst);
        // ROS_INFO_STREAM("FUUUUCK point 20");

        // // step 1 input control sigals for mavs
        // // input control sigals for mavs using v_ptr_uavcables
        // for (size_t i = 0; i < 4; i++)
        // {
        //     v_ptr_uavcables[i]->InputControllerInput(v_mavs_thrusts[i], v_mavs_torques[i]);
        // }
        // // drone_cable.InputControllerInput(mav_control_input.first, mav_control_input.second);
        // // ROS_INFO_STREAM("FUUUUCK point 21");
        // // step 2 update kinematics of joints
        // ptr_payload->ComputeJointKinematics();


        // // step 3 check collision between mavs and payload at current iteration
        // for (size_t i = 0; i < 4; i++)
        //     {
        //         v_ptr_uavcables[i]->CheckInelasticCollision();
        //     }

        // // save collision status to v_flags_inelastic_collision_status
        // std::vector<bool> v_flags_inelastic_collision_status(4,false);
        // for (size_t i = 0; i < 4; i++)
        //     {
        //         v_flags_inelastic_collision_status[i] = v_ptr_uavcables[i]->inelasticCollisionStauts();
        //         // std::cout<<"ith element of v_flags_inelastic_collision_status is"<<v_flags_inelastic_collision_status[i]<<std::endl;
        //     }


        // /*Avoid any collision*/
        // for (size_t i = 0; i < 4; i++)
        //     {
        //         v_flags_inelastic_collision_status[i] = false;
               
        //     }


        // // step 3 distribute vels among collided mavs and payload until there is no collision
        // while (std::any_of(v_flags_inelastic_collision_status.begin(), v_flags_inelastic_collision_status.end(), [](const bool& collision_flag) { return collision_flag; })) 
        // {

        //      ROS_DEBUG_STREAM("FUUUUCK point 6");    
        //     // 3.1 define a vector of 4 mav-cable inelatic collision status
        //     std::vector<bool> v_mavs_inelastic_status_before_collision(4,false);
        //     //assign v_mavs_inelastic_collision with value of inelastic_collision_ of each mav-cable
        //     for (size_t i = 0; i < 4; i++)
        //     {
        //         v_mavs_inelastic_status_before_collision[i] = v_ptr_uavcables[i]->inelasticCollisionStauts();
        //     }

        //     // 3.2 check collision between mavs and payload
        //     // check each mav if it has collision with its attach point
        //     // update vels of payload because of collision
        //     ptr_payload->UpdateVelCollided();

        //     // 3.3 update vel of mav if collision happend
        //     for (size_t i = 0; i < 4; i++)
        //     {
        //         // obtain ith uav-cable instance from vector
        //         std::shared_ptr<UAVCable>& ptr_uavcable = v_ptr_uavcables.at(i);

        //         // check if there is collision between mav and payload
        //         // UpdateMAVVelCollided(const Eigen::Quaterniond &payload_attitude, const Eigen::Vector3d &payload_vel_collided, const Eigen::Vector3d &payload_bodyrate_collided)
        //         ptr_uavcable->UpdateMAVVelCollided(ptr_payload->pose().att, ptr_payload->vels().linear_vel, ptr_payload->vels().bodyrate);
        //     };

        //     // 3.4 save inelastic collision status of each mav-cable after collision
        //     std::vector<bool> v_mavs_inelastic_status_after_collision(4,false);
        //     //assign v_mavs_inelastic_status_after_collision with value of inelastic_collision_ of each mav-cable
        //     for (size_t i = 0; i < 4; i++)
        //     {
        //         v_mavs_inelastic_status_after_collision[i] = v_ptr_uavcables[i]->inelasticCollisionStauts();
        //     }

        //     // 3.5 check and find "new collision":
        //     // that was not collisioned before
        //     // but will get collision after  
        //     std::vector<bool> v_mavs_inelastic_status_diff(4,false);

        //     for (size_t i = 0; i < v_mavs_inelastic_status_diff.size(); ++i) 
        //         {
        //             v_mavs_inelastic_status_diff[i] = (v_mavs_inelastic_status_after_collision[i] && !v_mavs_inelastic_status_before_collision[i]);

        //             // std::cout<<i<<"th element is"<<v_mavs_inelastic_status_diff[i]<<std::endl; 
        //         };

        //     bool condition_exist_new_collision = std::any_of(v_mavs_inelastic_status_diff.begin(),
        //                              v_mavs_inelastic_status_diff.end(),
        //                              [](bool val) { return val; });

        //     // 3.6
        //     if (condition_exist_new_collision)
        //     {
        //         for (size_t i = 0; i < v_flags_inelastic_collision_status.size(); ++i) 
        //             {
        //                 v_flags_inelastic_collision_status[i] = (v_mavs_inelastic_status_after_collision[i] ||v_mavs_inelastic_status_before_collision[i]);
                         
        //             };
        //     }
        //     else
        //     {
        //         std::cout<<"no new collision"<<std::endl;

        //         // update post and vels of joint
        //         ptr_payload->ComputeJointKinematics();

        //         v_flags_inelastic_collision_status = v_mavs_inelastic_status_after_collision;

        //         for (size_t i = 0; i < 4; i++)
        //             {
        //                 // debug here
        //                 v_ptr_uavcables[i]->CheckInelasticCollision();
        //             }

        //     }
     
        // }

        // ROS_DEBUG_STREAM("FUUUUCK point 7");   
        // // step 4 update dynamics of mavs and payload
        // // 4.1 compute interaction parameters from each UAV-Cable
        // CooperIntertPara interaction_parameters;

        // Eigen::Matrix3d sum_m_C_i = Eigen::Matrix3d::Zero();
        // Eigen::Matrix3d sum_m_D_i = Eigen::Matrix3d::Zero();
        // Eigen::Matrix3d sum_m_E_i = Eigen::Matrix3d::Zero();
        // Eigen::Matrix3d m_mass_matrix = Eigen::Matrix3d::Identity() * ptr_payload->mass();

        // for (size_t i = 0; i < 4; i++)
        //     {
        //        // compute MDi, MCi and MEi for each mav     
        //         v_ptr_uavcables[i]->ComputeMatrixMDiMCiMEi(ptr_payload->pose().att);

        //         // accumulate MDi, MCi and MEi
        //         sum_m_C_i = sum_m_C_i +  v_ptr_uavcables[i]->m_C_i();
        //         sum_m_D_i = sum_m_C_i +  v_ptr_uavcables[i]->m_D_i();
        //         sum_m_E_i = sum_m_C_i +  v_ptr_uavcables[i]->m_E_i();

        //         // accumlate
        //         m_mass_matrix = m_mass_matrix + v_ptr_uavcables[i]->m_mass_matrix();
        //     }        
        
        // // assigen to interaction_parameters
        // interaction_parameters.m_C = sum_m_C_i;
        // interaction_parameters.m_D = sum_m_D_i;
        // interaction_parameters.m_E = sum_m_E_i;
        // interaction_parameters.m_mass_matrix = m_mass_matrix;

        // ptr_payload->InputPayloadInteractPara(interaction_parameters);

        // // compute net wrench applied by UAVs to payload
        // Wrench net_mavs_wrench;
        // for (size_t i = 0; i < 4; i++)
        //     {  
        //         v_ptr_uavcables[i]->ComputeInteractionWrenches(ptr_payload->pose().att, ptr_payload->vels().bodyrate);
        //         auto mav_wrench = v_ptr_uavcables[i]->attach_point_wrench();
        //         net_mavs_wrench = net_mavs_wrench + mav_wrench;
        //     }           
        
        // ptr_payload->InputDronesNetWrenches(net_mavs_wrench);


        // ptr_payload->DoOneStepInt();

        // for (size_t i = 0; i < 4; i++)
        // {
        //        // compute MDi, MCi and MEi for each mav     
        //         v_ptr_uavcables[i]->ComputeNetWrenchApplied2MAV();
        //         v_ptr_uavcables[i]->DoOneStepInt();
        // }  
        














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


Eigen::Matrix3d GeomQuadnToEigenRotMatrix(const geometry_msgs::Quaternion& q_geom)
{

    auto q = Eigen::Quaterniond(q_geom.w, q_geom.x, q_geom.y, q_geom.z);

    return q.toRotationMatrix();
}