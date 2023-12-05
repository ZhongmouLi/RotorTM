#ifndef QUADROTORPOINTMASS_SIMULATOR_H
#define QUADROTORPOINTMASS_SIMULATOR_H

#include <iostream>
#include <memory>
#include <Eigen/Dense>
#include <thread>

// #include <boost/asio.hpp> // for bost pool

#include "rotor_tm_sim/lib_quadrotor.hpp"

#include "rotor_tm_sim/lib_pointmass.hpp"

class rotorTMQuadrotorPointMass
{
    public:

        // quadrotor object
        std::shared_ptr<Quadrotor> quadrotor;   

        // point mass object
        std::shared_ptr<PointMass> pm_payload;    

    private:

        // // quadrotor object
        // std::shared_ptr<QuadrotorDynamicSimulator> quadrotor;   

        // // point mass object
        // std::shared_ptr<PointMass> pm_payload;   

        // var to check if cable is slack or not
        bool cable_is_slack_ = false;

        // mass of drone
        double mav_mass_;

        // mass of payload, i.e. point mass
        double payload_mass_;

        // cable length
        double cable_length_;

        // drone thrust force, 3X1 vector in world frame 
        Eigen::Vector3d mav_thrust_force_;

        // drone torque, 3X1 vector in body frame 
        Eigen::Vector3d mav_torque_;

        // tension force of cable, 3X1 vector in world frame 
        Eigen::Vector3d tension_force_;

        // check if cable is slack or not
        // input: drone post and payload post: 3X1 vector in world frame
        // output: bool var
        bool isSlack(const Eigen::Vector3d &mav_position, const Eigen::Vector3d &payload_position);
        
        // compute tension force
        // input:drone post,  payload post, drone vel, payload vel: each is 3X1 vector in world frame
        // output: tension force 3X1 vector in world frame
        Eigen::Vector3d computeTensionForce(const Eigen::Vector3d &mav_position, const Eigen::Vector3d &payload_position, const Eigen::Vector3d &mav_vel, const Eigen::Vector3d &payload_vel);

        // compute vels of mav and payload after collision
        // input: drone post,  payload post, drone vel, payload vel: each is 3X1 vector in world frame
        // output:: drone vel after collision and payload vel after collision: 3X1 vector in world frame
        std::pair<Eigen::Vector3d, Eigen::Vector3d> updateVel4CableCollision(const Eigen::Vector3d &mav_position, const Eigen::Vector3d &payload_position, const Eigen::Vector3d &mav_vel, const Eigen::Vector3d &payload_vel);        

        // step size for integration
        double step_size_;

        // var to record current step
        double current_step_ = 0;

        // prevent creating null instance
        rotorTMQuadrotorPointMass();

        // boost::asio::thread_pool pool_rotorTM;

        // std::function<void()>  quadrotor_func = std::bind(&QuadrotorDynamicSimulator::doOneStepInt, quadrotor);
        // std::function<void()>  payload_func = std::bind(&PointMass::doOneStepInt, pm_payload);

    public:

        // constructor
        rotorTMQuadrotorPointMass(const double &MAV_mass, const Eigen::Matrix3d &m_inertia, const double &pd_mass, const double &cable_length, const double &step_size);

        // input uav thrust (scale)  and torque (3X1 vector in body frame)
        void inputMAVThrust(const double &mav_thrust);

        void inputMAVTorque(const Eigen::Vector3d &mav_torque){ mav_torque_ =  mav_torque; };

        // define initial position of system by specifying the initial position of payload
        // init post of drone = int post of payload + cable length
        void setInitPost(const Eigen::Vector3d &payload_init_position);

        // call one step integration
        void doOneStepint();

};

#endif