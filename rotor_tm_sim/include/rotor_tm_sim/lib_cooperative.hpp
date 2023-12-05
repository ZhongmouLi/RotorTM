#ifndef QUADROTORCOOPERATIVE_SIMULATOR_H
#define QUADROTORCOOPERATIVE_SIMULATOR_H

#include <iostream>
#include <memory>
#include <Eigen/Dense>
#include <thread>

// #include <boost/asio.hpp> // for bost pool

#include "rotor_tm_sim/lib_quadrotor_dynamic_simulator.hpp"

#include "rotor_tm_sim/lib_pointmass_dynamic_simulator.hpp"

#include "rotor_tm_sim/lib_payload.hpp"

class RotorTMQuadrotorCooperative
{
    public:

        // quadrotor object
        // std::shared_ptr<QuadrotorDynamicSimulator> quadrotor; 
        std::vector<std::shared_ptr<QuadrotorDynamicSimulator>> v_mav_;

        // payload
        std::shared_ptr<Payload> payload_;    

        // cables
        std::vector<std::shared_ptr<Cable>> v_cable_;

    private:


        // drone thrust force, 3X1 vector in world frame 
        std::vector<Eigen::Vector3d> v_mav_thrust_force_;

        // drone torque, 3X1 vector in body frame 
        std::vector<Eigen::Vector3d>  v_mav_torque_;

        // tension force of cable, 3X1 vector in world frame 
        std::vector<Eigen::Vector3d> v_tension_force_;

        // group of quadrotors whose cables are in taut
        std::vector<std::shared_ptr<QuadrotorDynamicSimulator>> v_mav_taut;
        
        // group of quadrotors whose cables are in slack
        std::vector<std::shared_ptr<QuadrotorDynamicSimulator>> v_mav_slack;

        // check if cable is slack or not
        // input: drone post and payload post: 3X1 vector in world frame
        // output: bool var
        bool isSlack(const Eigen::Vector3d &mav_position, const Eigen::Vector3d &payload_position);

        // check cables' status and group the corresponding drones into v_mav_taut and v_mav_slack
        // input:drone post,  payload post, drone vel, payload vel: each is 3X1 vector in world frame
        // output: update v_mav_taut and v_mav_slack
        void checkCableStatus()


        // compute vels of mav and payload after collision
        // input: drone post,  payload post, drone vel, payload vel: each is 3X1 vector in world frame
        // output:: drone vel after collision and payload vel after collision: 3X1 vector in world frame
        std::pair<Eigen::Vector3d, Eigen::Vector3d> updateVel4CableCollision(const Eigen::Vector3d &mav_position, const Eigen::Vector3d &payload_position, const Eigen::Vector3d &mav_vel, const Eigen::Vector3d &payload_vel);        

        // step size for integration
        double step_size_;

        // var to record current step
        double current_step_ = 0;

        // prevent creating null instance
        rotorTMQuadrotorCooperative();

        // boost::asio::thread_pool pool_rotorTM;

        // std::function<void()>  quadrotor_func = std::bind(&QuadrotorDynamicSimulator::doOneStepInt, quadrotor);
        // std::function<void()>  payload_func = std::bind(&PointMassDynamicSimulator::doOneStepInt, pm_payload);

    public:

        // constructor
        rotorTMQuadrotorCooperative(const double &MAV_mass, const Eigen::Matrix3d &m_inertia, const double &pd_mass, const double &cable_length, const double &step_size);

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