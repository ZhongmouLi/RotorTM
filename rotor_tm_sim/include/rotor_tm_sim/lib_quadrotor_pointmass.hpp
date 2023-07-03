#ifndef QUADROTORPOINTMASS_SIMULATOR_H
#define QUADROTORPOINTMASS_SIMULATOR_H

#include <iostream>
#include <memory>
#include <Eigen/Dense>

#include "rotor_tm_sim/lib_quadrotor_dynamic_simulator.hpp"

#include "rotor_tm_sim/lib_pointmass_dynamic_simulator.hpp"

class rotorTMQuadrotorPointMass
{

    private:
        std::shared_ptr<QuadrotorDynamicSimulator> quadrotor;   

        std::shared_ptr<PointMassDynamicSimulator> pm_payload;   

        rotorTMQuadrotorPointMass();

        bool cable_is_slack_;

        double mav_mass_;

        double payload_mass_;

        double cable_length_;

        Eigen::Vector3d mav_thrust_force_;

        Eigen::Vector3d mav_torque_;

        Eigen::Vector3d tension_force_;


        bool isSlack(const Eigen::Vector3d &mav_position, const Eigen::Vector3d &payload_position);

        Eigen::Vector3d computeTensionForce(const Eigen::Vector3d &mav_position, const Eigen::Vector3d &payload_position, const Eigen::Vector3d &mav_vel, const Eigen::Vector3d &payload_vel);

        // compute vels of mav and payload after collision
        std::pair<Eigen::Vector3d, Eigen::Vector3d> updateVel4CableCollision(const Eigen::Vector3d &mav_position, const Eigen::Vector3d &payload_position, const Eigen::Vector3d &mav_vel, const Eigen::Vector3d &payload_vel);        

    public:

        // constructor
        rotorTMQuadrotorPointMass(const double &MAV_mass, const Eigen::Matrix3d &m_inertia, const double &pd_mass, const double &step_size);

        // input uav thrust and torque
        void inputMAVThrust(const double &mav_thrust);

        void inputMAVTorque(const Eigen::Vector3d &mav_torque){ mav_torque_ =  mav_torque; };

        void setInitPost(const Eigen::Vector3d &payload_init_position);

        void doOneStepint();

};

#endif