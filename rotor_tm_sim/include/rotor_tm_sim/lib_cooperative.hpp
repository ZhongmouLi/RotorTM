#ifndef QUADROTORCOOPERATIVE_SIMULATOR_H
#define QUADROTORCOOPERATIVE_SIMULATOR_H


#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>
#include <string> 
#include "lib_uav_cable.hpp"
#include "lib_payload.hpp"


class Cooperative{

    private:

        // number of robots
        size_t number_robots_;

        // controller inputs for n MAVs
        // for each MAV, the control input is thrust (scalar) + torque (3X1 vector)        
        std::vector<std::pair<double, Eigen::Vector3d>> v_controllers_inputs_;

        // compute the projection of robot vel perpendicular to the cable
        // input: drone vel (3X1 vector) and cable direction (3X1 vector)
        // output:: drone vel projection that is perpendicular to cable (3X1 vector)
        Eigen::Vector3d CalVelProjPerpendicularCable(const Eigen::Vector3d drone_vel, const Eigen::Vector3d &cable_direction);


        Cooperative();

        const double gravity_ = 9.8;

        bool intial_payload_acc_set = false;

    public:

    // put them in private LATER
    // vector of cable instances
    std::vector<UAVCable> v_drone_cable_;

    // payload
    Payload payload_;

    Cooperative(const std::vector<Eigen::Vector3d> &v_attach_point_post_bf, const double &mav_mass, const Eigen::Matrix3d &mav_inertia, const double cable_length, const double &payload_mass, const Eigen::Matrix3d &payload_inertia, const double &step_size);

    // CheckCollisions4MAVsPayload checks if collission happens for every cable
    // void CheckCollisions4MAVCables();

    // set init post of payload
    void SetPayloadInitPost();

    void SetPayloadInitialAccAndBodyrateACC();

    // update vels of collided MAVs and payload 
    // method: eq (39)-(42)
    void UpdateVelsCollidedUAVsPayload();

    void InputControllerInput4MAVs(const Eigen::VectorXd v_mavs_thrust, const std::vector<Eigen::Vector3d> v_mavs_torque);

    // compute interation wrenches and vars for MAVs and payload
    void ComputeInteractWrenches();

    // call one step dynamic simulation for MAVs and payload
    void DoOneStepInt4Robots();
};


#endif