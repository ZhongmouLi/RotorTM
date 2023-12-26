#ifndef QUADROTORCOOPERATIVE_SIMULATOR_H
#define QUADROTORCOOPERATIVE_SIMULATOR_H


#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include "lib_cable.hpp"
#include "lib_rigidbody.hpp"
#include "lib_payload.hpp"


class Cooperative{

    private:

        // vector of cable instances
        std::vector<UAVCable> v_drone_cable_;

        // payload
        Payload payload_;

        // number of robots
        size_t number_robots;

        // controller inputs for n MAVs
        // for each MAV, the control input is thrust (scalar) + torque (3X1 vector)        
        std::vector<std::pair<double, Eigen::Vector3d>> v_controllers_inputs_;

        // compute the projection of robot vel perpendicular to the cable
        // input: drone vel (3X1 vector) and cable direction (3X1 vector)
        // output:: drone vel projection that is perpendicular to cable (3X1 vector)
        Eigen::Vector3d CalVelProjPerpendicularCable(const Eigen::Vector3d drone_vel, const Eigen::Vector3d &cable_direction);


        Cooperative();

    public:

    Cooperative(const std::vector<Eigen::Vector3d> &v_attach_point_post_bf, const double &mass, const Eigen::Matrix3d &m_inertia, const double &step_size);

    // CheckCollisions4MAVsPayload checks if collission happens for every cable
    void CheckCollisions4MAVCables();

    // update vels of collided MAVs and payload 
    // method: eq (39)-(42)
    void UpdateVelsCollidedUAVsPayload();

    // compute interation wrenches and vars for MAVs and payload
    void ComputeInteractWrenches();

    // call one step dynamic simulation for payload
    void DoOneStepInt4Payload();


    // call one step dynamic simulation for MAVs and payload
    void DoOneStepInt4Robots();
};


#endif