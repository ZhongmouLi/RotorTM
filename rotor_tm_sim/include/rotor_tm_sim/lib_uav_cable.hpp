#ifndef UAVCABLE_SIMULATOR_H
#define UAVCABLE_SIMULATOR_H

#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include "lib_cable.hpp"
#include "lib_quadrotor.hpp"


// typedef Eigen::Matrix<double, 12, 1> object_state;


class UAVCable{

    public:
        // cable instances (shared pointers)
        Cable cable_; 

        // uav instances (shared pointers)
        Quadrotor drone_;

    private:
        
        //control input
        // mav thrust (double)
        double mav_thrust_;
        // mav torque (3X1 vector) in body frame
        Eigen::Vector3d mav_torque_;   

        // attach point post
        std::shared_ptr<Eigen::Vector3d> ptr_attach_point_post_;

        UAVCable();

        // compute the projection of robot vel perpendicular to the cable
        // input: drone vel (3X1 vector) and cable direction (3X1 vector)
        // output:: drone vel projection that is perpendicular to cable (3X1 vector)
        Eigen::Vector3d CalVelProjPerpendicularCable(const Eigen::Vector3d drone_vel, const Eigen::Vector3d &cable_direction);

        void UpdateVelCollidedOneUAVPayload();

    public:

    
    UAVCable(const double &mass, const Eigen::Matrix3d &m_inertia, const double &step_size, const double & cable_length);    


    void UpdateVelCollidedUAVVel(const Eigen::Quaterniond &payload_attitude, const Eigen::Vector3d &attach_point_body_frame, const Eigen::Vector3d &payload_vel_collided, const Eigen::Vector3d &payload_bodyrate_collided);

    // check collisions for a UAV cable object with its attach point
    void CheckCollision(const Eigen::Vector3d &attachpoint_post, const Eigen::Vector3d &attachpoint_vel);

    // compute control inputs for mav for both slack and taut status
    void ComputeControlInputs4MAV();

    //  
    // method: eq (39)-(42)
    void UpdateVelsCollidedUAVsPayload();

    // import control input from controller
    void InputControllerInput(const double &mav_thrust, const Eigen::Vector3d &mav_torque);


};
#endif
