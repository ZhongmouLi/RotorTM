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
        Quadrotor mav_;

    private:
        
        //control input
        // mav thrust (double)
        double mav_thrust_;
        // mav torque (3X1 vector) in body frame
        Eigen::Vector3d mav_torque_;   

        // attach point force
        // it is the force the drone applies to the attach point
        Eigen::Vector3d mav_attach_point_force_;

        // it is the force the drone applies to the attach point
        Eigen::Vector3d mav_attach_point_torque_;        

        // paramters for payload dynamic equation
        Eigen::Matrix3d m_D_i_;
        Eigen::Matrix3d m_C_i_;
        Eigen::Matrix3d m_E_i_;               

        UAVCable() = delete ;

        // compute the projection of robot vel perpendicular to the cable
        // input: drone vel (3X1 vector) and cable direction (3X1 vector)
        // output:: drone vel projection that is perpendicular to cable (3X1 vector)
        Eigen::Vector3d CalVelProjPerpendicularCable(const Eigen::Vector3d mav_vel, const Eigen::Vector3d &cable_direction);


        // compute attach point force that is the force applied by a drone to the payload at its attached point
        Eigen::Vector3d ComputeAttachPointForce(const Eigen::Vector3d &cable_direction, const Eigen::Vector3d &cable_bodyrate, const Eigen::Vector3d &attach_point_post, const Eigen::Quaterniond &payload_attitude, Eigen::Vector3d &payload_bodyrate);


        // compute attach point torque that is the torque applied by a drone to the payload at its attached point
        Eigen::Vector3d ComputeAttachPointTorque(const Eigen::Vector3d &attach_point_post, const Eigen::Quaterniond &payload_attitude, Eigen::Vector3d &attach_point_force);
  
    public:

    // constrcuster for quadrotor and cable
    UAVCable(const double &mass, const Eigen::Matrix3d &m_inertia, const double &step_size, const double & cable_length);    

    // check if collision happens for a UAV cable object with its attach point
    void CheckCollision(const Eigen::Vector3d &attachpoint_post, const Eigen::Vector3d &attachpoint_vel);

    // update vel of UAV if collision happend
    void UpdateVelCollidedUAVVel(const Eigen::Quaterniond &payload_attitude, const Eigen::Vector3d &attach_point_body_frame, const Eigen::Vector3d &payload_vel_collided, const Eigen::Vector3d &payload_bodyrate_collided);

    // compute force and torque applied by MAV to payload at attach point position
    void ComputeAttachPointWrenches(const Eigen::Vector3d &attach_point_post, const Eigen::Quaterniond &payload_attitude, Eigen::Vector3d &payload_bodyrate);

    // compute term m_D (m means matrix) and m_D is to compute payload translational dynamic equation
    void ComputeMatrixMDiMCiMEi(const Eigen::Vector3d &cable_direction, const Eigen::Quaterniond &payload_attitude, const Eigen::Vector3d &attach_point_post);

    // compute control inputs for mav for both slack and taut status
    void ComputeControlInputs4MAV(const Eigen::Vector3d &attach_point_acc);

    // import control input from controller
    void InputControllerInput(const double &mav_thrust, const Eigen::Vector3d &mav_torque);


    // obtain class member variables
    // obtain attach point force
    void GetAttachPointForce(Eigen::Vector3d &mav_attach_point_force){ mav_attach_point_force = mav_attach_point_force_;};

    // obtain attach point torque
    void GetAttachPointTorque(Eigen::Vector3d &mav_attach_point_torque){ mav_attach_point_torque = mav_attach_point_torque_;};

    // obtain m_D_i
    void GetMatrixMDiMCiMEi(Eigen::Matrix3d &m_C_i, Eigen::Matrix3d &m_D_i, Eigen::Matrix3d &m_E_i) const;

};
#endif
