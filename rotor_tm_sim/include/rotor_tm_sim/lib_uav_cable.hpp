#ifndef UAVCABLE_SIMULATOR_H
#define UAVCABLE_SIMULATOR_H

#include <iostream>
#include <Eigen/Dense>
#include <iomanip>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include "rotor_tm_sim/base/lib_cable.hpp"
#include "rotor_tm_sim/base/lib_base.hpp"
#include "rotor_tm_sim/lib_quadrotor.hpp"
// #include "rotor_tm_sim/lib_joint.hpp"

// typedef Eigen::Matrix<double, 12, 1> object_state;
class Joint;

class UAVCable{

    public:
        // uav instances
        Quadrotor mav_;

        // cable instances
        Cable cable_; 

        //
        // std::weak_ptr<const Payload> ptr_payload_;

    private:

        // var indicates inelastic collision
        bool inelastic_collision_ = false;

        // attach point is a shared property between payload and UAVCable
        mutable std::weak_ptr<const Joint> ptr_joint_;

        //control input
        // // mav thrust (double)
        // double mav_thrust_input_;
        // // mav torque (3X1 vector) in body frame
        // Eigen::Vector3d mav_torque_input_;   
        Wrench mav_input_wrench_;

        // attach point force
        // // it is the force the drone applies to the attach point
        // Eigen::Vector3d mav_attach_point_force_;

        // // it is the force the drone applies to the attach point
        // Eigen::Vector3d mav_attach_point_torque_;        
        Wrench mav_attach_point_wrench_;

        // paramters for payload dynamic equation
        Eigen::Matrix3d m_D_i_;
        Eigen::Matrix3d m_C_i_;
        Eigen::Matrix3d m_E_i_;               

        // compute the projection of robot vel perpendicular to the cable
        // input: drone vel (3X1 vector) and cable direction (3X1 vector)
        // output:: drone vel projection that is perpendicular to cable (3X1 vector)
        // Eigen::Vector3d CalVelProjPerpendicularCable(const Eigen::Vector3d mav_vel, const Eigen::Vector3d &cable_direction);
        Eigen::Vector3d CalVelProjPerpendicularCable();

        // compute attach point force that is the force applied by a drone to the payload at its attached point
        // Eigen::Vector3d ComputeAttachPointForce(const Eigen::Vector3d &cable_direction, const Eigen::Vector3d &cable_bodyrate, const Eigen::Vector3d &attach_point_post, const Eigen::Quaterniond &payload_attitude, Eigen::Vector3d &payload_bodyrate);

        Eigen::Vector3d ComputeNetForceApplied2AttachPoint(const Eigen::Quaterniond &payload_attitude, const Eigen::Vector3d &payload_bodyrate);

        // compute attach point torque that is the torque applied by a drone to the payload at its attached point
        // Eigen::Vector3d ComputeAttachPointTorque(const Eigen::Vector3d &attach_point_post_bf, const Eigen::Quaterniond &payload_attitude, Eigen::Vector3d &attach_point_force);
        Eigen::Vector3d ComputeNetTorqueApplied2AttachPoint(const Eigen::Quaterniond &payload_attitude, const Eigen::Vector3d &attach_point_force);
        
        std::shared_ptr<const Joint> ptr_joint();

        UAVCable() = delete ;
        
    public:

    // constrcuster for quadrotor and cable
    // UAVCable(const double &mass, const Eigen::Matrix3d &m_inertia, const double & cable_length, const double &step_size);   
    UAVCable(const MassProperty &mav_mass_property, const double & cable_length, const double &step_size);  

    UAVCable(const MassProperty &mav_mass_property, const double & cable_length, const std::shared_ptr<const Joint> &ptr_attach_point, const double &step_size);  
    
    // call one step dynamic simulation
    virtual void DoOneStepInt() final;

    void CheckInelasticCollision();

    void UpdateCableTautStatus();

    // check if collision happens for a UAV cable object with its attach point
    // bool IsCollided(const Eigen::Vector3d &attachpoint_post, const Eigen::Vector3d &attachpoint_vel);

    // update vel of UAV if collision happend
    // void UpdateVelCollidedMAVVel(const Eigen::Quaterniond &payload_attitude, const Eigen::Vector3d &attach_point_body_frame, const Eigen::Vector3d &payload_vel_collided, const Eigen::Vector3d &payload_bodyrate_collided);
    void UpdateMAVVelCollided(const Eigen::Quaterniond &payload_attitude, const Eigen::Vector3d &payload_vel_collided, const Eigen::Vector3d &payload_bodyrate_collided);


    // compute force and torque applied by MAV to payload at attach point position
    // void ComputeAttachPointWrenches(const Eigen::Vector3d &attach_point_post_bf, const Eigen::Vector3d &attach_point_post, const Eigen::Vector3d &attach_point_vel, const Eigen::Quaterniond &payload_attitude, Eigen::Vector3d &payload_bodyrate);
    void ComputeInteractionWrenches(const Eigen::Quaterniond &payload_attitude, const Eigen::Vector3d &payload_bodyrate);


    // compute term m_D (m means matrix) and m_D is to compute payload translational dynamic equation
    // void ComputeMatrixMDiMCiMEi(const Eigen::Vector3d &cable_direction, const Eigen::Quaterniond &payload_attitude, const Eigen::Vector3d &attach_point_post);
    void ComputeMatrixMDiMCiMEi(const Eigen::Quaterniond &payload_attitude);

    // compute control inputs for mav for both slack and taut status
    // void ComputeNetWrenchApplied2MAV(const Eigen::Vector3d &attach_point_acc);
    void ComputeNetWrenchApplied2MAV();

    // import control input from controller
    // void InputControllerInput(const double &mav_thrust, const Eigen::Vector3d &mav_torque);
    void InputControllerInput(const double &mav_thrust, const Eigen::Vector3d &mav_torque);
    

    // set initial post of MAV
    void SetMAVInitPost(const Eigen::Vector3d &mav_post);

    // set initial post of MAV that is above payload post with cable being taut
    // input attach point post
    void SetMAVInitPostCableTautWithAttachPointPost(const Eigen::Vector3d &attach_point_init_post);

    // obtain class member variables

    // // obtain attach point torque
    // void GetAttachPointTorque(Eigen::Vector3d &mav_attach_point_torque){ mav_attach_point_torque = mav_attach_point_torque_;};
    inline Wrench attach_point_wrench() const {return mav_attach_point_wrench_;};

    // obtain m_D_i
    // void GetMatrixMDiMCiMEi(Eigen::Matrix3d &m_C_i, Eigen::Matrix3d &m_D_i, Eigen::Matrix3d &m_E_i) const;
    // obtain m_D_i with m_D_i()
    inline Eigen::Matrix3d m_D_i() const {return m_D_i_;};

    // obtain m_C_i with m_C_i()
    inline Eigen::Matrix3d m_C_i() const {return m_C_i_;};

    // obtain m_E_i with m_E_i()
    inline Eigen::Matrix3d m_E_i() const {return m_E_i_;};

    // obtain mass matrix
    Eigen::Matrix3d m_mass_matrix() const;    

    // obtain cable collision status
    inline bool inelasticCollisionStauts() const {return inelastic_collision_;};    

    void SetCollisionStatus(const bool &collision_status);

};
#endif
