#ifndef PAYLOAD_SIMULATOR_H
#define PAYLOAD_SIMULATOR_H

#include <iostream>
#include <boost/numeric/odeint.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include "lib_rigidbody.hpp"
#include "lib_uav_cable.hpp"


class Payload: public RigidBody{
    private:

        // vector of attach point positions in body frame
        // it is fixed during initilisation
       std::vector<Eigen::Vector3d> v_attach_point_posts_body_frame_; 

       // vector of attach point positions in world frame
       // a vector of 3d vectors and each of them represent an attach point's post
       std::vector<Eigen::Vector3d> v_attach_points_posts_;

       // vector of attach point vels     
       std::vector<Eigen::Vector3d> v_attach_points_vels_; 

        // vector of attach point accs     
       std::vector<Eigen::Vector3d> v_attach_points_accs_; 

       // net force applied by drones
       Eigen::Vector3d drones_net_force_;

       Eigen::Matrix3d m_mass_matrix_;

       Eigen::Matrix3d m_D_;

       Eigen::Matrix3d m_E_;

       Eigen::Matrix3d m_F_;

       // net torque applied by drones
       Eigen::Vector3d drones_net_torque_;

        // ComputeMatrixJi is a function for updating vels after collision
        // compute matrix Ji in Eq46
        Eigen::MatrixXd ComputeMatrixJi(const Eigen::Vector3d &cable_direction, const Eigen::Quaterniond &payload_attitude, const Eigen::Vector3d &attach_point_body_frame);

        // ComputeVectorbi is a function for updating vels after collision    
        // compute vector bi in Eq46
        Eigen::VectorXd ComputeVectorbi(const double &drone_mass, const Eigen::Vector3d &drone_vel, const Eigen::Vector3d &cable_direction, const Eigen::Quaterniond &payload_attitude, const Eigen::Vector3d &attach_point_body_frame);

        // translational dynamic model
        Eigen::Vector3d ComputeTransDynamics(const Eigen::Vector3d &drones_net_forces, const Eigen::Matrix3d &mass_matrix, const Eigen::Vector3d term_payload_bodyrate_acc,  const Eigen::Vector3d payload_bodyrate_acc);

        // rotational dynamic model
        Eigen::Vector3d ComputeRotDynamics(const Eigen::Vector3d &drones_net_forces, const Eigen::Vector3d &drones_net_torques, const Eigen::Matrix3d &m_mass_matrix, const Eigen::Vector3d &payload_bodyrate, const Eigen::Matrix3d &m_C, const Eigen::Matrix3d &m_D, const Eigen::Matrix3d &m_E);

        Payload();

    public:

    Payload(const std::vector<Eigen::Vector3d> &v_attach_point_post_bf, const double &mass, const Eigen::Matrix3d &m_inertia, const double &step_size);

    // input mass matrix  
    void InputMassMatrix(const Eigen::Matrix3d &m_mass_matrix);

    // input translational dynamic model inputs: drones' net force to the payload and term m_D
    void InputDronesNetForces(const Eigen::Vector3d &drones_net_force, const Eigen::Matrix3d &m_D);

    // input rotational dynamic model inputs: drones' net force to the payload and term m_D
    void InputDronesNetTorques(const Eigen::Vector3d &drones_net_torque, const Eigen::Matrix3d &m_C, const Eigen::Matrix3d &m_E);

    // void CalVel4AttachPoint();      

    // ComputeAttachPointsPostVel computes post and vels of attach points
    void ComputeAttachPointsPostVel();

    void ComputeAttachPointAccs();

    // GetAttachPointsPos obtain (i+1)th attach points post
    void GetOneAttachPointPost(size_t i, Eigen::Vector3d attach_point_post) const;

    // GetAttachPointsVel obtain (i+1)th attach points vel
    void GetAttachPointVel(size_t i, Eigen::Vector3d attach_point_vel) const;    

    // void UpdateVelCollided(const std::vector<std::shared_ptr<UAVCable>> v_drone_cable_ptr);

    void UpdateVelCollided(const std::vector<UAVCable> &v_drone_cable);
};

#endif
