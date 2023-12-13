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
       std::vector<Eigen::Vector3d> v_attach_point_posts_;

       // vector of attach point vels     
       std::vector<Eigen::Vector3d> v_attach_point_vels_; 

        // vector of attach point accs     
       std::vector<Eigen::Vector3d> v_attach_point_accs_; 

        // ComputeMatrixJi is a function for updating vels after collision
        // compute matrix Ji in Eq46
        Eigen::MatrixXd ComputeMatrixJi(const Eigen::Vector3d &cable_direction, const Eigen::Quaterniond &payload_attitude, const Eigen::Vector3d &attach_point_body_frame);

        // ComputeVectorbi is a function for updating vels after collision    
        // compute vector bi in Eq46
        Eigen::VectorXd ComputeVectorbi(const double &drone_mass, const Eigen::Vector3d &drone_vel, const Eigen::Vector3d &cable_direction, const Eigen::Quaterniond &payload_attitude, const Eigen::Vector3d &attach_point_body_frame);

        Payload();

    public:

    Payload(const std::vector<Eigen::Vector3d> &v_attach_point_post_bf, const double &mass, const Eigen::Matrix3d &m_inertia, const double &step_size);

    // void CalPost4AttachPoint();

    // void CalVel4AttachPoint();      

    // UpdateMotion4AttachPoint computes acc of attach points
    void UpdateMotion4AttachPoint();

    void ComputeAttachPointAccs();

    void getAttachPointPost(std::vector<Eigen::Vector3d> &v_attach_point_post) const;

    void UpdateVelCollided(const std::vector<std::shared_ptr<UAVCable>> v_drone_cable_ptr);

};

#endif
