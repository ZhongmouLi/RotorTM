#ifndef PAYLOAD_SIMULATOR_H
#define PAYLOAD_SIMULATOR_H

#include <iostream>
#include <boost/numeric/odeint.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include "rotor_tm_sim/lib_rigidbody.hpp"



// typedef Eigen::Matrix<double, 12, 1> object_state;


class Payload: public RigidBody{
    private:

        // vector of attach point positions in body frame
        // it is fixed during initilisation
       std::vector<Eigen::Vector3d> v_attach_point_posts_body_frame_; 

       // vectir of attach point positions in world frame
       // a vector of 3d vectors and each of them represent an attach point's post
       std::vector<Eigen::Vector3d> v_attach_point_posts_;


       // vectir of attach point vels     
       std::vector<Eigen::Vector3d> v_attach_point_vels_; 

        Payload();
    public:

    Payload(const std::vector<Eigen::Vector3d> &v_attach_point_post, const double &mass, const Eigen::Matrix3d &m_inertia, const double &step_size);

    void CalPost4AttachPoint();

    void CalVel4AttachPoint();      

    void UpdateMotion4AttachPoint();

};
#endif
