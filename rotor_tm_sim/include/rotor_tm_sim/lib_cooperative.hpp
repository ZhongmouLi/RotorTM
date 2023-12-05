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



class Cooperative{

    private:

        // vector of cable instances (shared pointers)
       std::vector<std::shared_ptr<Cable>> v_cables_ptr_; 

       // vectir of uav instances (shared pointers)
       std::vector<std::shared_ptr<RigidBody>> v_drones_ptr_;

        // group of cable and uavs 
        std::vector<std::pair<std::shared_ptr<RigidBody>, std::shared_ptr<Cable>>> v_pairs_drone_cable_;
        
        
        // payload instance (shared ptr)   
        std::shared_ptr<RigidBody> payload_ptr_;

        Cooperative();

        // compute the projection of robot vel perpendicular to the cable
        // input: drone vel (3X1 vector) and cable direction (3X1 vector)
        // output:: drone vel projection that is perpendicular to cable (3X1 vector)
        Eigen::Vector3d CalVelProjPerpendicularCable(const Eigen::Vector3d drone_vel, const Eigen::Vector3d &cable_direction);

        void UpdateVelCollidedOneUAVPayload();

    public:

    Cooperative(const std::vector<Eigen::Vector3d> &v_attach_point_post_bf, const double &mass, const Eigen::Matrix3d &m_inertia, const double &step_size);

    // check collsions for every cable
    void CheckCollision();

    // 
    // method: eq (39)-(42)
    void UpdateVelsCollidedUAVsPayload();


};


#endif