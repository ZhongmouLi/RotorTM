#ifndef CABLE_SIMULATOR_H
#define CABLE_SIMULATOR_H

#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <vector>

class Cable
{
    friend class UAVCable; // Declare UAVCable as a friend of Cable

    private:

        // length
        double length_;

        // var indicates taut of slack of cable
        bool taut_ = true;

        // tention force in world frame'
        Eigen::Vector3d tension_force_;

        // body rate
        Eigen::Vector3d body_rate_;

        // cable position in world frame
        Eigen::Vector3d xi_;

        Cable();

        // threshold for zero
        const double k_threshold = 1e-3;

    public:


        Cable(const double &length);


        // compute cable direction in world frame
        // input:   posts of attach points and drones
        // methods: eq (16)
        // output: xi_        
        void ComputeCableDirection(const Eigen::Vector3d &attachpoint_post, const Eigen::Vector3d &robot_post);

        // compute cable tension force in world frame
        // input:   mav mass, mav thrust force (3X1 vector) in world frame, acc of attach point (3X1 vector) in world frame 
        // methods: eq (19)
        // output: tension_force_  
        void ComputeCableTensionForce(const double &robot_mass, const Eigen::Vector3d &mav_thrust_force , const Eigen::Vector3d &xi, const Eigen::Vector3d &attach_point_acc);


        // compute tension force in world frame

        // change taut status of cable
        // input:   (1) posts of attach points and drones
        //          (2) vels of attach points and drones
        // methods: equation (34)-(38)
        // output: change boolen var taut_
        void CheckTaut(const Eigen::Vector3d &attachpoint_post, const Eigen::Vector3d &robot_post, const Eigen::Vector3d &attachpoint_vel, const Eigen::Vector3d &robot_vel);

        // obtain cable direction
        inline void GetCableDirection(Eigen::Vector3d &xi) {xi= xi_;};

        // obtain cable taut status
        inline void GetCableTautStatus(bool &cable_taut) {cable_taut = taut_;};

        // obtain cable tension force
        inline void GetCableTensionForce(Eigen::Vector3d &cable_tension_force){cable_tension_force = tension_force_;};

        inline void GetCableLength(double &cable_length){cable_length = length_;};
};



#endif
