
#ifndef LIBBASW_SIMULATOR_H
#define LIBBASW_SIMULATOR_H

#include <iostream>
#include <Eigen/Dense>
#include <cmath>


struct MassProperty {
        double mass;
        Eigen::Matrix3d inertia;
        };

struct Vels{
        Eigen::Vector3d linear_vel  = Eigen::Vector3d::Zero(); 
        Eigen::Vector3d  bodyrate = Eigen::Vector3d::Zero();         

        Vels(const Eigen::Vector3d& l_vel, const Eigen::Vector3d& a_rate) : linear_vel(l_vel), bodyrate(a_rate) {};
        Vels() = default;
        };  

struct Accs{      
        Eigen::Vector3d linear_acc  = Eigen::Vector3d::Zero(); 
        Eigen::Vector3d angular_acc  = Eigen::Vector3d::Zero(); 

        Accs(const Eigen::Vector3d& l_acc, const Eigen::Vector3d& a_acc) : linear_acc(l_acc), angular_acc(a_acc) {};
        Accs() = default;
        };  


struct Pose{
        Eigen::Vector3d post  = Eigen::Vector3d::Zero(); 
        Eigen::Quaterniond att = Eigen::Quaterniond::Identity();

        Pose(const Eigen::Vector3d& p, const Eigen::Quaterniond& q) : post(p), att(q) {};
        Pose() = default;
};

struct Wrench{
        Eigen::Vector3d force = Eigen::Vector3d::Zero(); 
        Eigen::Vector3d torque = Eigen::Vector3d::Zero();

        // Overload the + operator for Wrench
        Wrench operator+(const Wrench& other) const {
                Wrench result;
                result.force = this->force + other.force;   // Add the forces
                result.torque = this->torque + other.torque; // Add the torques
                return result;
        }
        };


struct ThrustWrench{
        double thrust = 0;
        Eigen::Vector3d torque = Eigen::Vector3d::Zero();

        ThrustWrench(const double& t, const Eigen::Vector3d& tau) : thrust(t), torque(tau) {};
        ThrustWrench() = default;
};


#endif        