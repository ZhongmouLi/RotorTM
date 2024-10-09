#ifndef QUADROTORDYNQMIC_SIMULATOR_H
#define QUADROTORDYNQMIC_SIMULATOR_H

#include <iostream>
#include <boost/numeric/odeint.hpp>
#include <Eigen/Dense>
// #include <boost/numeric/odeint/external/eigen/eigen_algebra.hpp>
#include <boost/numeric/odeint/external/eigen/eigen.hpp>
#include <cmath>
#include <vector>
#include "rotor_tm_sim/base/lib_rigidbody.hpp"


using namespace boost::numeric::odeint;

class Quadrotor: public RigidBody
{

    private:

        double mav_thrust_;

        Eigen::Vector3d mav_torque_;

    public:

        // constructor
        Quadrotor(const MassProperty &mass_property, const double &step_size);

        void InputDroneThrustTorque(const double &mav_thrust, const Eigen::Vector3d &mav_torque);
        // set vel from outside

        // inline void GetThrustForce(Eigen::Vector3d &mav_thrsut_force) const {mav_thrsut_force = mav_thrust_force_;};      
};
#endif
