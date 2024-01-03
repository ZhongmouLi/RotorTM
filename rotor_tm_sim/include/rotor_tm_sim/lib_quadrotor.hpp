#ifndef QUADROTORDYNQMIC_SIMULATOR_H
#define QUADROTORDYNQMIC_SIMULATOR_H

#include <iostream>
#include <boost/numeric/odeint.hpp>
#include <Eigen/Dense>
// #include <boost/numeric/odeint/external/eigen/eigen_algebra.hpp>
#include <boost/numeric/odeint/external/eigen/eigen.hpp>
#include <cmath>
#include <vector>
#include "lib_rigidbody.hpp"


using namespace boost::numeric::odeint;

class Quadrotor: public RigidBody
{

    private:

        double mav_thrust_;

        Eigen::Vector3d mav_thrust_force_;

    public:

        // constructor
        Quadrotor(const double &mass, const Eigen::Matrix3d &m_inertia, const double &step_size);

        // call one step integration
        // void DoOneStepInt();

        // integration for one step
        // Declare the function call operator to use odeint
        // void operator()(const mav_state &x , mav_state &dxdt, const double time) final; 

        void InputThurst(const double &mav_thrust); // mav_thrust is the norm of thrust force

        // set vel from outside

        inline void GetThrustForce(Eigen::Vector3d &mav_thrsut_force) const {mav_thrsut_force = mav_thrust_force_;};      
};
#endif
