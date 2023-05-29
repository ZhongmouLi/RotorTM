#ifndef QUADROTORDYNQMIC_SIMULATOR_H
#define QUADROTORDYNQMIC_SIMULATOR_H

#include <iostream>
#include <boost/numeric/odeint.hpp>
#include <Eigen/Dense>
// #include <boost/numeric/odeint/external/eigen/eigen_algebra.hpp>
#include <boost/numeric/odeint/external/eigen/eigen.hpp>
#include <cmath>
#include <vector>

// #include <functional>
// namespace pl = std::placeholders;

using namespace boost::numeric::odeint;


typedef Eigen::Matrix<double, 12, 1> quadrotor_state;


class QuadrotorDynamicSimulator
{
    private:
        // quadrotor parameters
        double mass_;
        
        Eigen::Matrix3d m_inertia_;

        const double gravity_ = 9.8;

        Eigen::Vector3d post_;  // position vector in world frame
        Eigen::Vector3d vel_;   // vel vector in world frame
        Eigen::Vector3d bodyrate_;  // bodyrate vector in body frame
        Eigen::Quaterniond attitude_;   // attitude in quaternion

        // dynamic inputs
        // thrust force in world frame
        Eigen::Vector3d thrust_;
        // torque in body frame
        Eigen::Vector3d torque_;

        // int parameter
        double step_size_;
        double current_step_ = 0;

        // simulator seetigs for quadrotor
        // state vecgor for a quadrotor (12X1) including position, velcity, bodyrate, euler angle
        quadrotor_state done_state_;

        // solver ruge_kutta
        runge_kutta4<quadrotor_state> stepper_;


        // rotational dynamic
        // compute d_bodyrate
        Eigen::Vector3d quadRotDynac(const Eigen::Vector3d &torque, const Eigen::Matrix3d &Inertia, const Eigen::Vector3d &bodyrate);

        // translation dyanmic
        Eigen::Vector3d quadTransDynac(const Eigen::Vector3d &Thurst, const double &mass, const double &gravity);

        // bodyrate to d_Euler
        Eigen::Vector3d quadBodyrate2Eulerrate(const Eigen::Vector3d &bodyrate, const Eigen::Matrix3d &trans_matrix);

        // matrix transforming bodyrate to d_Euler
        Eigen::Matrix3d quadTransMatrix(const double &phi, const double &theta, const double &psi);

        // dynamic model of quadrotor
        void rhs(const quadrotor_state &x , quadrotor_state &dxdt, const double time);

        // obtain quadrotor state
        void assignDroneState(const quadrotor_state &done_state);
        
        // transfer deg to radian
        inline double deg2rad(double deg) {return deg * M_PI / 180.0;};

        // prevent creating instance using none par
        QuadrotorDynamicSimulator();

        

    public:

        // constructor
        QuadrotorDynamicSimulator(const double &mass, const Eigen::Matrix3d &m_inertia, const double &step_size);

        // do one step integration
        void doOneStepInt();
        void operator()(const quadrotor_state &x , quadrotor_state &dxdt, const double time); // Declare the function call operator

        // get drone status information
        void getPosition(Eigen::Vector3d &mav_position);

        void getVel(Eigen::Vector3d &mav_vel);

        void getBodyrate(Eigen::Vector3d &mav_bodyrate);

        void getAttitude(Eigen::Quaterniond &mav_attitude);

        void getCurrentTimeStep(double &current_time);

        // input for quadrotor simulator instance

        void inputForce(const Eigen::Vector3d &mav_thrust_force); // mav_thrust_force is thrust force vector in world frame

        void inputThurst(const double &mav_thrust); // mav_thrust is the norm of thrust force

        void inputTorque(const Eigen::Vector3d &mav_torque); //mav_torque is torque vector in body frame

        // set vel from outside
        void setVel(const Eigen::Vector3d &mav_vel);

};
#endif
