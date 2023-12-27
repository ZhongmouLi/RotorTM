#ifndef QUADROTORDYNQMIC_SIMULATOR_H
#define QUADROTORDYNQMIC_SIMULATOR_H

#include <iostream>
#include <boost/numeric/odeint.hpp>
#include <Eigen/Dense>
// #include <boost/numeric/odeint/external/eigen/eigen_algebra.hpp>
#include <boost/numeric/odeint/external/eigen/eigen.hpp>
#include <cmath>
#include <vector>


using namespace boost::numeric::odeint;


typedef Eigen::Matrix<double, 12, 1> quadrotor_state;


class Quadrotor
{
    private:
        // quadrotor parameters
        double mass_;
        
        Eigen::Matrix3d m_inertia_;

        const double gravity_ = 9.8;

        // dynamic inputs
        // thrust force in world frame
        Eigen::Vector3d thrust_;
        // torque in body frame
        Eigen::Vector3d torque_;

        // int parameter
        double step_size_;
        double current_step_ = 0;

        // simulator setings for quadrotor
        // state vecgor for a quadrotor (12X1) including position, velcity, euler angle, bodyrate, 
        // done_state_ = [x,     y,      z,      dx,     dy,     dz,     phi,    theta,      psi,    p,      q,      r]
        quadrotor_state done_state_;

        // solver ruge_kutta
        runge_kutta4<quadrotor_state> stepper_;


        // rotational dynamic
        // compute dbodyrate in body frame
        Eigen::Vector3d quadRotDynac(const Eigen::Vector3d &torque, const Eigen::Matrix3d &Inertia, const Eigen::Vector3d &bodyrate);

        // translation dyanmic
        // compute acceleration in world frame
        Eigen::Vector3d quadTransDynac(const Eigen::Vector3d &Thurst, const double &mass, const double &gravity);

        // compute matrix transforming bodyrate to dEuler
        Eigen::Matrix3d matirxBodyrate2EulerRate(const double &phi, const double &theta, const double &psi);
        

        // transfer deg to radian
        inline double deg2rad(double deg) {return deg * M_PI / 180.0;};

        // prevent creating instance using none par
        Quadrotor();

        

    public:

        // constructor
        Quadrotor(const double &mass, const Eigen::Matrix3d &m_inertia, const double &step_size);

        // call one step integration
        void doOneStepInt();

        // integration for one step
        // Declare the function call operator to use odeint
        void operator()(const quadrotor_state &x , quadrotor_state &dxdt, const double time); 

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

        // set initial position for quadrotor
        void setInitialPost(const Eigen::Vector3d &initial_post);        

        // transfer a vector to tis skew sym matrix
        Eigen::Matrix3d TransVector3d2SkewSymMatrix(Eigen::Vector3d vector); 

        inline void GetMass(double &mass) const {mass= mass_;};

        inline void GetInertia(Eigen::Matrix3d &m_inertia) const { m_inertia = m_inertia_;};    

        inline void GetThrustForce(Eigen::Vector3d &mav_thrsut_force) const {mav_thrsut_force = thrust_;};      
};
#endif
