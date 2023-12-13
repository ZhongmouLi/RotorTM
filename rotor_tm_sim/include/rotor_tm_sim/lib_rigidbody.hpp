#ifndef RIGIDBODY_SIMULATOR_H
#define RIGIDBODY_SIMULATOR_H

#include <iostream>
#include <boost/numeric/odeint.hpp>
#include <Eigen/Dense>
#include <boost/numeric/odeint/external/eigen/eigen.hpp>
#include <cmath>
#include <vector>


using namespace boost::numeric::odeint;


typedef Eigen::Matrix<double, 12, 1> object_state;


class RigidBody
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

        // simulator setings for an object in 3D
        // state vecgor for a rigid body (12X1) including position, velcity, euler angle, bodyrate, 
        // done_state_ = [x,     y,      z,      dx,     dy,     dz,     phi,    theta,      psi,    p,      q,      r]
        object_state done_state_;

        Eigen::Vector3d object_acc_;
        Eigen::Vector3d object_bodyrate_acc;


        // solver ruge_kutta
        runge_kutta4<object_state> stepper_;

        // rotational dynamic
        // compute dbodyrate in body frame
        Eigen::Vector3d rotDynac(const Eigen::Vector3d &torque, const Eigen::Matrix3d &Inertia, const Eigen::Vector3d &bodyrate);

        // translation dyanmic
        // compute acceleration in world frame
        Eigen::Vector3d transDynac(const Eigen::Vector3d &Thurst, const double &mass, const double &gravity);

        // compute matrix transforming bodyrate to dEuler
        Eigen::Matrix3d matirxBodyrate2EulerRate(const double &phi, const double &theta, const double &psi);
        

        // transfer deg to radian
        inline double deg2rad(double deg) {return deg * M_PI / 180.0;};

        // prevent creating instance using none par
        RigidBody();

        

    public:

        // constructor
        RigidBody(const double &mass, const Eigen::Matrix3d &m_inertia, const double &step_size);

        // call one step integration
        void doOneStepInt();

        // integration for one step
        // Declare the function call operator to use odeint
        void operator()(const object_state &x , object_state &dxdt, const double time); 

        // get drone status information
        void getPosition(Eigen::Vector3d &object_position);

        void getVel(Eigen::Vector3d &object_vel);

        void getBodyrate(Eigen::Vector3d &object_bodyrate);

        void getAttitude(Eigen::Quaterniond &object_attitude);

        void getCurrentTimeStep(double &current_time);

        // input for a rigid body
        void inputForce(const Eigen::Vector3d &force); // force is a force vector in world frame

        void inputTorque(const Eigen::Vector3d &torque); //torque is a torque vector in body frame

        // set initial position for quadrotor
        void setInitialPost(const Eigen::Vector3d &initial_post);        

        // set vel in the world frame
        void setVel(const Eigen::Vector3d &object_vel);

        // transfer a vector to tis skew sym matrix
        Eigen::Matrix3d TransVector3d2SkewSymMatrix(Eigen::Vector3d vector); 

        inline void GetMass(double &mass) const {mass= mass_;};

        inline void GetInertia(Eigen::Matrix3d &m_inertia) const { m_inertia = m_inertia_;};

        inline void GetAcc(Eigen::Matrix3d &object_acc) const { object_acc = object_acc_;};
};
#endif
