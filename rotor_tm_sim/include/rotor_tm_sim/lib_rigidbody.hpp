#ifndef RIGIDBODY_SIMULATOR_H
#define RIGIDBODY_SIMULATOR_H

#include <iostream>
#include <boost/numeric/odeint.hpp>
#include <Eigen/Dense>
#include <boost/numeric/odeint/external/eigen/eigen.hpp>
#include <cmath>
#include <vector>


using namespace boost::numeric::odeint;


// typedef Eigen::Matrix<double, 12, 1> object_state;

using object_state = Eigen::Matrix<double, 12, 1>;


struct MassProperty {
        double mass;
        Eigen::Matrix3d inertia;};

struct Kinematics{
        Eigen::Vector3d vel;
        Eigen::Vector3d bodyrate;         
        Eigen::Vector3d acc;
        Eigen::Vector3d bodyrate_acc;
        };  

struct Pose{
        Eigen::Vector3d post;  
        Eigen::Quaterniond att;
}

struct Wrench{
        Eigen::Vector3d force;
        Eigen::Vector3d torque;
        };



class RigidBody
{
   public:
        // quadrotor parameters
        // double mass_;
        
        // Eigen::Matrix3d m_inertia_;
        MassProperty mass_property_;

        // int parameter
        double step_size_;

        // dynamic inputs
        // force in world frame
        // Eigen::Vector3d force_;
        // // torque in body frame
        // Eigen::Vector3d torque_;
        Wrench input_wrench_;

        Pose pose_;

        Kinematics kinematics_;

        // simulator setings for an object in 3D
        // state vecgor for a rigid body (12X1) including position, velcity, euler angle, bodyrate, 
        // state_ = [x,     y,      z,      dx,     dy,     dz,     phi,    theta,      psi,    p,      q,      r]
        object_state state_;

        // object acc (3X1 vector) and bodyrate acc (3X1 vector) 
        Eigen::Vector3d object_acc_;
        Eigen::Vector3d object_bodyrate_acc_ = Eigen::Vector3d::Zero();

        // rotational dynamic
        // compute dbodyrate in body frame
        Eigen::Vector3d RotDynac(const Eigen::Vector3d &torque, const Eigen::Matrix3d &Inertia, const Eigen::Vector3d &bodyrate);

        // translation dyanmic
        // compute acceleration in world frame
        Eigen::Vector3d TransDynac(const Eigen::Vector3d &Thurst, const double &mass, const double &gravity);

        // compute matrix transforming bodyrate to dEuler
        Eigen::Matrix3d matirxBodyrate2EulerRate(const double &phi, const double &theta);

        // prevent creating instance using none par
        RigidBody();

        // solver ruge_kutta
        runge_kutta4<object_state> stepper_;        

        double current_step_ = 0;
        
    public:

        // constructor
        // RigidBody(const double &mass, const Eigen::Matrix3d &m_inertia, const double &step_size);
        RigidBody(const MassProperty &mass_property, const double &step_size);

        // call one step integration
        // make it virtual and final forbids itself to be overloaded in derived classes
        virtual void DoOneStepInt() final;

        // integration for one step
        // Declare the function call operator to use odeint
        // Make it virtual as devrived classes may have different dynamic models
        virtual void operator()(const object_state &x , object_state &dxdt, const double time); 

        // get drone status information
        void GetPosition(Eigen::Vector3d &object_position) const;

        void GetVel(Eigen::Vector3d &object_vel) const;

        void GetAcc(Eigen::Vector3d &object_acc) const;

        void GetBodyrate(Eigen::Vector3d &object_bodyrate)const;

        void GetAttitude(Eigen::Quaterniond &object_attitude) const ;

        void GetBodyRateAcc(Eigen::Vector3d &object_bodyrate_acc) const;

        void GetState(object_state &state) const;

        void GetCurrentTimeStep(double &current_time);

        // input for a rigid body
        // void InputForce(const Eigen::Vector3d &force); // force is a force vector in world frame

        // void InputTorque(const Eigen::Vector3d &torque); //torque is a torque vector in body frame
        void InputWrench(const Wrench &mav_wrench);

        // set initial position
        void SetInitialPost(const Eigen::Vector3d &initial_post);    

        // set initial attitude in Euler Angles
        void SetInitialAttitude(const double &phi, const double &theta, const double &psi);   

        // set vel in the world frame
        void SetVel(const Eigen::Vector3d &object_vel);

        // set acc in the world frame
        void SetAcc(const Eigen::Vector3d &object_acc);

        // set bodyrate in the body frame
        void SetBodyrate(const Eigen::Vector3d &object_bodyrate);

        // set bodyrate_acc 
        void SetBodyrateAcc(const Eigen::Vector3d &object_bodyrate_acc);

        // inline void GetMass(double &mass) const {mass= mass_;};

        // inline void GetInertia(Eigen::Matrix3d &m_inertia) const { m_inertia = m_inertia_;};

        inline void GetMassProperty(MassProperty &mass_property) const { mass_property = mass_property_;};


        void SetStatesZeros();


        // TODO:ZLi move physical and mathematical funcs and consts to another file
        const double gravity_ = 9.8;    

        // transfer a vector to tis skew sym matrix
        Eigen::Matrix3d TransVector3d2SkewSymMatrix(Eigen::Vector3d vector); 

        // transfer deg to radian
        inline double deg2rad(double deg) {return deg * M_PI / 180.0;};


        virtual ~RigidBody(){};

};
#endif
