#ifndef RIGIDBODY_SIMULATOR_H
#define RIGIDBODY_SIMULATOR_H

#include <iostream>
#include <boost/numeric/odeint.hpp>
#include <Eigen/Dense>
#include <boost/numeric/odeint/external/eigen/eigen.hpp>
#include <cmath>
#include <vector>
#include <array>
#include <cstdio>
#include <algorithm>  // For std::fill
#include "spdlog/spdlog.h"
#include "rotor_tm_sim/base/lib_logger.hpp"
#include "rotor_tm_sim/base/lib_base.hpp"
#include "rotor_tm_sim/base/lib_utils.hpp"

using namespace boost::numeric::odeint;


// typedef Eigen::Matrix<double, 12, 1> object_state;

// using object_state = Eigen::Matrix<double, 13, 1>;
using object_state = std::array<double, 13>;

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

        Eigen::Matrix3d inv_inertia_;


        // simulator setings for an object in 3D
        // state vecgor for a rigid body (13X1) including position, velcity, quaternion, bodyrate, 
        // state_ = [x,     y,      z,      dx,     dy,     dz,     qw,     qx,     qy,     qz,    theta,      psi,    p,      q,      r]
        object_state state_;


        Accs accs_;

        // object acc (3X1 vector) and bodyrate acc (3X1 vector) 
        // Eigen::Vector3d object_acc_;
        // Eigen::Vector3d object_bodyrate_acc_ = Eigen::Vector3d::Zero();

        // rotational dynamic
        // compute dbodyrate in body frame
        // Eigen::Vector3d RotDynac(const Eigen::Vector3d &torque, const Eigen::Matrix3d &Inertia, const Eigen::Vector3d &bodyrate);
        Eigen::Vector3d RotDynac();

        // translation dyanmic
        // compute acceleration in world frame
        Eigen::Vector3d TransDynac();

        void NormalizeQuaternion(object_state& state);

        // compute matrix transforming bodyrate to dEuler
        Eigen::Matrix3d matirxBodyrate2EulerRate(const double &phi, const double &theta);

        Eigen::Vector4d ComputeQuaternionDerivative(const Eigen::Quaterniond &qn, const Eigen::Vector3d &bodyrate);

        bool checkEquilibrium(const object_state& state)  ;

        std::array<double, 3> EigenToArray(const Eigen::Vector3d& vec);

        Eigen::Vector3d ArrayToEigen(const std::array<double, 3>& arr);
        

        // prevent creating instance using none par
        RigidBody();

        // solver ruge_kutta
        runge_kutta4<object_state> stepper_;      
    
        // typedef controlled_runge_kutta<runge_kutta_cash_karp54<object_state>> stepper_type;
        // stepper_type stepper_;

        // typedef rosenbrock4<object_state> stepper_type;
        // typedef default_error_checker<double, array_algebra, default_operations> error_checker_type;

        void RK23Step(const object_state& current_state, const double dt, object_state& next_state);

        bool AdaptiveRK23Step(object_state& state, double& dt, double& current_time);

        // controlled_runge_kutta<stepper_type, error_checker_type> controlled_stepper_; 
        void RK4Step(const object_state& current_state, const double dt, object_state& next_state);

        void AB4Step(const object_state& current_state, const double dt, object_state& next_state);

        std::vector<object_state> previous_derivatives;  // Store derivatives for AB4
        int n_stored_derivatives;

        // typedef rosenbrock4<object_state> stepper_type;

        // // Define the stepper
        // stepper_type stepper_;
        // object_state state_err_; // Error estimate
        // using error_stepper_type = runge_kutta_dopri5<object_state>;
        // using controlled_stepper_type = controlled_runge_kutta<error_stepper_type>;
        // controlled_stepper_type controlled_stepper_;
  
        // runge_kutta_fehlberg78<object_state> stepper_;
        using error_stepper_type = runge_kutta_cash_karp54<object_state>;
        using error_checker_type = default_error_checker<
        double,
        array_algebra,
        default_operations
        >;
        using controlled_stepper_type = controlled_runge_kutta<error_stepper_type, error_checker_type>;

        controlled_stepper_type controlled_stepper;  // Stepper as class member
        double current_step_ = 0;

    public:

        // constructor
        // RigidBody(const double &mass, const Eigen::Matrix3d &m_inertia, const double &step_size);
        RigidBody(const MassProperty &mass_property, const double &step_size);

        // call one step integration
        // make it virtual and final forbids itself to be overloaded in derived classes
        virtual void DoOneStepInt();

        // integration for one step
        // Declare the function call operator to use odeint
        // Make it virtual as devrived classes may have different dynamic models
        virtual void operator()(const object_state &x , object_state &dxdt, const double time); 

        // get drone status information
        // void GetPosition(Eigen::Vector3d &object_position) const;


        // get robot infor
        double mass() const;
        
        Eigen::Matrix3d inertia() const;

        Pose pose() const;

        Vels vels() const;

        Accs accs() const;
        // void GetVel(Eigen::Vector3d &object_vel) const;

        // void GetAcc(Eigen::Vector3d &object_acc) const;

        // void GetBodyrate(Eigen::Vector3d &object_bodyrate)const;

        // void GetAttitude(Eigen::Quaterniond &object_attitude) const ;

        // void GetBodyRateAcc(Eigen::Vector3d &object_bodyrate_acc) const;

        object_state state() const;

        Eigen::VectorXd stateEigen() const;

        // void GetCurrentTimeStep(double &current_time);

        double timestep() const;

        // input for a rigid body
        // void InputForce(const Eigen::Vector3d &force); // force is a force vector in world frame

        // void InputTorque(const Eigen::Vector3d &torque); //torque is a torque vector in body frame
        void InputWrench(const Wrench &mav_wrench);

        // set initial position
        void SetInitialPost(const Eigen::Vector3d &initial_post);    

        // set post in the world frame
        void SetPost(const Eigen::Vector3d &object_post);

        // set initial attitude in Euler Angles
        void SetInitialAttitude(const double &phi, const double &theta, const double &psi);  

        void SetAttitude(const Eigen::Quaterniond &qa); 

        // set vel in the world frame
        void SetLinearVel(const Eigen::Vector3d &object_vel);

        // set acc in the world frame
        void SetLinearAcc(const Eigen::Vector3d &object_acc);

        // set bodyrate in the body frame
        void SetBodyrate(const Eigen::Vector3d &object_bodyrate);

        // set bodyrate_acc 
        void SetAngularAcc(const Eigen::Vector3d &object_bodyrate_acc);

        // inline void GetMass(double &mass) const {mass= mass_;};

        // inline void GetInertia(Eigen::Matrix3d &m_inertia) const { m_inertia = m_inertia_;};

        // inline void GetMassProperty(MassProperty &mass_property) const { mass_property = mass_property_;};


        void SetStatesZeros();


        // TODO:ZLi move physical and mathematical funcs and consts to another file
        // const double gravity_ = 9.8;    

        // transfer a vector to tis skew sym matrix
        Eigen::Matrix3d TransVector3d2SkewSymMatrix(Eigen::Vector3d vector); 

        // transfer deg to radian
        inline double deg2rad(double deg) {return deg * M_PI / 180.0;};


        ~RigidBody();


        protected:  // Change from private to protected

        std::unique_ptr<MyLogger> ptr_logger; // Use unique_ptr to avoid memory leak

        bool logging_enabled = false;

        public:
        
        // Add a method to enable logging when needed
        virtual void enable_logging(const std::string& log_file);
        
        // Check if logging is enabled
        bool has_logging() const;
        

        // void write_log_info(const std::string& message) const;
        
        // void write_log_debug(const std::string& message) const;
        
        // void write_log_warn(const std::string& message) const;
        
        // void write_log_error(const std::string& message) const;

        std::shared_ptr<spdlog::logger> get_spdlog_logger() const;
};
#endif
