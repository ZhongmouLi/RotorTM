#ifndef POINTMASS_SIMULATOR_H
#define POINTMASS_SIMULATOR_H

#include <iostream>
#include <boost/numeric/odeint.hpp>
#include <Eigen/Dense>
// #include <boost/numeric/odeint/external/eigen/eigen_algebra.hpp>
#include <boost/numeric/odeint/external/eigen/eigen.hpp>
#include <cmath>
#include <vector>



using namespace boost::numeric::odeint;


typedef Eigen::Matrix<double, 6, 1> pointmass_state;

class PointMass
{
    private:
        // point mass parameters
        double mass_;
        
        // const para
        const double gravity_ = 9.8;

        // Eigen::Vector3d post_;  // position vector in world frame
        // Eigen::Vector3d vel_;   // vel vector in world frame


        // dynamic inputs
        Eigen::Vector3d force_;

        // int parameter
        double step_size_;
        double current_step_ = 0;

        // simulator settigs for pointmass
        // state vector for a point mass (6X1) including position, velcity
        pointmass_state ptmas_state_;

        // solver ruge_kutta
        runge_kutta4<pointmass_state> stepper_;


        // translation dyanmic
        Eigen::Vector3d ptmasTransDynac(const Eigen::Vector3d &force, const double &mass, const double &gravity);


        // dynamic model of point mass
        void rhs(const pointmass_state &x , pointmass_state &dxdt, const double time);

        // // obtain pointmass state
        // void assignPtMasState(const pointmass_state &ptmas_state);
        
        // transfer deg to radian
        inline double deg2rad(double deg) {return deg * M_PI / 180.0;};

        // prevent creating instance using none par
        PointMass();

        

    public:

        // constructor
        PointMass(const double &mass, const double &step_size);

        // do one step integration
        void doOneStepInt();

        void operator()(const pointmass_state &x , pointmass_state &dxdt, const double time); // Declare the function call operator

        // get pointmass status information
        void getPosition(Eigen::Vector3d &ptmas_position);

        void getVel(Eigen::Vector3d &ptmas_vel);


        void getCurrentTimeStep(double &current_time);

        // input for quadrotor simulator instance

        void inputForce(const Eigen::Vector3d &force); // force is 3X1 vector in world frame

        // interface to set vel 
        void setVel(const Eigen::Vector3d &ptmas_vel);

        // set initial position at the beginging of simulation 
        void setInitialPost(const Eigen::Vector3d &initial_position);        
};
#endif
