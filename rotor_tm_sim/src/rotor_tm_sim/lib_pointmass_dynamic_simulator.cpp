#include "rotor_tm_sim/lib_pointmass_dynamic_simulator.hpp"


PointMassDynamicSimulator::PointMassDynamicSimulator(const double &mass, const double &step_size): mass_(mass), step_size_(step_size)
{
    ptmas_state_.setZero();
};



//
void PointMassDynamicSimulator::operator() (const pointmass_state &x , pointmass_state &dxdt, const double time)
{
    // define post_v to obtain input state X
    // X = [post_v] = [position vel] 6X1
    // Eigen::VectorXd post_v(6);

    // // define dpost_v for output dX
    // Eigen::VectorXd dpost_v(6);

    // // get postion  from input
    // post_v = x;


    // translation in world frame
    // P = [x,y,z,dx, dy, dz]
    // dP = [dx, dy, dz, ddx, ddy, ddz]
    // dpost_v(0) = post_v(3);
    // dpost_v(1) = post_v(4);
    // dpost_v(2) = post_v(5);

    // // [ddx ddy ddz] = (F-mg)/m
    // dpost_v.tail(3) = ptmasTransDynac(force_, mass_, gravity_);

    // // assign
    // dxdt =  dpost_v;


    // translation in world frame
    // x = [x,y,z,dx, dy, dz]
    // dxdt = [dx, dy, dz, ddx, ddy, ddz]
    dxdt(0) = x(3);
    dxdt(1) = x(4);
    dxdt(2) = x(5);

    // [ddx ddy ddz] = (F-mg)/m
    dxdt.tail(3) = ptmasTransDynac(force_, mass_, gravity_);
}


// interface for calling dynamic simulation for point mass
void PointMassDynamicSimulator::doOneStepInt()
{

    this->stepper_.do_step(*this, ptmas_state_, current_step_, step_size_);

    // acculate simulation steps
    current_step_ = current_step_ + step_size_;

    // // assign states to position and velocity
    // assignPtMasState(ptmas_state_);

};
  
void PointMassDynamicSimulator::setInitialPost(const Eigen::Vector3d &initial_position)
{

    ptmas_state_.head<3>() = initial_position;
};



Eigen::Vector3d PointMassDynamicSimulator::ptmasTransDynac(const Eigen::Vector3d &force, const double &mass, const double &gravity)
{   
    Eigen::Vector3d acc = Eigen::Vector3d::Zero();

    acc = (force-mass*gravity*Eigen::Vector3d::UnitZ())/mass;

    return acc;
}


// void PointMassDynamicSimulator::assignPtMasState(const pointmass_state &ptmas_state)
// {
//     // done_state = [x,y,z,dx, dy, dz]
//     post_ = ptmas_state.head<3>();
//     vel_ = ptmas_state.segment<3>(3);
// };  
 

void PointMassDynamicSimulator::setVel(const Eigen::Vector3d &ptmas_vel)
{
    ptmas_state_.tail<3>() = ptmas_vel;
}


void PointMassDynamicSimulator::getPosition(Eigen::Vector3d &ptmas_position)
{
    ptmas_position = ptmas_state_.head<3>();
};

void PointMassDynamicSimulator::getVel(Eigen::Vector3d &ptmas_vel)
{
    ptmas_vel = ptmas_state_.tail<3>();
};


void PointMassDynamicSimulator::inputForce(const Eigen::Vector3d &force)
{
    force_ = force;
};


void PointMassDynamicSimulator::getCurrentTimeStep(double &current_time)
{
    current_time = current_step_;
};
