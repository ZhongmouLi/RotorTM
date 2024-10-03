#include "rotor_tm_sim/base/lib_rigidbody.hpp"



// RigidBody::RigidBody(const double &mass,  const Eigen::Matrix3d &m_inertia, const double &step_size): mass_(mass), m_inertia_(m_inertia),step_size_(step_size) 
// {
//     SetStatesZeros();
//     // std::cout<<std::setw(16)<<"[----------] RigidBody: RigidBody is called" << std::endl;

//     // std::cout<<std::setw(16)<<"[----------] RigidBody: RigidBody state is " << state_.transpose()<<std::endl;
// };


RigidBody::RigidBody(const MassProperty &mass_property, const double &step_size): mass_property_(mass_property),step_size_(step_size) 
{
    SetStatesZeros();
};


void RigidBody::SetStatesZeros()
{
    state_.setZero();
    // std::cout<<std::setw(16)<<"[----------] RigidBody: SetStatesZeros is called" << std::endl;
};




void RigidBody::InputWrench(const Wrench &input_wrench)
{
    std::cout<<std::string(8, ' ')<<"Entre RigidBody::InputWrench"<<std::endl;
    
    input_wrench_ = input_wrench;

    auto object_acc = TransDynac(input_wrench_.force, mass_property_.mass, gravity_);
    
    std::cout<<std::string(8, ' ')<<"mav acc is " << object_acc.transpose()<<std::endl;
    std::cout<<std::string(8, ' ')<<"mav net input force is " << input_wrench_.force.transpose()<<std::endl;
    std::cout<<std::string(8, ' ')<<"mav net input torque is " << input_wrench_.torque.transpose()<<std::endl;
    // accs_.linear_acc = TransDynac(input_wrench_.force, mass_property_.mass, gravity_);

    // obtain bodyrate from state
    // Eigen::Vector3d bodyrate = state_.tail(3);

    // compute dp, dq ,dr
    // object_bodyrate_acc_ = RotDynac(input_wrench_.torque, mass_property_.inertia, bodyrate);
    // accs_.angular_acc = RotDynac(input_wrench_.torque, mass_property_.inertia, bodyrate);
    std::cout<<std::string(8, ' ')<<"Leave RigidBody::InputWrench"<<std::endl;
}


// TODO-ZLi use quaterion for rotation dynamics in the future
Eigen::Vector3d RigidBody::RotDynac(const Eigen::Vector3d &torque, const Eigen::Matrix3d &Inertia, const Eigen::Vector3d &bodyrate)
{
    Eigen::Vector3d dBodyRate = Eigen::Vector3d::Zero();

    dBodyRate = Inertia.householderQr().solve(-bodyrate.cross(Inertia*bodyrate) + torque);

    // save body_rate_acc
    // object_bodyrate_acc_ = dBodyRate;

    return dBodyRate;
}

Eigen::Vector3d RigidBody::TransDynac(const Eigen::Vector3d &force_applied, const double &mass, const double &gravity)
{   
    Eigen::Vector3d acc = Eigen::Vector3d::Zero();

    acc = (force_applied-mass*gravity*Eigen::Vector3d::UnitZ())/mass;

    return acc;
}



// // void RigidBody::rhs(const quadrotor_state &x , quadrotor_state &dxdt, const double time)
void RigidBody::operator() (const object_state &x , object_state &dxdt, const double time)
{

    // static bool is_recursing = false;
    // if (is_recursing) return;  // Prevent recursion
    // is_recursing = true;
    

    // std::cout<<std::setw(16)<< << "state " << x.transpose()<<std::endl; 
    // get sub X vector
    // x =  [x,     y,      z,      dx,     dy,     dz,     phi,    theta,      psi,    p,      q,      r]
    // x =  [x,     y,      z,      dx,     dy,     dz,     qw,      qx,          qy,      qz,    p,      q,      r]
    // dx = [dx,    dy,     dz,     ddx,    ddy,    ddz,    dphi,   dtheta,     dpsi,   dp,     dq,     dr]

    // For instance
    // std::cout<<std::setw(16)<< "Euler angle is "<< x.segment<3>(6).transpose()<<std::endl;
    // std::cout<<std::setw(16)<< "Bodyrate is "<< x.tail(3).transpose()<<std::endl;
    // std::cout<<std::setw(16)<< "position is "<< x.head(3).transpose()<<std::endl;
    // std::cout<<std::setw(16)<< "vel is "<< x.segment<3>(3).transpose()<<std::endl;


    // define bodyrate
    Eigen::Vector3d bodyrate;
    bodyrate = x.tail(3);

    // translation in world frame
    // P = [x,y,z,dx, dy, dz]
    // dP = [dx, dy, dz, ddx, ddy, ddz]
    dxdt.head(3) = x.segment<3>(3);
    // [ddx ddy ddz] = (F-mg)/m
    dxdt.segment<3>(3) = TransDynac(input_wrench_.force, mass_property_.mass, gravity_);

    // std::cout<<std::setw(16)<< "Fuck base class is "<< "force is "<< force_.transpose()<< "mass is" << mass_<<std::endl;

    // compute matrix that maps bodyrate to dEuler
    Eigen::Matrix3d matrix_pdr2dEuler;
    matrix_pdr2dEuler = matirxBodyrate2EulerRate(x(6), x(7));

    // compute dphi,   dtheta,     dpsi
    // compute dqua--< p, q, r
    dxdt.segment<3>(6) = matrix_pdr2dEuler * bodyrate;

    // compute dp, dq ,dr
    dxdt.tail(3) = RotDynac(input_wrench_.torque, mass_property_.inertia, bodyrate);


    // std::cout<<std::setw(16)<<"fuck uav post" << x.head(3).transpose() <<std::endl;
    // std::cout<<std::setw(16)<<"fuck uav acc" <<  dxdt.segment<3>(3).transpose() <<std::endl;
    // std::cout<<std::setw(16)<<"fuck uav input force" <<  force_.transpose() <<std::endl;
    // update linear acc and angular acc
   
    

    // is_recursing = false;

    // accs_.linear_acc = dxdt.segment<3>(3);
    SetLinearAcc(dxdt.segment<3>(3));
    // accs_.angular_acc = dxdt.tail(3);
    SetAngularAcc(dxdt.tail(3));
    // std::cout<<std::setw(16)<<"accs_.linear_acc is " <<  accs_.linear_acc.transpose() <<std::endl;
}



void RigidBody::DoOneStepInt()
{

    // call one step integration for quadrotor dynamics
    // this->stepper_.do_step(*this, state_, current_step_, step_size_);
    this->stepper_.do_step(std::ref(*this), state_, current_step_, step_size_);

    // update current step
    current_step_ = current_step_ + step_size_;

};
  

void RigidBody::SetInitialPost(const Eigen::Vector3d &initial_post)
{
    
    // state vecgor for a rigid body (12X1) including position, velcity, euler angle, bodyrate, 
    // state_ = [x,     y,      z,      dx,     dy,     dz,     phi,    theta,      psi,    p,      q,      r]
    state_.head(3) = initial_post;
    // std::cout<<std::setw(16)<< "[----------] RigidBody: SetInitialPost state is " << state_.transpose()<<std::endl;
}; 


void RigidBody::SetInitialAttitude(const double &phi, const double &theta, const double &psi)
{
    // recall that state_ = [x,     y,      z,      dx,     dy,     dz,     phi,    theta,      psi,    p,      q,      r]
    state_[6] = phi;
    state_[7] = theta;
    state_[8] = psi;
    // std::cout<<std::setw(16)<< "initial state" << state_.transpose()<<std::endl;
    // std::cout<<std::setw(16)<< "initial attitude" << state_.segment<3>(6)<<std::endl;
};

void RigidBody::SetVel(const Eigen::Vector3d &object_vel)
{
    state_.segment<3>(3) = object_vel;
};

void RigidBody::SetLinearAcc(const Eigen::Vector3d &object_acc)
{
    accs_.linear_acc = object_acc;
};

// update objec' bodyrate with input
void RigidBody::SetBodyrate(const Eigen::Vector3d &object_bodyrate)
{

    state_.tail(3) = object_bodyrate;
}

void RigidBody::SetAngularAcc(const Eigen::Vector3d &object_bodyrate_acc)
{
    accs_.angular_acc = object_bodyrate_acc;
};




// void RigidBody::GetPosition(Eigen::Vector3d &object_position) const
// {
//     // std::cout<<std::setw(16)<< "[----------] RigidBody: GetPosition state is" << state_.transpose() <<std::endl;
//     object_position = state_.head(3);
//     // std::cout<<std::setw(16)<< "[----------] RigidBody: GetPosition" << state_.head<3>().transpose()<< object_position.transpose() <<std::endl;
// };


// void RigidBody::GetState(object_state &state) const 
// {

//     state = state_;
// }


// void RigidBody::GetVel(Eigen::Vector3d &object_vel) const
// {
//     object_vel = state_.segment<3>(3);
// };


// void RigidBody::GetAcc(Eigen::Vector3d &object_acc) const
// { 
    
//     // compute translation acc
//     object_acc = object_acc_;
// }

// void RigidBody::GetBodyrate(Eigen::Vector3d &object_bodyrate) const
// {
//     object_bodyrate = state_.tail<3>();
// };

// void RigidBody::GetAttitude(Eigen::Quaterniond &object_attitude) const
// {

//     // compute rotation in Quaternion from quadrotor state
//     // rotation is created using ZYX rotation in its body frame
//     auto attitude =  Eigen::AngleAxisd(state_(8), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(state_(7),Eigen::Vector3d::UnitY()) *Eigen::AngleAxisd(state_(6), Eigen::Vector3d::UnitX());

//     // normalise
//     attitude.normalize();

//     // assisn 
//     object_attitude = attitude;

// };

// void RigidBody::GetBodyRateAcc(Eigen::Vector3d &object_bodyrate_acc) const
// { 
//     object_bodyrate_acc = object_bodyrate_acc_;
    
// }




double RigidBody::mass() const
{

    return mass_property_.mass;
}



Eigen::Matrix3d RigidBody::inertia() const
{

    return mass_property_.inertia;

}




Pose RigidBody::pose() const
{
    auto robot_post = state_.head<3>();

    auto robot_att = Eigen::AngleAxisd(state_(8), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(state_(7),Eigen::Vector3d::UnitY()) *Eigen::AngleAxisd(state_(6), Eigen::Vector3d::UnitX());

    robot_att.normalize();

    Pose robot_pose(robot_post, robot_att);

    return robot_pose;
};


Vels RigidBody::vels() const
{
    //x =  [x,     y,      z,      dx,     dy,     dz,     phi,    theta,      psi,    p,      q,      r]
    auto robot_linear_vel = state_.segment<3>(3);
    auto robot_angular_rate = state_.tail<3>(); 


    // define output
    Vels robot_vels(robot_linear_vel, robot_angular_rate);

    return robot_vels;

}


Accs RigidBody::accs() const
{

    return accs_;
}

// void RigidBody::GetCurrentTimeStep(double &current_time)
// {
//     current_time = current_step_;
// };


double RigidBody::timestep() const
{

    return current_step_;
}


Eigen::Matrix3d RigidBody::TransVector3d2SkewSymMatrix(Eigen::Vector3d vector)
{
    Eigen::Matrix3d m_skewsym;

    m_skewsym << 0, -vector(2), vector(1),
        vector(2), 0, -vector(0),
        -vector(1), vector(0), 0;

    return m_skewsym;
};


Eigen::Matrix3d RigidBody::matirxBodyrate2EulerRate(const double &phi, const double &theta)
{
    Eigen::Matrix3d m_Bodyrate2EulerRate;

        m_Bodyrate2EulerRate(0,0) = 1;
        m_Bodyrate2EulerRate(0,1) = sin(phi)*tan(theta);
        m_Bodyrate2EulerRate(0,2) = cos(phi)*tan(theta);

        m_Bodyrate2EulerRate(1,0) = 0;
        m_Bodyrate2EulerRate(1,1) = cos(phi);
        m_Bodyrate2EulerRate(1,2) = -sin(phi);

        m_Bodyrate2EulerRate(2,0) = 0;
        m_Bodyrate2EulerRate(2,1) = sin(phi)/cos(theta);
        m_Bodyrate2EulerRate(2,2) = cos(phi)/cos(theta);

    return m_Bodyrate2EulerRate;
} 
