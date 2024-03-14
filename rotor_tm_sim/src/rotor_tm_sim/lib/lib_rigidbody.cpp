#include "rotor_tm_sim/lib_rigidbody.hpp"



RigidBody::RigidBody(const double &mass,  const Eigen::Matrix3d &m_inertia, const double &step_size): mass_(mass), m_inertia_(m_inertia),step_size_(step_size) 
{
    SetStatesZeros();
    // std::cout<<"[----------] RigidBody: RigidBody is called" << std::endl;

    // std::cout<<"[----------] RigidBody: RigidBody state is " << state_.transpose()<<std::endl;
};


void RigidBody::SetStatesZeros()
{
    state_.setZero();
    // std::cout<<"[----------] RigidBody: SetStatesZeros is called" << std::endl;
};


// TODO-ZLi use quaterion for rotation dynamics in the future
Eigen::Vector3d RigidBody::RotDynac(const Eigen::Vector3d &torque, const Eigen::Matrix3d &Inertia, const Eigen::Vector3d &bodyrate)
{
    Eigen::Vector3d dBodyRate = Eigen::Vector3d::Zero();

    dBodyRate = Inertia.householderQr().solve(-bodyrate.cross(Inertia*bodyrate) + torque);

    // save body_rate_acc
    object_bodyrate_acc_ = dBodyRate;

    return dBodyRate;
}

Eigen::Vector3d RigidBody::TransDynac(const Eigen::Vector3d &Thurst, const double &mass, const double &gravity)
{   
    Eigen::Vector3d acc = Eigen::Vector3d::Zero();

    acc = (Thurst-mass*gravity*Eigen::Vector3d::UnitZ())/mass;

    return acc;
}


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

// // void RigidBody::rhs(const quadrotor_state &x , quadrotor_state &dxdt, const double time)
void RigidBody::operator() (const object_state &x , object_state &dxdt, const double time)
{

    static bool is_recursing = false;
    if (is_recursing) return;  // Prevent recursion
    is_recursing = true;
    

    // std::cout << "state " << x.transpose()<<std::endl; 
    // get sub X vector
    // x =  [x,     y,      z,      dx,     dy,     dz,     phi,    theta,      psi,    p,      q,      r]
    // dx = [dx,    dy,     dz,     ddx,    ddy,    ddz,    dphi,   dtheta,     dpsi,   dp,     dq,     dr]

    // For instance
    // std::cout<< "Euler angle is "<< x.segment<3>(6).transpose()<<std::endl;
    // std::cout<< "Bodyrate is "<< x.tail(3).transpose()<<std::endl;
    // std::cout<< "position is "<< x.head(3).transpose()<<std::endl;
    // std::cout<< "vel is "<< x.segment<3>(3).transpose()<<std::endl;


    // define bodyrate
    Eigen::Vector3d bodyrate;
    bodyrate = x.tail(3);

    // translation in world frame
    // P = [x,y,z,dx, dy, dz]
    // dP = [dx, dy, dz, ddx, ddy, ddz]
    dxdt.head(3) = x.segment<3>(3);
    // [ddx ddy ddz] = (F-mg)/m
    dxdt.segment<3>(3) = TransDynac(force_, mass_, gravity_);

    // std::cout<< "Fuck base class is "<< "force is "<< force_.transpose()<< "mass is" << mass_<<std::endl;

    // compute matrix that maps bodyrate to dEuler
    Eigen::Matrix3d matrix_pdr2dEuler;
    matrix_pdr2dEuler = matirxBodyrate2EulerRate(x(6), x(7));

    // compute dphi,   dtheta,     dpsi
    dxdt.segment<3>(6) = matrix_pdr2dEuler * bodyrate;

    // compute dp, dq ,dr
    dxdt.tail(3) = RotDynac(torque_, m_inertia_, bodyrate);


    std::cout<<"fuck uav post" << x.head(3).transpose() <<std::endl;
    std::cout<<"fuck uav acc" <<  dxdt.segment<3>(3).transpose() <<std::endl;
    std::cout<<"fuck uav input force" <<  force_.transpose() <<std::endl;
    
    is_recursing = false;
}



void RigidBody::DoOneStepInt()
{

    // call one step integration for quadrotor dynamics
    this->stepper_.do_step(*this, state_, current_step_, step_size_);

    // update current step
    current_step_ = current_step_ + step_size_;

};
  

void RigidBody::SetInitialPost(const Eigen::Vector3d &initial_post)
{
    
    // state vecgor for a rigid body (12X1) including position, velcity, euler angle, bodyrate, 
    // state_ = [x,     y,      z,      dx,     dy,     dz,     phi,    theta,      psi,    p,      q,      r]
    state_.head(3) = initial_post;
    // std::cout<< "[----------] RigidBody: SetInitialPost state is " << state_.transpose()<<std::endl;
}; 


void RigidBody::SetInitialAttitude(const double &phi, const double &theta, const double &psi)
{
    // recall that state_ = [x,     y,      z,      dx,     dy,     dz,     phi,    theta,      psi,    p,      q,      r]
    state_[6] = phi;
    state_[7] = theta;
    state_[8] = psi;
    // std::cout<< "initial state" << state_.transpose()<<std::endl;
    // std::cout<< "initial attitude" << state_.segment<3>(6)<<std::endl;
};

void RigidBody::SetVel(const Eigen::Vector3d &object_vel)
{
    state_.segment<3>(3) = object_vel;
};

void RigidBody::SetAcc(const Eigen::Vector3d &object_acc)
{
    object_acc_ = object_acc;
};

// update objec' bodyrate with input
void RigidBody::SetBodyrate(const Eigen::Vector3d &object_bodyrate)
{

    state_.tail(3) = object_bodyrate;
}

void RigidBody::SetBodyrateAcc(const Eigen::Vector3d &object_bodyrate_acc)
{
    object_bodyrate_acc_ = object_bodyrate_acc;
};

void RigidBody::GetPosition(Eigen::Vector3d &object_position) const
{
    // std::cout<< "[----------] RigidBody: GetPosition state is" << state_.transpose() <<std::endl;
    object_position = state_.head(3);
    // std::cout<< "[----------] RigidBody: GetPosition" << state_.head<3>().transpose()<< object_position.transpose() <<std::endl;
};


void RigidBody::GetState(object_state &state) const 
{

    state = state_;
}


void RigidBody::GetVel(Eigen::Vector3d &object_vel) const
{
    object_vel = state_.segment<3>(3);
};


void RigidBody::GetAcc(Eigen::Vector3d &object_acc) const
{ 
    
    // compute translation acc
    object_acc = object_acc_;
}

void RigidBody::GetBodyrate(Eigen::Vector3d &object_bodyrate) const
{
    object_bodyrate = state_.tail<3>();
};

void RigidBody::GetAttitude(Eigen::Quaterniond &object_attitude) const
{

    // compute rotation in Quaternion from quadrotor state
    // rotation is created using ZYX rotation in its body frame
    auto attitude =  Eigen::AngleAxisd(state_(8), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(state_(7),Eigen::Vector3d::UnitY()) *Eigen::AngleAxisd(state_(6), Eigen::Vector3d::UnitX());

    // normalise
    attitude.normalize();

    // assisn 
    object_attitude = attitude;

};

void RigidBody::GetBodyRateAcc(Eigen::Vector3d &object_bodyrate_acc) const
{ 
    object_bodyrate_acc = object_bodyrate_acc_;
    
}

void RigidBody::InputForce(const Eigen::Vector3d &force)
{
    force_ = force;
    // compute acc
    object_acc_ = TransDynac(force_, mass_, gravity_);
    // std::cout<<"[----------] RigidBodyt/InputForce input force is  " << force_.transpose()<<std::endl;
    // std::cout<<"[----------] RigidBodyt/InputForce mass is  " << mass_<<std::endl;
    // std::cout<<"[----------] RigidBodyt/InputForce gravity is  " << gravity_<<std::endl;
    // std::cout<<"[----------] RigidBodyt/InputForce object_acc_ is  " << object_acc_.transpose()<<std::endl;    
};

// void RigidBody::inputThurst(const double &mav_thrust)
// {
//     //1. compute thrust force in body frame [0,0,T]
//     Eigen::Vector3d force_force_bf(0,0,mav_thrust);

//     //2. obtain rotation matrix that represents drone rotation w.r.t world frame
//     // 2.1 compute attitude from drone state's Euler angles
//     Eigen::Quaterniond attitude = Eigen::AngleAxisd(state_(8), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(state_(7),Eigen::Vector3d::UnitY()) *Eigen::AngleAxisd(state_(6), Eigen::Vector3d::UnitX());
//     // 2.2 normalise
//     attitude.normalize();
//     // 2.3 obtain rot matrix from quaternion
//     Eigen::Matrix3d rot_matrix = attitude.toRotationMatrix();

//     // 3. compute thrust force in world frame
//     Eigen::Vector3d force_force_wf =  rot_matrix * force_force_bf;

//     inputForce(force_force_wf);

// };

void RigidBody::InputTorque(const Eigen::Vector3d &torque)
{
    torque_ = torque;
    // compute body_rate_acc
    
    // obtain bodyrate from state
    Eigen::Vector3d bodyrate = state_.tail(3);

    // compute dp, dq ,dr
    object_bodyrate_acc_ = RotDynac(torque_, m_inertia_, bodyrate);

};


void RigidBody::GetCurrentTimeStep(double &current_time)
{
    current_time = current_step_;
};


Eigen::Matrix3d RigidBody::TransVector3d2SkewSymMatrix(Eigen::Vector3d vector)
{
    Eigen::Matrix3d m_skewsym;

    m_skewsym << 0, -vector(2), vector(1),
        vector(2), 0, -vector(0),
        -vector(1), vector(0), 0;

    return m_skewsym;
};
