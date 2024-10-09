#include "rotor_tm_sim/lib_quadrotor.hpp"



Quadrotor::Quadrotor(const MassProperty &mass_property, const double &step_size):RigidBody(mass_property, step_size)
{
    //    SetStatesZeros();
};

// void Quadrotor::rhs(const quadrotor_state &x , quadrotor_state &dxdt, const double time)
// void Quadrotor::operator() (const quadrotor_state &x , quadrotor_state &dxdt, const double time)
// {

//     static bool is_recursing = false;
//     if (is_recursing) return;  // Prevent recursion
//     is_recursing = true;
    

//     // std::cout << "state " << x.transpose()<<std::endl; 
//     // get sub X vector
//     // x =  [x,     y,      z,      dx,     dy,     dz,     phi,    theta,      psi,    p,      q,      r]
//     // dx = [dx,    dy,     dz,     ddx,    ddy,    ddz,    dphi,   dtheta,     dpsi,   dp,     dq,     dr]

//     // For instance
//     // std::cout<< "Euler angle is "<< x.segment<3>(6).transpose()<<std::endl;
//     // std::cout<< "Bodyrate is "<< x.tail(3).transpose()<<std::endl;
//     // std::cout<< "position is "<< x.head(3).transpose()<<std::endl;
//     // std::cout<< "vel is "<< x.segment<3>(3).transpose()<<std::endl;


//     // define bodyrate
//     Eigen::Vector3d bodyrate;
//     bodyrate = x.tail(3);

//     // translation in world frame
//     // P = [x,y,z,dx, dy, dz]
//     // dP = [dx, dy, dz, ddx, ddy, ddz]
//     dxdt.head(3) = x.segment<3>(3);
//     // [ddx ddy ddz] = (F-mg)/m
//     dxdt.segment<3>(3) = quadTransDynac(thrust_, mass_, gravity_);

//     // 
//     Eigen::Matrix3d matrix_pdr2dEuler;
//     matrix_pdr2dEuler = matirxBodyrate2EulerRate(x(6), x(7), x(8));

//     // compute dphi,   dtheta,     dpsi
//     dxdt.segment<3>(6) = matrix_pdr2dEuler * bodyrate;

//     // compute dp, dq ,dr
//     dxdt.tail(3) = quadRotDynac(torque_, m_inertia_, bodyrate);


//     is_recursing = false;
// }


void Quadrotor::InputDroneThrustTorque(const double &mav_thrust, const Eigen::Vector3d &mav_torque)
{
    //1. recall thrust force in world frame =  0^R_L * [0,0,T] 
     Eigen::Vector3d thrust_force_bf(0,0,mav_thrust);
     
    // thrust force in world frame is mav_thrust_force_

    //2. obtain rotation matrix that represents drone rotation w.r.t world frame

    Eigen::Matrix3d rot_matrix =  pose().att.toRotationMatrix(); 

    // std::cout<< "rot matrix of quadrotor is " <<std::endl <<rot_matrix<<std::endl;

    // 3. compute thrust force in world frame
    Eigen::Vector3d mav_thrust_force =  rot_matrix * thrust_force_bf;

    // 4. create input wrench
    Wrench mav_wrench = {mav_thrust_force, mav_torque};

    InputWrench(mav_wrench);

    // 5. save thrust and torque
    mav_thrust_ = mav_thrust;
    mav_torque_ = mav_torque;
}


// void Quadrotor::InputNetForce(const Eigen::Vector3d &mav_net_force)
// {
//     //1. compute gravity
//     double mav_mass;
//     GetMass(mav_mass);

//     Eigen::Vector3d mav_input_force = mav_net_force + (Eigen::Vector3d::UnitZ() * mav_mass * gravity_);

//     InputForce(mav_input_force);

// }

// void Quadrotor::InputThurst(const double &mav_thrust)
// {
    

//     std::cout<<"[----------] mav_thrust_force_ is " << mav_thrust_force_.transpose() << std::endl;

//     InputForce(mav_thrust_force_);

// };