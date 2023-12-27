#include "rotor_tm_sim/lib_quadrotor.hpp"



Quadrotor::Quadrotor(const double &mass,  const Eigen::Matrix3d &m_inertia, const double &step_size):RigidBody(mass, m_inertia, step_size)
{
   SetStatesZeros();
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


void Quadrotor::InputThurst(const double &mav_thrust)
{
    //1. recall thrust force in world frame =  0^R_L * [0,0,T] 
     Eigen::Vector3d thrust_force_bf(0,0,mav_thrust);
     
    // thrust force in world frame is mav_thrust_force_

    //2. obtain rotation matrix that represents drone rotation w.r.t world frame
    
    // 2.1 obtain mav state from base class
    mav_state mav_state;
    GetState(mav_state);

    // 2.2 compute attitude from drone state's Euler angles
    Eigen::Quaterniond attitude = Eigen::AngleAxisd(mav_state(8), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(mav_state(7),Eigen::Vector3d::UnitY()) *Eigen::AngleAxisd(mav_state(6), Eigen::Vector3d::UnitX());
    // 2.3 normalise
    attitude.normalize();
    // 2.4 obtain rot matrix from quaternion
    Eigen::Matrix3d rot_matrix = attitude.toRotationMatrix();

    // 3. compute thrust force in world frame
    mav_thrust_force_ =  rot_matrix * thrust_force_bf;

    InputForce(mav_thrust_force_);

};