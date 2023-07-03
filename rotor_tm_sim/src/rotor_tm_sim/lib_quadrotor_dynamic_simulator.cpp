#include "rotor_tm_sim/lib_quadrotor_dynamic_simulator.hpp"



QuadrotorDynamicSimulator::QuadrotorDynamicSimulator(const double &mass,  const Eigen::Matrix3d &m_inertia, const double &step_size): mass_(mass), step_size_(step_size), m_inertia_(m_inertia) 
{
    done_state_.setZero();
};


Eigen::Vector3d QuadrotorDynamicSimulator::quadRotDynac(const Eigen::Vector3d &torque, const Eigen::Matrix3d &Inertia, const Eigen::Vector3d &bodyrate)
{
    Eigen::Vector3d dBodyRate = Eigen::Vector3d::Zero();

    dBodyRate = Inertia.householderQr().solve(-bodyrate.cross(Inertia*bodyrate) + torque);

    return dBodyRate;
}

Eigen::Vector3d QuadrotorDynamicSimulator::quadTransDynac(const Eigen::Vector3d &Thurst, const double &mass, const double &gravity)
{   
    Eigen::Vector3d acc = Eigen::Vector3d::Zero();

    acc = (Thurst-mass*gravity*Eigen::Vector3d::UnitZ())/mass;

    return acc;
}


Eigen::Matrix3d QuadrotorDynamicSimulator::matirxBodyrate2EulerRate(const double &phi, const double &theta, const double &psi)
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

// void QuadrotorDynamicSimulator::rhs(const quadrotor_state &x , quadrotor_state &dxdt, const double time)
void QuadrotorDynamicSimulator::operator() (const quadrotor_state &x , quadrotor_state &dxdt, const double time)
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
    dxdt.segment<3>(3) = quadTransDynac(thrust_, mass_, gravity_);

    // 
    Eigen::Matrix3d matrix_pdr2dEuler;
    matrix_pdr2dEuler = matirxBodyrate2EulerRate(x(6), x(7), x(8));

    // compute dphi,   dtheta,     dpsi
    dxdt.segment<3>(6) = matrix_pdr2dEuler * bodyrate;

    // compute dp, dq ,dr
    dxdt.tail(3) = quadRotDynac(torque_, m_inertia_, bodyrate);


    is_recursing = false;
}



void QuadrotorDynamicSimulator::doOneStepInt()
{

    // call one step integration for quadrotor dynamics
    this->stepper_.do_step(*this, done_state_, current_step_, step_size_);

    // update current step
    current_step_ = current_step_ + step_size_;


};
  


void QuadrotorDynamicSimulator::setVel(const Eigen::Vector3d &mav_vel)
{
    done_state_.segment<3>(3) = mav_vel;
}


void QuadrotorDynamicSimulator::getPosition(Eigen::Vector3d &mav_position)
{
    mav_position = done_state_.head<3>();
};

void QuadrotorDynamicSimulator::getVel(Eigen::Vector3d &mav_vel)
{
    mav_vel = done_state_.segment<3>(3);
};

void QuadrotorDynamicSimulator::getBodyrate(Eigen::Vector3d &mav_bodyrate)
{
    mav_bodyrate = done_state_.tail<3>();
};

void QuadrotorDynamicSimulator::getAttitude(Eigen::Quaterniond &mav_attitude)
{

    // compute rotation in Quaternion from quadrotor state
    // rotation is created using ZYX rotation in its body frame
    auto attitude =  Eigen::AngleAxisd(done_state_(8), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(done_state_(7),Eigen::Vector3d::UnitY()) *Eigen::AngleAxisd(done_state_(6), Eigen::Vector3d::UnitX());

    // normalise
    attitude.normalize();

    // assisn 
    mav_attitude = attitude;

};

void QuadrotorDynamicSimulator::inputForce(const Eigen::Vector3d &mav_thrust_force)
{
    thrust_ = mav_thrust_force;
};

void QuadrotorDynamicSimulator::inputThurst(const double &mav_thrust)
{
    //1. compute thrust force in body frame [0,0,T]
    Eigen::Vector3d thrust_force_bf(0,0,mav_thrust);

    //2. obtain rotation matrix that represents drone rotation w.r.t world frame
    // 2.1 compute attitude from drone state's Euler angles
    Eigen::Quaterniond attitude = Eigen::AngleAxisd(done_state_(8), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(done_state_(7),Eigen::Vector3d::UnitY()) *Eigen::AngleAxisd(done_state_(6), Eigen::Vector3d::UnitX());
    // 2.2 normalise
    attitude.normalize();
    // 2.3 obtain rot matrix from quaternion
    Eigen::Matrix3d rot_matrix = attitude.toRotationMatrix();

    // 3. compute thrust force in world frame
    Eigen::Vector3d thrust_force_wf =  rot_matrix * thrust_force_bf;

    inputForce(thrust_force_wf);

};

void QuadrotorDynamicSimulator::inputTorque(const Eigen::Vector3d &mav_torque)
{
    torque_ = mav_torque;
};

void QuadrotorDynamicSimulator::getCurrentTimeStep(double &current_time)
{
    current_time = current_step_;
};
