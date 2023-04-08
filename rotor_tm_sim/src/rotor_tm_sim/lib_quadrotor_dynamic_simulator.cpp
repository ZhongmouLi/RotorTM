#include "rotor_tm_sim/lib_quadrotor_dynamic_simulator.hpp"


// using namespace std;
// using namespace boost::numeric::odeint;

// typedef Eigen::Matrix<double, 12, 1> quadrotor_state;


QuadrotorDynamicSimulator::QuadrotorDynamicSimulator(const double &mass,  const Eigen::Matrix3d &m_inertia, const double &step_size): mass_(mass), step_size_(step_size), m_inertia_(m_inertia) 
{
    done_state_.setZero();
};


Eigen::Vector3d QuadrotorDynamicSimulator::quadRotDynac(const Eigen::Vector3d &torque, const Eigen::Matrix3d &Inertia, const Eigen::Vector3d &bodyrate)
{
    Eigen::Vector3d dBodyRate = Eigen::Vector3d::Zero();

    dBodyRate = Inertia.ldlt().solve(-bodyrate.cross(Inertia*bodyrate) + torque);

    return dBodyRate;
}

Eigen::Vector3d QuadrotorDynamicSimulator::quadTransDynac(const Eigen::Vector3d &Thurst, const double &mass, const double &gravity)
{   
    Eigen::Vector3d acc = Eigen::Vector3d::Zero();

    acc = (Thurst-mass*gravity*Eigen::Vector3d::UnitZ())/mass;

    return acc;
}

Eigen::Vector3d QuadrotorDynamicSimulator::quadBodyrate2Eulerrate(const Eigen::Vector3d &bodyrate, const Eigen::Matrix3d &trans_matrix)
{

    Eigen::Vector3d Eulerrate = Eigen::Vector3d::Zero();

    Eulerrate = trans_matrix.ldlt().solve(bodyrate);

    return Eulerrate;

}

Eigen::Matrix3d QuadrotorDynamicSimulator::quadTransMatrix(const double &phi, const double &theta, const double &psi)
{
    // ref https://aviation.stackexchange.com/questions/83993/the-relation-between-euler-angle-rate-and-body-axis-rates
    Eigen::Matrix3d trans_matrix;
    trans_matrix(0,0) = 1;
    trans_matrix(0,1) = 0;
    trans_matrix(0,2) = -sin(theta);

    trans_matrix(1,0) = 0;
    trans_matrix(1,1) = cos(theta);
    trans_matrix(1,2) = sin(theta)*cos(theta);

    trans_matrix(2,0) = 0;
    trans_matrix(2,1) = -sin(theta);
    trans_matrix(2,2) = cos(theta)*cos(theta);

    return trans_matrix;
} 

void QuadrotorDynamicSimulator::rhs(const quadrotor_state &x , quadrotor_state &dxdt, const double time)
{
    // Eigen::Vector3d thrust(1,2,0);
    // double mass =1;

    // Eigen::Vector3d torque(0,0,0);
    // Eigen::Matrix3d Inertia = Eigen::Matrix3d::Identity(3,3);
    // Inertia(0,0)=0.1;
    // Inertia(1,1)=0.1;
    // Inertia(2,2)=0.2;        

    // get sub X vector
    // X = [post_v att_v]
    Eigen::VectorXd post_v(6);
    Eigen::VectorXd att_v(6);

    // define sub dX vector
    Eigen::VectorXd dpost_v(6);
    Eigen::VectorXd datt_v(6);

    // get postion and attitude from input
    post_v = x.head(6);
    att_v = x.tail(6);

    // define bodyrate
    // Eigen::Vector3d bodyrate;
    // bodyrate = att_v.tail(3);
    bodyrate_ = att_v.tail(3);

    // translation in world frame
    // P = [x,y,z,dx, dy, dz]
    // dP = [dx, dy, dz, ddx, ddy, ddz]
    dpost_v(0) = post_v(3);
    dpost_v(1) = post_v(4);
    dpost_v(2) = post_v(5);
    // [ddx ddy ddz] = (F-mg)/m
    dpost_v.tail(3) = quadTransDynac(thrust_, mass_, gravity);

    // attitude in body frame
    // att_v = [phi, theta, psi, p, q, r]
    // datt_v = [dphi, dtheta, dpsi, dp, dq, dr]

    // 
    Eigen::Matrix3d trans_matrix;
    trans_matrix = quadTransMatrix(att_v(0), att_v(1), att_v(2));

    datt_v.head(3) = quadBodyrate2Eulerrate(bodyrate_, trans_matrix);

    // 
    datt_v.tail(3) = quadRotDynac(torque_, m_inertia_, bodyrate_);


    // assign
    dxdt.head(6) =  dpost_v;
    dxdt.tail(6) =  datt_v;

    this->post_ = post_v.head(3);
    this->vel_  = post_v.tail(3);

    std::cout<<"post_ "<<this->post_.transpose()<< std::endl;
}


// void QuadrotorDynamicSimulator::rhs(const quadrotor_state &x , quadrotor_state &dxdt, const double time)
void QuadrotorDynamicSimulator::operator() (const quadrotor_state &x , quadrotor_state &dxdt, const double time)
{
     
    // get sub X vector
    // X = [post_v att_v]
    Eigen::VectorXd post_v(6);
    Eigen::VectorXd att_v(6);

    // define sub dX vector
    Eigen::VectorXd dpost_v(6);
    Eigen::VectorXd datt_v(6);

    // get postion and attitude from input
    post_v = x.head(6);
    att_v = x.tail(6);

    // define bodyrate
    Eigen::Vector3d bodyrate;
    bodyrate = att_v.tail(3);
    // bodyrate_ = att_v.tail(3);

    // translation in world frame
    // P = [x,y,z,dx, dy, dz]
    // dP = [dx, dy, dz, ddx, ddy, ddz]
    dpost_v(0) = post_v(3);
    dpost_v(1) = post_v(4);
    dpost_v(2) = post_v(5);
    // [ddx ddy ddz] = (F-mg)/m
    dpost_v.tail(3) = quadTransDynac(thrust_, mass_, gravity);

    // attitude in body frame
    // att_v = [phi, theta, psi, p, q, r]
    // datt_v = [dphi, dtheta, dpsi, dp, dq, dr]

    // 
    Eigen::Matrix3d trans_matrix;
    trans_matrix = quadTransMatrix(att_v(0), att_v(1), att_v(2));

    datt_v.head(3) = quadBodyrate2Eulerrate(bodyrate, trans_matrix);

    // 
    datt_v.tail(3) = quadRotDynac(torque_, m_inertia_, bodyrate_);


    // assign
    dxdt.head(6) =  dpost_v;
    dxdt.tail(6) =  datt_v;

    // this->post_ = post_v.head(3);
    // this->vel_  = post_v.tail(3);

    // std::cout<<"post_ "<<this->post_.transpose()<< std::endl;
}



void QuadrotorDynamicSimulator::doOneStepInt()
{

    // this->stepper_.do_step([&](const quadrotor_state &x , quadrotor_state &dxdt, const double time) {
    //   this->rhs(done_state_, dxdt, time);
    // }, done_state_, step_size_, step_size_);

    this->stepper_.do_step(*this, done_state_, current_step_, step_size_);

    // void (*operation)(const quadrotor_state &x,  quadrotor_state &dxdt, const double time)

//    std::cout << "current step " << current_step_ << " position is "  <<this->post_.transpose() << std::endl;
//    std::cout << "current step " << current_step_ << " state is "  <<this->done_state_.transpose() << std::endl;

    // std::cout << "current step " << current_step_ << std::endl;

    current_step_ = current_step_ + step_size_;

    assignDroneState(done_state_);

};
  

void QuadrotorDynamicSimulator::assignDroneState(const quadrotor_state &done_state)
{
    // done_state = [x,y,z,dx, dy, dz, phi, theta, psi, p, q, r]
    post_ = done_state.head<3>();
    vel_ = done_state.segment<3>(3);
    bodyrate_ = done_state.tail<3>();

    attitude_ =  Eigen::AngleAxisd(done_state(8), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(done_state(7),Eigen::Vector3d::UnitY()) *Eigen::AngleAxisd(done_state(6), Eigen::Vector3d::UnitX());
    attitude_.normalize();

};  
 

void QuadrotorDynamicSimulator::getPosition(Eigen::Vector3d &mav_position)
{
    mav_position = post_;
};

void QuadrotorDynamicSimulator::getVel(Eigen::Vector3d &mav_vel)
{
    mav_vel = vel_;
};

void QuadrotorDynamicSimulator::getBodyrate(Eigen::Vector3d &mav_bodyrate)
{
    mav_bodyrate = bodyrate_;
};

void QuadrotorDynamicSimulator::getAttitude(Eigen::Quaterniond &mav_attitude)
{
    mav_attitude = attitude_;
};

void QuadrotorDynamicSimulator::inputThurstForce(const Eigen::Vector3d &mav_thrust)
{
    thrust_ = mav_thrust;
};

void QuadrotorDynamicSimulator::inputTorque(const Eigen::Vector3d &mav_torque)
{
    torque_ = mav_torque;
};

void QuadrotorDynamicSimulator::getCurrentTimeStep(double &current_time)
{
    current_time = current_step_;
};
