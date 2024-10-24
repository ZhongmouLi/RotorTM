#include "rotor_tm_sim/base/lib_rigidbody.hpp"



// RigidBody::RigidBody(const double &mass,  const Eigen::Matrix3d &m_inertia, const double &step_size): mass_(mass), m_inertia_(m_inertia),step_size_(step_size) 
// {
//     SetStatesZeros();
//     // std::cout<<std::setw(16)<<"[----------] RigidBody: RigidBody is called" << std::endl;

//     // std::cout<<std::setw(16)<<"[----------] RigidBody: RigidBody state is " << state_.transpose()<<std::endl;
// };


RigidBody::RigidBody(const MassProperty &mass_property, const double &step_size): mass_property_(mass_property),step_size_(step_size), controlled_stepper(
              error_checker_type(1.0e-6, 1.0e-3)  // absolute and relative tolerances
             )
{

    // set stable states
    SetStatesZeros();

    // compute inverse of inertia
    inv_inertia_ = mass_property_.inertia.inverse();


    

    
};


void RigidBody::SetStatesZeros()
{
    // state_.setZero();
    // state_[6] = 1; 

     std::fill(state_.begin(), state_.end(), 0);
     state_.at(6) = 1;
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

    // get const inertia inverse
    // dBodyRate = Inertia.householderQr().solve(-bodyrate.cross(Inertia*bodyrate) + torque);

    dBodyRate = inv_inertia_ * (-bodyrate.cross(Inertia*bodyrate) + torque);

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
    // x =  [x,     y,      z,      dx,     dy,     dz,     qnw,      qnx,          qny,      qnz,    p,      q,      r]
    // dx = [dx,    dy,     dz,     ddx,    ddy,    ddz,    dqnw,     dqnx,         dqny,     dqnz,   dp,     dq,     dr]




    // translation in world frame
    // P = [x,y,z,dx, dy, dz]
    // dP = [dx, dy, dz, ddx, ddy, ddz]
    // dxdt.head(3) = x.segment<3>(3);
    dxdt.at(0) = x.at(3);
    dxdt.at(1) = x.at(4);
    dxdt.at(2) = x.at(5);

    // [ddx ddy ddz] = (F-mg)/m
    // dxdt.segment<3>(3) = TransDynac(input_wrench_.force, mass_property_.mass, gravity_);
    auto ddx = TransDynac(input_wrench_.force, mass_property_.mass, gravity_);
    dxdt.at(3) = ddx[0];
    dxdt.at(4) = ddx[1];
    dxdt.at(5) = ddx[2];

    // std::cout<<std::setw(16)<< "Fuck base class is "<< "force is "<< force_.transpose()<< "mass is" << mass_<<std::endl;

    // compute matrix that maps bodyrate to dEuler
    // Eigen::Matrix3d matrix_pdr2dEuler;
    // matrix_pdr2dEuler = matirxBodyrate2EulerRate(x(6), x(7));

    // compute dphi,   dtheta,     dpsi
    // compute dqua--< p, q, r
    // dxdt.segment<3>(6) = matrix_pdr2dEuler * bodyrate;


    // map bodyrate to quaternion derivative
    // current att in quaternion
    Eigen::Quaterniond qn(x.at(6), x.at(7), x.at(8), x.at(9));
    qn.normalize();

    // convert bodyrate into quaternion
    // define bodyrate
    // Eigen::Vector3d bodyrate;
    // bodyrate = x.tail(3);    
    Eigen::Vector3d bodyrate(x.at(10), x.at(11), x.at(12));
    // Eigen::Quaterniond omega(0, bodyrate[0], bodyrate[1], bodyrate[2]);

    // // compute quaternion derivative
    // Eigen::Quaterniond dqn = 0.5 * ( omega * qn);
    // dqn.normalize();
    auto dqn = ComputeQuaternionDerivative(qn, bodyrate);

    // dxdt[6] = dqn(0);
    // dxdt[7] = dqn(1);
    // dxdt[8] = dqn(2);
    // dxdt[9] = dqn(3);

    dxdt.at(6) = dqn(0);
    dxdt.at(7) = dqn(1);
    dxdt.at(8) = dqn(2);
    dxdt.at(9) = dqn(3);

    // compute dp, dq ,dr
    // dxdt.tail(3) = RotDynac(input_wrench_.torque, mass_property_.inertia, bodyrate);
    auto dpqr = RotDynac(input_wrench_.torque, mass_property_.inertia, bodyrate);
    
    dxdt.at(10) = dpqr[0];
    dxdt.at(11) = dpqr[1];
    dxdt.at(12) = dpqr[2];

    // std::cout<<std::setw(16)<<"fuck uav post" << x.head(3).transpose() <<std::endl;
    // std::cout<<std::setw(16)<<"fuck uav acc" <<  dxdt.segment<3>(3).transpose() <<std::endl;
    // std::cout<<std::setw(16)<<"fuck uav input force" <<  force_.transpose() <<std::endl;
    // update linear acc and angular acc
   
    // Normalize quaternion after integration
    // double qw = state_.at(6), qx = state_.at(7), qy = state_.at(8), qz = state_.at(9);
    // double norm = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
    // state_.at(6) /= norm;
    // state_.at(7) /= norm;
    // state_.at(8) /= norm;
    // state_.at(9) /= norm;
    // is_recursing = false;

    // accs_.linear_acc = dxdt.segment<3>(3);
    // SetLinearAcc(dxdt.segment<3>(3));
    // accs_.angular_acc = dxdt.tail(3);
    // SetAngularAcc(dxdt.tail(3));

    Eigen::Vector3d linear_acc(dxdt.at(3), dxdt.at(4), dxdt.at(5));
    SetLinearAcc(linear_acc);
    Eigen::Vector3d angular_acc(dxdt.at(10), dxdt.at(11), dxdt.at(12));
    SetAngularAcc(angular_acc);
    // std::cout<<std::setw(16)<<"accs_.linear_acc is " <<  accs_.linear_acc.transpose() <<std::endl;
}



void RigidBody::DoOneStepInt()
{

    // call one step integration for quadrotor dynamics
    // this->stepper_.do_step(*this, state_, current_step_, step_size_);
    // this->stepper_.do_step(std::ref(*this), state_, current_step_, step_size_);

        // typedef runge_kutta_cash_karp54<object_state> stepper_type;
    //     controlled_runge_kutta<stepper_type> controlled_stepper;

    // // Perform a single adaptive integration step
    //     integrate_adaptive(
    //         controlled_stepper,
    //         std::ref(*this), // ODE system (operator())
    //         state_,           // Current state (position and velocity)
    //         current_step_,            // Current time
    //         current_step_ + step_size_,       // End time for this step
    //         step_size_               // Initial step size (adaptive stepper adjusts it)
    //     );

        // typedef runge_kutta_cash_karp54<object_state> stepper_type;
        // typedef default_error_checker<double, array_algebra, default_operations> error_checker_type;

        // double abs_err = 1e-6;  // Absolute error tolerance
        // double rel_err = 1e-3;   // Relative error tolerance
        // // double dt = 0.01;        // Start with a smaller time step


        // controlled_runge_kutta<stepper_type, error_checker_type> controlled_stepper(error_checker_type(abs_err, rel_err));

    
        // Perform a single adaptive integration step
        // controlled_stepper.try_step(std::ref(*this), state_, current_step_, step_size_);
        // controlled_stepper_.try_step(std::ref(*this), state_, current_step_, step_size_);


        // bool success = stepper_.do_step(system, state_, current_step_, step_size_, state_err_);
        // if (!success) {
        //     std::cerr << "Step failed with the stiff solver." << std::endl;
        // }


        double current_time = current_step_;
        double end_time = current_step_ + step_size_;

        integrate_adaptive(
            controlled_stepper,
            std::ref(*this),
            state_,
            current_time,
            end_time,
            step_size_ * 0.1
        );

        current_step_ = end_time;

        // // Normalize quaternion after integration
        double qw = state_.at(6), qx = state_.at(7), qy = state_.at(8), qz = state_.at(9);
        double norm = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
        state_.at(6) /= norm;
        state_.at(7) /= norm;
        state_.at(8) /= norm;
        state_.at(9) /= norm;


    // update current step
    // current_step_ = current_step_ + step_size_;
    //    current_step_ =  
    // int sub_steps = 10;  // Adjust as needed
    // double sub_step_size = step_size_ / sub_steps;
    
    // for (int i = 0; i < sub_steps; ++i)
    // {
    //     stepper_.do_step(std::ref(*this), state_, current_step_, sub_step_size);
    //     current_step_ += sub_step_size;
        
    //     // Normalize quaternion after each sub-step
    //     double qw = state_.at(6), qx = state_.at(7), qy = state_.at(8), qz = state_.at(9);
    //     double norm = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
    //     state_.at(6) /= norm;
    //     state_.at(7) /= norm;
    //     state_.at(8) /= norm;
    //     state_.at(9) /= norm;
    // }    

};
  


void RigidBody::SetInitialPost(const Eigen::Vector3d &initial_post)
{
    
    // state vecgor for a rigid body (12X1) including position, velcity, euler angle, bodyrate, 
    // state_ = [x,     y,      z,      dx,     dy,     dz,     qnw,      qnx,          qny,      qnz,    p,      q,      r]
    // state_.head(3) = initial_post;
    state_.at(0) = initial_post[0];
    state_.at(1) = initial_post[1];
    state_.at(2) = initial_post[2];
    // std::cout<<std::setw(16)<< "[----------] RigidBody: SetInitialPost state is " << state_.transpose()<<std::endl;
}; 



void RigidBody::SetPost(const Eigen::Vector3d &object_post)
{

    // state_.head<3>() = object_post;
    state_.at(0) = object_post[0];
    state_.at(1) = object_post[1];
    state_.at(2) = object_post[2];
};



void RigidBody::SetInitialAttitude(const double &phi, const double &theta, const double &psi)
{
    // recall that state_ = [x,     y,      z,      dx,     dy,     dz,     qnw,      qnx,          qny,      qnz,    p,      q,      r]
    // state_[6] = phi;
    // state_[7] = theta;
    // state_[8] = psi;
    // std::cout<<std::setw(16)<< "initial state" << state_.transpose()<<std::endl;
    // std::cout<<std::setw(16)<< "initial attitude" << state_.segment<3>(6)<<std::endl;

    Eigen::Quaterniond qn;
    
    qn = Eigen::AngleAxisd(psi, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitX())  ;

    qn.normalize();

    // state_[6] = qn.w();
    // state_[7] = qn.x();
    // state_[8] = qn.y();
    // state_[9] = qn.z();
    state_.at(6) = qn.w();
    state_.at(7) = qn.x();
    state_.at(8) = qn.y();
    state_.at(9) = qn.z();
};


void RigidBody::SetLinearVel(const Eigen::Vector3d &object_vel)
{
    // state_.segment<3>(3) = object_vel;
    state_.at(3) = object_vel[0];
    state_.at(4) = object_vel[1];
    state_.at(5) = object_vel[2];
};

void RigidBody::SetLinearAcc(const Eigen::Vector3d &object_acc)
{
    accs_.linear_acc = object_acc;
};

// update objec' bodyrate with input
void RigidBody::SetBodyrate(const Eigen::Vector3d &object_bodyrate)
{

    // state_.tail(3) = object_bodyrate;
    state_.at(10) = object_bodyrate[0];
    state_.at(11) = object_bodyrate[1];
    state_.at(12) = object_bodyrate[2];
}

void RigidBody::SetAngularAcc(const Eigen::Vector3d &object_bodyrate_acc)
{
    accs_.angular_acc = object_bodyrate_acc;
};







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
    // auto robot_post = state_.head<3>();
    Eigen::Vector3d robot_post(state_.at(0), state_.at(1), state_.at(2));

    // auto robot_att = Eigen::AngleAxisd(state_(8), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(state_(7),Eigen::Vector3d::UnitY()) *Eigen::AngleAxisd(state_(6), Eigen::Vector3d::UnitX());

    // robot_att.normalize();

    Eigen::Quaterniond robot_att = Eigen::Quaterniond(state_.at(6), state_.at(7), state_.at(8), state_.at(9));
    robot_att.normalize();

    Pose robot_pose(robot_post, robot_att);

    return robot_pose;
};


Vels RigidBody::vels() const
{
    //x =  [x,     y,      z,      dx,     dy,     dz,     qnw,      qnx,          qny,      qnz,    p,      q,      r]
    // auto robot_linear_vel = state_.segment<3>(3);
    // auto robot_angular_rate = state_.tail<3>(); 
    Eigen::Vector3d robot_linear_vel(state_.at(3), state_.at(4), state_.at(5));
    Eigen::Vector3d robot_angular_rate(state_.at(10), state_.at(11), state_.at(12));


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


Eigen::Vector4d RigidBody::ComputeQuaternionDerivative(const Eigen::Quaterniond &qn, const Eigen::Vector3d &bodyrate)
{
    // // Quaternion representing the angular velocity (0, bodyrate_x, bodyrate_y, bodyrate_z)
    // Eigen::Quaterniond omega_q(0, bodyrate(0), bodyrate(1), bodyrate(2));

    // // Quaternion derivative
    // // Eigen::Quaterniond dqn = 0.5 * (qn * omega_q);
    // Eigen::Quaterniond dqn =  omega_q * qn;  // Quaternion multiplication
    // dqn.coeffs() = dqn.coeffs() * 0.5;   


    // // Normalize the result to ensure the quaternion stays normalized
    // dqn.normalize();

    // return dqn;

     // Extract quaternion components
    double qW = qn.w();
    double qX = qn.x();
    double qY = qn.y();
    double qZ = qn.z();

    double p = bodyrate(0);
    double q = bodyrate(1);
    double r = bodyrate(2);

    const double K_quat = 0.5;
    double quaterror = 1.0 - (qW * qW + qX * qX + qY * qY + qZ * qZ);

    // Define the angular velocity matrix
    Eigen::Matrix4d omega_matrix;
    omega_matrix << 0, -p, -q, -r,
                    p, 0, r, -q,
                    q, -r, 0, p,
                    r, q, -p, 0;
    Eigen::Vector4d v_qn(qn.w(), qn.x(), qn.y(), qn.z());    

    // Calculate the quaternion derivative
    Eigen::Vector4d term1 = omega_matrix * v_qn;  // Direct calculation
    //coud omega_matrix
    // std::cout<<"omega_matrix is "<< omega_matrix<<std::endl;
    //cout quat.coeffs()
    // std::cout<<"v_quad is "<< v_quad.transpose()<<std::endl;
    // std::cout<<"term1 is "<< term1.transpose()<<std::endl;
    auto term2= K_quat * quaterror * v_qn;  // Add error term
    // std::cout<<"term 2 is "<< term2.transpose()<<std::endl;

    Eigen::Vector4d qLdot = 0.5*term1 + term2;
    // std::cout<<"qLdot is "<< qLdot.transpose()<<std::endl;
    return qLdot;
}



std::array<double, 3> RigidBody::EigenToArray(const Eigen::Vector3d& vec)
{
    std::array<double, 3> arr;
    arr[0] = vec(0);
    arr[1] = vec(1);
    arr[2] = vec(2);
    return arr;
}


Eigen::Vector3d RigidBody::ArrayToEigen(const std::array<double, 3>& arr)
{
    Eigen::Vector3d vec;
    vec << arr[0], arr[1], arr[2];
    return vec;
}

