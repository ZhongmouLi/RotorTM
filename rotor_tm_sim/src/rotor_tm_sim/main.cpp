#include <iostream>
#include <boost/numeric/odeint.hpp>
#include <Eigen/Dense>
#include <boost/numeric/odeint/external/eigen/eigen_algebra.hpp>
// #include <Eigen/src/Core/util/Macros.h>
#include <boost/numeric/odeint/external/eigen/eigen.hpp>


using namespace std;
using namespace boost::numeric::odeint;

typedef Eigen::Matrix<double, 12, 1> quadrotor_state;
// typedef Eigen::Vector2d quadrotor_state;

Eigen::Vector3d quadRotDynac(const Eigen::Vector3d &torque, const Eigen::Matrix3d &Inertia, const Eigen::Vector3d &bodyrate)
{
    Eigen::Vector3d dBodyRate = Eigen::Vector3d::Zero();

    dBodyRate = Inertia.ldlt().solve(-bodyrate.cross(Inertia*bodyrate) + torque);

    return dBodyRate;
}

Eigen::Vector3d quadTransDynac(const Eigen::Vector3d &Thurst, const double &mass, const double &gravity)
{   
    Eigen::Vector3d acc = Eigen::Vector3d::Zero();

    acc = (Thurst-mass*gravity*Eigen::Vector3d::UnitZ())/mass;

    return acc;
}

Eigen::Vector3d quadBodyrate2Eulerrate(const Eigen::Vector3d &bodyrate, const Eigen::Matrix3d &trans_matrix)
{

    Eigen::Vector3d Eulerrate = Eigen::Vector3d::Zero();

    Eulerrate = trans_matrix.ldlt().solve(bodyrate);

    return Eulerrate;

}

Eigen::Matrix3d quadTransMatrix(const double &phi, const double &theta, const double &psi)
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


void rhs(const quadrotor_state &x , quadrotor_state &dxdt , const double t )
{
    Eigen::Vector3d thrust(1,2,0);
    double mass =1;

    Eigen::Vector3d torque(0,0,0);
    Eigen::Matrix3d Inertia = Eigen::Matrix3d::Identity(3,3);
    Inertia(0,0)=0.1;
    Inertia(1,1)=0.1;
    Inertia(2,2)=0.2;        

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

    // translation in world frame
    // P = [x,y,z,dx, dy, dz]
    // dP = [dx, dy, dz, ddx, ddy, ddz]
    dpost_v(0) = post_v(3);
    dpost_v(1) = post_v(4);
    dpost_v(2) = post_v(5);
    // [ddx ddy ddz] = (F-mg)/m
    dpost_v.tail(3) = quadTransDynac(thrust, mass, 9.8);

    // attitude in body frame
    // att_v = [phi, theta, psi, p, q, r]
    // datt_v = [dphi, dtheta, dpsi, dp, dq, dr]

    // 
    Eigen::Matrix3d trans_matrix;
    trans_matrix = quadTransMatrix(att_v(0), att_v(1), att_v(2));

    datt_v.head(3) = quadBodyrate2Eulerrate(bodyrate, trans_matrix);

    // 
    datt_v.tail(3) = quadRotDynac(torque, Inertia, bodyrate);


    // assign
    dxdt.head(6) =  dpost_v;
    dxdt.tail(6) =  datt_v;

}



int main()
{
    quadrotor_state drone;
    drone.setZero();


    runge_kutta4<quadrotor_state> stepper;
    
    const double dt = 0.01;

    for( double t=0.0 ; t<1.0 ; t+= dt )
    {
        stepper.do_step(rhs , drone , t , dt);
        // std::cout<<"at step "<<t<< " int post error " << drone(0)-0.5*9.8*(t+dt)*(t+dt) << std::endl;
        
        // std::cout<<"at step "<<t<<  " int vel is "<< drone(1)-9.8*(t+dt) << std::endl;

        // std::cout<<"at step "<<t<< " state is "<< drone.transpose()<<std::endl; 

        std::cout<<"at step "<<t<< " z error   " << drone(2) + 0.5*9.8*(t+dt)*(t+dt) << std::endl;

        std::cout<<"at step "<<t<< " y error   " << drone(1) - 0.5*2*(t+dt)*(t+dt) << std::endl;

        std::cout<<"at step "<<t<< " x error   " << drone(0) - 0.5*1*(t+dt)*(t+dt) << std::endl;
    }    
}