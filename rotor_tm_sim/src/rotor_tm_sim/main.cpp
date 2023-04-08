#include <iostream>
#include "rotor_tm_sim/lib_quadrotor_dynamic_simulator.hpp"

using namespace boost::numeric::odeint;

int main()
{
    quadrotor_state drone;
    drone.setZero();

    double mass =1;

    Eigen::Matrix3d m_inertia = Eigen::Matrix3d::Identity(3,3);
    m_inertia(0,0)=0.1;
    m_inertia(1,1)=0.1;
    m_inertia(2,2)=0.2;      

    const double dt = 0.01;
    double t = 3*dt;


    QuadrotorDynamicSimulator quadrotor_dynmaic(mass, m_inertia, dt);

    Eigen::Vector3d thrust(1,2,0);

    Eigen::Vector3d torque(0,0,0);
   
    quadrotor_dynmaic.inputThurstForce(thrust);

    quadrotor_dynmaic.inputTorque(torque);



    Eigen::Vector3d drone_position;
    Eigen::Vector3d drone_vel;
    double time;


    for( double t=0.0 ; t<0.5 ; t+= 2*dt )
    {
        // stepper.do_step(rhs , drone , t , dt);
        quadrotor_dynmaic.doOneStepInt();

        quadrotor_dynmaic.getPosition(drone_position);

        quadrotor_dynmaic.getVel(drone_vel);
        
        quadrotor_dynmaic.getCurrentTimeStep(time);

        std::cout<<"current time is "<<time<<std::endl;    

        std::cout<<"position is "<<drone_position.transpose()<<std::endl;    

        std::cout<<"vel is "<<drone_vel.transpose()<<std::endl; 
    }    


}