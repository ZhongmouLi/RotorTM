#include <iostream> 
#include <gtest/gtest.h>
#include <memory>
#include <cmath>
#include <cstdlib>
#include "rotor_tm_sim/lib_quadrotor_dynamic_simulator.hpp"


class rotorTMTest : public ::testing::Test
{
public:

rotorTMTest(){
    Eigen::Matrix3d m_inertia = Eigen::Matrix3d::Identity(3,3);

    double mass =1;

    double dt = 0.01;

    ptr_drone = std::make_shared<QuadrotorDynamicSimulator>(mass, m_inertia, dt);
}

~rotorTMTest(){
}

protected:
    std::shared_ptr<QuadrotorDynamicSimulator> ptr_drone;
};


TEST_F(rotorTMTest, checkGTest){
    // test if gTest is well integrated
    ASSERT_TRUE(true);
}

TEST_F(rotorTMTest, checkInstanceClass){
    // test if instance is created
    ASSERT_TRUE(ptr_drone!=nullptr);
}

TEST_F(rotorTMTest, calHoverEqulibirum){

    // define force to compensite gravity and zero torque
    const Eigen::Vector3d force(0, 0, 9.8);

    const Eigen::Vector3d torque(0, 0, 0);
    
    // input force and torque
    ptr_drone->inputForce(force);
    ptr_drone->inputTorque(torque);

    // do one integration
    ptr_drone->doOneStepInt();

    // check position = 0.5gt^2
    Eigen::Vector3d pos = Eigen::Vector3d::Random();

    ptr_drone->getPosition(pos);

    ASSERT_EQ(pos[0], 0); 
    ASSERT_EQ(pos[1], 0); 
    ASSERT_EQ(pos[2], 0);   

    // check vel = gt
    Eigen::Vector3d vel = Eigen::Vector3d::Random();
    ptr_drone->getVel(vel);

    ASSERT_EQ(vel[0], 0); 
    ASSERT_EQ(vel[1], 0); 
    ASSERT_EQ(vel[2], 0);  

    // check attitude = 0
    Eigen::Quaterniond att = Eigen::Quaterniond::UnitRandom();
    ptr_drone->getAttitude(att);
    ASSERT_EQ(att.x(), 0); 
    ASSERT_EQ(att.y(), 0); 
    ASSERT_EQ(att.z(), 0);  
    ASSERT_EQ(att.w(), 1);  

    // check bodyrate =0
    Eigen::Vector3d bodyrate = Eigen::Vector3d::Random();
    ptr_drone->getBodyrate(bodyrate);
    ASSERT_EQ(bodyrate[0], 0); 
    ASSERT_EQ(bodyrate[1], 0); 
    ASSERT_EQ(bodyrate[2], 0);  


    //check time step = dt
    double step_current = 0;
    ptr_drone->getCurrentTimeStep(step_current);
    ASSERT_EQ(step_current, 0.01);  
}


TEST_F(rotorTMTest, applyForce4Zdirection){

    // define force in Z direction and zero torque
    const Eigen::Vector3d force(0, 0, 15);

    const Eigen::Vector3d torque(0, 0, 0);
    
    // input force and torque
    ptr_drone->inputForce(force);
    ptr_drone->inputTorque(torque);

    // do one integration
    ptr_drone->doOneStepInt();

    // check position = 0.5gt^2
    Eigen::Vector3d pos = Eigen::Vector3d::Random();

    ptr_drone->getPosition(pos);

    ASSERT_EQ(pos[0], 0); 
    ASSERT_EQ(pos[1], 0); 
    EXPECT_FLOAT_EQ(pos[2], 0.00026);   

    // check vel = gt
    Eigen::Vector3d vel = Eigen::Vector3d::Random();
    ptr_drone->getVel(vel);

    ASSERT_EQ(vel[0], 0); 
    ASSERT_EQ(vel[1], 0); 
    EXPECT_FLOAT_EQ(vel[2], 0.05199999999999999);  

    // check attitude = 0
    Eigen::Quaterniond att = Eigen::Quaterniond::UnitRandom();
    ptr_drone->getAttitude(att);
    ASSERT_EQ(att.x(), 0); 
    ASSERT_EQ(att.y(), 0); 
    ASSERT_EQ(att.z(), 0);  
    ASSERT_EQ(att.w(), 1);  

    // check bodyrate =0
    Eigen::Vector3d bodyrate = Eigen::Vector3d::Random();
    ptr_drone->getBodyrate(bodyrate);
    ASSERT_EQ(bodyrate[0], 0); 
    ASSERT_EQ(bodyrate[1], 0); 
    ASSERT_EQ(bodyrate[2], 0);  


    //check time step = dt
    double step_current = 0;
    ptr_drone->getCurrentTimeStep(step_current);
    ASSERT_EQ(step_current, 0.01);  
}

TEST_F(rotorTMTest, applyRandomForceOneStep){

    // define radom force and zero torque
    const Eigen::Vector3d force = Eigen::Vector3d::Random();

    const Eigen::Vector3d torque(0, 0, 0);
    
    // input force and torque
    ptr_drone->inputForce(force);
    ptr_drone->inputTorque(torque);

    // do one integration
    ptr_drone->doOneStepInt();

    // check position = 0.5gt^2
    Eigen::Vector3d pos = Eigen::Vector3d::Random();

    ptr_drone->getPosition(pos);

    EXPECT_FLOAT_EQ(pos[0], 0.5 * (force[0]) * 0.01 *0.01); 
    EXPECT_FLOAT_EQ(pos[1], 0.5 * (force[1]) * 0.01 *0.01); 
    EXPECT_FLOAT_EQ(pos[2], 0.5 * (force[2]-9.8) * 0.01 *0.01); 

    // check vel = gt
    Eigen::Vector3d vel = Eigen::Vector3d::Random();
    ptr_drone->getVel(vel);

    EXPECT_FLOAT_EQ(vel[0], (force[0]) *0.01);
    EXPECT_FLOAT_EQ(vel[1], (force[1]) *0.01);
    EXPECT_FLOAT_EQ(vel[2], (force[2]-9.8) *0.01);

    // check attitude = 0
    Eigen::Quaterniond att = Eigen::Quaterniond::UnitRandom();
    ptr_drone->getAttitude(att);
    ASSERT_EQ(att.x(), 0); 
    ASSERT_EQ(att.y(), 0); 
    ASSERT_EQ(att.z(), 0);  
    ASSERT_EQ(att.w(), 1);  

    // check bodyrate =0
    Eigen::Vector3d bodyrate = Eigen::Vector3d::Random();
    ptr_drone->getBodyrate(bodyrate);
    ASSERT_EQ(bodyrate[0], 0); 
    ASSERT_EQ(bodyrate[1], 0); 
    ASSERT_EQ(bodyrate[2], 0);  


    //check time step = dt
    double step_current = 0;
    ptr_drone->getCurrentTimeStep(step_current);
    ASSERT_EQ(step_current, 0.01);  
}

TEST_F(rotorTMTest, applyIdentityForceTwoSteps){

    // define radom force and zero torque
    // const Eigen::Vector3d force = Eigen::Vector3d::Random();
    const Eigen::Vector3d force(1, 1, 1);

    const Eigen::Vector3d torque(0, 0, 0);
    
    // input force and torque
    ptr_drone->inputForce(force);
    ptr_drone->inputTorque(torque);

    // do two steps integration
    const double dt = 0.01;
    for(double t=0.0 ; t<2*dt ; t+= dt)
    {
            ptr_drone->doOneStepInt();
            // printf("current step is %.3f", t);
    }
    

    // check position = 0.5gt^2
    Eigen::Vector3d pos = Eigen::Vector3d::Random();

    ptr_drone->getPosition(pos);

    EXPECT_FLOAT_EQ(pos[0], 0.5 * (force[0]) * pow(0.02,2)); 
    EXPECT_FLOAT_EQ(pos[1], 0.5 * (force[1]) * pow(0.02,2)); 
    EXPECT_FLOAT_EQ(pos[2], 0.5 * (force[2]-9.8) * pow(0.02,2)); 

    // check vel = gt
    Eigen::Vector3d vel = Eigen::Vector3d::Random();
    ptr_drone->getVel(vel);

    EXPECT_FLOAT_EQ(vel[0], (force[0]) *0.02);
    EXPECT_FLOAT_EQ(vel[1], (force[1]) *0.02);
    EXPECT_FLOAT_EQ(vel[2], (force[2]-9.8) *0.02);

    // check attitude = 0
    Eigen::Quaterniond att = Eigen::Quaterniond::UnitRandom();
    ptr_drone->getAttitude(att);
    ASSERT_EQ(att.x(), 0); 
    ASSERT_EQ(att.y(), 0); 
    ASSERT_EQ(att.z(), 0);  
    ASSERT_EQ(att.w(), 1);  

    // check bodyrate =0
    Eigen::Vector3d bodyrate = Eigen::Vector3d::Random();
    ptr_drone->getBodyrate(bodyrate);
    ASSERT_EQ(bodyrate[0], 0); 
    ASSERT_EQ(bodyrate[1], 0); 
    ASSERT_EQ(bodyrate[2], 0);  


    //check time step = dt
    double step_current = 0;
    ptr_drone->getCurrentTimeStep(step_current);
    ASSERT_EQ(step_current, 0.02);  
}

TEST_F(rotorTMTest, applyRandomnForceTwoSteps){

    // define radom force and zero torque
    const Eigen::Vector3d force = Eigen::Vector3d::Random();
    //const Eigen::Vector3d force(1, 1, 1);

    const Eigen::Vector3d torque(0, 0, 0);
    
    // input force and torque
    ptr_drone->inputForce(force);
    ptr_drone->inputTorque(torque);

    // do two steps integration
    const double dt = 0.01;
    for(double t=dt ; t<=2*dt ; t+= dt)
    {
            ptr_drone->doOneStepInt();
            // printf("current step is %.3f \n", t);
    }
    

    // check position = 0.5gt^2
    Eigen::Vector3d pos = Eigen::Vector3d::Random();

    ptr_drone->getPosition(pos);

    EXPECT_FLOAT_EQ(pos[0], 0.5 * (force[0]) * pow(0.02,2)); 
    EXPECT_FLOAT_EQ(pos[1], 0.5 * (force[1]) * pow(0.02,2)); 
    EXPECT_FLOAT_EQ(pos[2], 0.5 * (force[2]-9.8) * pow(0.02,2)); 

    // check vel = gt
    Eigen::Vector3d vel = Eigen::Vector3d::Random();
    ptr_drone->getVel(vel);

    EXPECT_FLOAT_EQ(vel[0], (force[0]) *0.02);
    EXPECT_FLOAT_EQ(vel[1], (force[1]) *0.02);
    EXPECT_FLOAT_EQ(vel[2], (force[2]-9.8) *0.02);

    // check attitude = 0
    Eigen::Quaterniond att = Eigen::Quaterniond::UnitRandom();
    ptr_drone->getAttitude(att);
    ASSERT_EQ(att.x(), 0); 
    ASSERT_EQ(att.y(), 0); 
    ASSERT_EQ(att.z(), 0);  
    ASSERT_EQ(att.w(), 1);  

    // check bodyrate =0
    Eigen::Vector3d bodyrate = Eigen::Vector3d::Random();
    ptr_drone->getBodyrate(bodyrate);
    ASSERT_EQ(bodyrate[0], 0); 
    ASSERT_EQ(bodyrate[1], 0); 
    ASSERT_EQ(bodyrate[2], 0);  


    //check time step = dt
    double step_current = 0;
    ptr_drone->getCurrentTimeStep(step_current);
    ASSERT_EQ(step_current, 0.02);  
}

TEST_F(rotorTMTest, applyRandomnForceTenSteps){

    // define radom force and zero torque
    const Eigen::Vector3d force = Eigen::Vector3d::Random();
    //const Eigen::Vector3d force(1, 1, 1);

    const Eigen::Vector3d torque(0, 0, 0);
    
    // input force and torque
    ptr_drone->inputForce(force);
    ptr_drone->inputTorque(torque);

    // do 10 steps integration
    const double dt = 0.01;
    for(double t=dt ; t<=10*dt ; t+= dt)
    {
            ptr_drone->doOneStepInt();
            // printf("current step is %.3f \n", t);
    }
    

    // check position = 0.5gt^2
    Eigen::Vector3d pos = Eigen::Vector3d::Random();

    ptr_drone->getPosition(pos);

    EXPECT_FLOAT_EQ(pos[0], 0.5 * (force[0]) * pow(10*dt,2)); 
    EXPECT_FLOAT_EQ(pos[1], 0.5 * (force[1]) * pow(10*dt,2)); 
    EXPECT_FLOAT_EQ(pos[2], 0.5 * (force[2]-9.8) * pow(10*dt,2)); 

    // check vel = gt
    Eigen::Vector3d vel = Eigen::Vector3d::Random();
    ptr_drone->getVel(vel);

    EXPECT_FLOAT_EQ(vel[0], (force[0]) *10*dt);
    EXPECT_FLOAT_EQ(vel[1], (force[1]) *10*dt);
    EXPECT_FLOAT_EQ(vel[2], (force[2]-9.8) *10*dt);

    // check attitude = 0
    Eigen::Quaterniond att = Eigen::Quaterniond::UnitRandom();
    ptr_drone->getAttitude(att);
    ASSERT_EQ(att.x(), 0); 
    ASSERT_EQ(att.y(), 0); 
    ASSERT_EQ(att.z(), 0);  
    ASSERT_EQ(att.w(), 1);  

    // check bodyrate =0
    Eigen::Vector3d bodyrate = Eigen::Vector3d::Random();
    ptr_drone->getBodyrate(bodyrate);
    ASSERT_EQ(bodyrate[0], 0); 
    ASSERT_EQ(bodyrate[1], 0); 
    ASSERT_EQ(bodyrate[2], 0);  


    //check time step = dt
    double step_current = 0;
    ptr_drone->getCurrentTimeStep(step_current);
    ASSERT_FLOAT_EQ(step_current, 10*dt);  
}

TEST_F(rotorTMTest, applyRandomnForceThousandSteps){

    // define radom force and zero torque
    const Eigen::Vector3d force = Eigen::Vector3d::Random();
    //const Eigen::Vector3d force(1, 1, 1);

    const Eigen::Vector3d torque(0, 0, 0);
    
    // input force and torque
    ptr_drone->inputForce(force);
    ptr_drone->inputTorque(torque);

    // do two steps integration
    const double dt = 0.01;
    for(double t=dt ; t<=1000*dt ; t+= dt)
    {
            ptr_drone->doOneStepInt();
            // printf("current step is %.3f \n", t);
    }
    

    // check position = 0.5gt^2
    Eigen::Vector3d pos = Eigen::Vector3d::Random();

    ptr_drone->getPosition(pos);

    EXPECT_FLOAT_EQ(pos[0], 0.5 * (force[0]) * pow(1000*dt,2)); 
    EXPECT_FLOAT_EQ(pos[1], 0.5 * (force[1]) * pow(1000*dt,2)); 
    EXPECT_FLOAT_EQ(pos[2], 0.5 * (force[2]-9.8) * pow(1000*dt,2)); 

    // check vel = gt
    Eigen::Vector3d vel = Eigen::Vector3d::Random();
    ptr_drone->getVel(vel);

    EXPECT_FLOAT_EQ(vel[0], (force[0]) *1000*dt);
    EXPECT_FLOAT_EQ(vel[1], (force[1]) *1000*dt);
    EXPECT_FLOAT_EQ(vel[2], (force[2]-9.8) *1000*dt);

    // check attitude = 0
    Eigen::Quaterniond att = Eigen::Quaterniond::UnitRandom();
    ptr_drone->getAttitude(att);
    ASSERT_EQ(att.x(), 0); 
    ASSERT_EQ(att.y(), 0); 
    ASSERT_EQ(att.z(), 0);  
    ASSERT_EQ(att.w(), 1);  

    // check bodyrate =0
    Eigen::Vector3d bodyrate = Eigen::Vector3d::Random();
    ptr_drone->getBodyrate(bodyrate);
    ASSERT_EQ(bodyrate[0], 0); 
    ASSERT_EQ(bodyrate[1], 0); 
    ASSERT_EQ(bodyrate[2], 0);  


    //check time step = dt
    double step_current = 0;
    ptr_drone->getCurrentTimeStep(step_current);
    ASSERT_FLOAT_EQ(step_current, 1000*dt);  
}

TEST_F(rotorTMTest, applyRandomnForceNonOriginThousandSteps){

    // define radom force and zero torque
    const Eigen::Vector3d force = Eigen::Vector3d::Random();
    //const Eigen::Vector3d force(1, 1, 1);

    const Eigen::Vector3d torque(0, 0, 0);
    
    // initial position of quadrotor
    const Eigen::Vector3d initial_post = Eigen::Vector3d::Random();
    ptr_drone->setInitialPost(initial_post);


    // input force and torque
    ptr_drone->inputForce(force);
    ptr_drone->inputTorque(torque);

    // do two steps integration
    const double dt = 0.01;
    for(double t=dt ; t<=1000*dt ; t+= dt)
    {
            ptr_drone->doOneStepInt();
            // printf("current step is %.3f \n", t);
    }
    

    // check position = 0.5gt^2
    Eigen::Vector3d pos = Eigen::Vector3d::Random();

    ptr_drone->getPosition(pos);

    EXPECT_FLOAT_EQ(pos[0], initial_post[0]+0.5 * (force[0]) * pow(1000*dt,2)); 
    EXPECT_FLOAT_EQ(pos[1], initial_post[1]+0.5 * (force[1]) * pow(1000*dt,2)); 
    EXPECT_FLOAT_EQ(pos[2], initial_post[2]+0.5 * (force[2]-9.8) * pow(1000*dt,2)); 

    // check vel = gt
    Eigen::Vector3d vel = Eigen::Vector3d::Random();
    ptr_drone->getVel(vel);

    EXPECT_FLOAT_EQ(vel[0], (force[0]) *1000*dt);
    EXPECT_FLOAT_EQ(vel[1], (force[1]) *1000*dt);
    EXPECT_FLOAT_EQ(vel[2], (force[2]-9.8) *1000*dt);

    // check attitude = 0
    Eigen::Quaterniond att = Eigen::Quaterniond::UnitRandom();
    ptr_drone->getAttitude(att);
    ASSERT_EQ(att.x(), 0); 
    ASSERT_EQ(att.y(), 0); 
    ASSERT_EQ(att.z(), 0);  
    ASSERT_EQ(att.w(), 1);  

    // check bodyrate =0
    Eigen::Vector3d bodyrate = Eigen::Vector3d::Random();
    ptr_drone->getBodyrate(bodyrate);
    ASSERT_EQ(bodyrate[0], 0); 
    ASSERT_EQ(bodyrate[1], 0); 
    ASSERT_EQ(bodyrate[2], 0);  


    //check time step = dt
    double step_current = 0;
    ptr_drone->getCurrentTimeStep(step_current);
    ASSERT_FLOAT_EQ(step_current, 1000*dt);  
}

TEST_F(rotorTMTest, applyTorque4XRotation){

    // define force to compensite gravity and zero torque
    const Eigen::Vector3d force(0, 0, 9.8);

    const Eigen::Vector3d torque(4.5, 0, 0);
    
    // input force and torque
    ptr_drone->inputForce(force);
    ptr_drone->inputTorque(torque);

    // do one integration
    ptr_drone->doOneStepInt();

    // check position = 0.5gt^2
    Eigen::Vector3d pos = Eigen::Vector3d::Random();

    ptr_drone->getPosition(pos);

    ASSERT_EQ(pos[0], 0); 
    ASSERT_EQ(pos[1], 0); 
    ASSERT_EQ(pos[2], 0);   

    // check vel = gt
    Eigen::Vector3d vel = Eigen::Vector3d::Random();
    ptr_drone->getVel(vel);

    ASSERT_EQ(vel[0], 0); 
    ASSERT_EQ(vel[1], 0); 
    ASSERT_EQ(vel[2], 0);  



    // check bodyrate
    Eigen::Vector3d bodyrate = Eigen::Vector3d::Random();
    ptr_drone->getBodyrate(bodyrate);
    EXPECT_FLOAT_EQ(bodyrate[0], 0.045); 
    EXPECT_FLOAT_EQ(bodyrate[1], 0); 
    EXPECT_FLOAT_EQ(bodyrate[2], 0);  

    // check attitude
    Eigen::Quaterniond att = Eigen::Quaterniond::UnitRandom();
    ptr_drone->getAttitude(att);
    EXPECT_FLOAT_EQ(att.x(), 0.00011249999976269531); 
    EXPECT_FLOAT_EQ(att.y(), 0); 
    EXPECT_FLOAT_EQ(att.z(), 0);  
    EXPECT_FLOAT_EQ(att.w(), 0.99999999367187498);  


    //check time step = dt
    double step_current = 0;
    ptr_drone->getCurrentTimeStep(step_current);
    ASSERT_EQ(step_current, 0.01);  
}

TEST_F(rotorTMTest, applyTorque4YRotation){

    // define force to compensite gravity and zero torque
    const Eigen::Vector3d force(0, 0, 9.8);

    const Eigen::Vector3d torque(0, 5.25, 0);
    
    // input force and torque
    ptr_drone->inputForce(force);
    ptr_drone->inputTorque(torque);

    // do one integration
    ptr_drone->doOneStepInt();

    // check position = 0.5gt^2
    Eigen::Vector3d pos = Eigen::Vector3d::Random();

    ptr_drone->getPosition(pos);

    ASSERT_EQ(pos[0], 0); 
    ASSERT_EQ(pos[1], 0); 
    ASSERT_EQ(pos[2], 0);   

    // check vel = gt
    Eigen::Vector3d vel = Eigen::Vector3d::Random();
    ptr_drone->getVel(vel);

    ASSERT_EQ(vel[0], 0); 
    ASSERT_EQ(vel[1], 0); 
    ASSERT_EQ(vel[2], 0);  



    // check bodyrate
    Eigen::Vector3d bodyrate = Eigen::Vector3d::Random();
    ptr_drone->getBodyrate(bodyrate);
    EXPECT_FLOAT_EQ(bodyrate[0], 0); 
    EXPECT_FLOAT_EQ(bodyrate[1], 0.0525); 
    EXPECT_FLOAT_EQ(bodyrate[2], 0);  

    // check attitude
    Eigen::Quaterniond att = Eigen::Quaterniond::UnitRandom();
    ptr_drone->getAttitude(att);
    EXPECT_FLOAT_EQ(att.x(), 0); 
    EXPECT_FLOAT_EQ(att.y(), 0.00013124999962316894); 
    EXPECT_FLOAT_EQ(att.z(), 0);  
    EXPECT_FLOAT_EQ(att.w(), 0.99999999138671880);  


    //check time step = dt
    double step_current = 0;
    ptr_drone->getCurrentTimeStep(step_current);
    ASSERT_EQ(step_current, 0.01);  
}

TEST_F(rotorTMTest, applyTorque4ZRotation){

    // define force to compensite gravity and zero torque
    const Eigen::Vector3d force(0, 0, 9.8);

    const Eigen::Vector3d torque(0, 0, 6.13);
    
    // input force and torque
    ptr_drone->inputForce(force);
    ptr_drone->inputTorque(torque);

    // do one integration
    ptr_drone->doOneStepInt();

    // check position = 0.5gt^2
    Eigen::Vector3d pos = Eigen::Vector3d::Random();

    ptr_drone->getPosition(pos);

    ASSERT_EQ(pos[0], 0); 
    ASSERT_EQ(pos[1], 0); 
    ASSERT_EQ(pos[2], 0);   

    // check vel = gt
    Eigen::Vector3d vel = Eigen::Vector3d::Random();
    ptr_drone->getVel(vel);

    ASSERT_EQ(vel[0], 0); 
    ASSERT_EQ(vel[1], 0); 
    ASSERT_EQ(vel[2], 0);  



    // check bodyrate
    Eigen::Vector3d bodyrate = Eigen::Vector3d::Random();
    ptr_drone->getBodyrate(bodyrate);
    EXPECT_FLOAT_EQ(bodyrate[0], 0); 
    EXPECT_FLOAT_EQ(bodyrate[1], 0); 
    EXPECT_FLOAT_EQ(bodyrate[2],  0.0613);  

    // check attitude
    Eigen::Quaterniond att = Eigen::Quaterniond::UnitRandom();
    ptr_drone->getAttitude(att);
    EXPECT_FLOAT_EQ(att.x(), 0); 
    EXPECT_FLOAT_EQ(att.y(), 0); 
    EXPECT_FLOAT_EQ(att.z(), 0.0001532499994001396);  
    EXPECT_FLOAT_EQ(att.w(), 0.9999999882572188);  


    //check time step = dt
    double step_current = 0;
    ptr_drone->getCurrentTimeStep(step_current);
    ASSERT_EQ(step_current, 0.01);  
}



TEST_F(rotorTMTest, applyTorque4XYZRotationTenSteps){

    // define force to compensite gravity and zero torque
    const Eigen::Vector3d force(0, 0, 9.8);

    const Eigen::Vector3d torque(0.1, 0.2, 0.3);
    
    // input force and torque
    ptr_drone->inputForce(force);
    ptr_drone->inputTorque(torque);

    // do 10 steps integration
    const double dt = 0.01;
    for(double t=dt ; t<=10*dt ; t+= dt)
    {
            ptr_drone->doOneStepInt();
            // printf("current step is %.3f \n", t);
    }
    

    // check position = 0.5gt^2
    Eigen::Vector3d pos = Eigen::Vector3d::Random();

    ptr_drone->getPosition(pos);

    ASSERT_EQ(pos[0], 0); 
    ASSERT_EQ(pos[1], 0); 
    ASSERT_EQ(pos[2], 0);   

    // check vel = gt
    Eigen::Vector3d vel = Eigen::Vector3d::Random();
    ptr_drone->getVel(vel);

    ASSERT_EQ(vel[0], 0); 
    ASSERT_EQ(vel[1], 0); 
    ASSERT_EQ(vel[2], 0);  



    // check bodyrate
    Eigen::Vector3d bodyrate = Eigen::Vector3d::Random();
    ptr_drone->getBodyrate(bodyrate);
    EXPECT_FLOAT_EQ(bodyrate[0], 0.01); 
    EXPECT_FLOAT_EQ(bodyrate[1], 0.02); 
    EXPECT_FLOAT_EQ(bodyrate[2], 0.03);  

    // check attitude
    Eigen::Quaterniond att = Eigen::Quaterniond::UnitRandom();
    ptr_drone->getAttitude(att);
    EXPECT_NEAR(att.x(), 2.5004037205422324e-4, 1e-5);     
    EXPECT_NEAR(att.y(), 5.0002658173730924e-4, 1e-5);      
    EXPECT_NEAR(att.z(), 7.5006364505842392e-4, 1e-5);     
    EXPECT_NEAR(att.w(), 9.9999956242878352e-1, 1e-5);      


    //check time step = dt
    double step_current = 0;
    ptr_drone->getCurrentTimeStep(step_current);
    ASSERT_FLOAT_EQ(step_current, 10*dt);  
}

TEST_F(rotorTMTest, applyTorque4XYZRotationThreeHundSteps){

    // define force to compensite gravity and zero torque
    const Eigen::Vector3d force(0, 0, 9.8);

    const Eigen::Vector3d torque(0.1, 0.2, 0.3);
    
    // input force and torque
    ptr_drone->inputForce(force);
    ptr_drone->inputTorque(torque);

    // do 10 steps integration
    const double dt = 0.01;
    for(double t=dt ; t<=300*dt ; t+= dt)
    {
            ptr_drone->doOneStepInt();
            // printf("current step is %.3f \n", t);
    }
    

    // check position = 0.5gt^2
    Eigen::Vector3d pos = Eigen::Vector3d::Random();

    ptr_drone->getPosition(pos);

    ASSERT_EQ(pos[0], 0); 
    ASSERT_EQ(pos[1], 0); 
    ASSERT_EQ(pos[2], 0);   

    // check vel = gt
    Eigen::Vector3d vel = Eigen::Vector3d::Random();
    ptr_drone->getVel(vel);

    ASSERT_EQ(vel[0], 0); 
    ASSERT_EQ(vel[1], 0); 
    ASSERT_EQ(vel[2], 0);  



    // check bodyrate
    Eigen::Vector3d bodyrate = Eigen::Vector3d::Random();
    ptr_drone->getBodyrate(bodyrate);
    EXPECT_FLOAT_EQ(bodyrate[0], 0.3); 
    EXPECT_FLOAT_EQ(bodyrate[1], 0.6); 
    EXPECT_FLOAT_EQ(bodyrate[2], 0.9);  

    // check attitude
    Eigen::Quaterniond att = Eigen::Quaterniond::UnitRandom();
    ptr_drone->getAttitude(att);
    EXPECT_NEAR(att.x(), 0.19934871398234738, 1e-5);    
    EXPECT_NEAR(att.y(), 0.3986953792988834, 1e-5);  
    EXPECT_NEAR(att.z(), 0.5980442862305684, 1e-5);  
    EXPECT_NEAR(att.w(), 0.6660669008938018, 1e-5);  


    //check time step = dt
    double step_current = 0;
    ptr_drone->getCurrentTimeStep(step_current);
    ASSERT_FLOAT_EQ(step_current, 300*dt);  
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
