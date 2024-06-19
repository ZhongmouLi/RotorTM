#include <iostream> 
#include <gtest/gtest.h>
#include <memory>
#include <cmath>
#include <cstdlib>
#include <random>
#include "rotor_tm_sim/lib_quadrotor.hpp"

double RandomGenerate(const double &minValue, const double &maxValue);

class rotorTMQuadrotorTest : public ::testing::Test
{
public:

rotorTMQuadrotorTest(){
Eigen::Matrix3d m_inertia = Eigen::Matrix3d::Identity(3,3);

    double mass =1;

    MassProperty mav_mass_property = {mass, m_inertia};

    double dt=0.01;

    ptr_quadrotor = std::make_shared<Quadrotor>(mav_mass_property, dt);
}

~rotorTMQuadrotorTest(){
}

protected:
    std::shared_ptr<Quadrotor> ptr_quadrotor;
};

// test if gTest is well integrated
TEST_F(rotorTMQuadrotorTest, checkGTest){
    ASSERT_TRUE(true);
}

// test if instance is created
TEST_F(rotorTMQuadrotorTest, checkInstanceClass){
    ASSERT_TRUE(ptr_quadrotor!=nullptr);
}

// input gravity-compensating thrust to hover
TEST_F(rotorTMQuadrotorTest, calHoverEqulibirum){

    // define force to compensite gravity and zero torque
    const double thrust  = 9.8;

    const Eigen::Vector3d torque{0, 0, 0};
    
    // input force and torque
    // ptr_quadrotor->InputThurst(thrust);
    // ptr_quadrotor->InputTorque(torque);
    ptr_quadrotor->InputDroneThrustTorque(thrust, torque);

    // do one integration
    ptr_quadrotor->DoOneStepInt();

    // check position = 0.5at^2
    Eigen::Vector3d pos = Eigen::Vector3d::Random();

    ptr_quadrotor->GetPosition(pos);

    ASSERT_EQ(pos[0], 0); 
    ASSERT_EQ(pos[1], 0); 
    ASSERT_EQ(pos[2], 0);   

    // check vel = gt
    Eigen::Vector3d vel = Eigen::Vector3d::Random();
    ptr_quadrotor->GetVel(vel);

    ASSERT_EQ(vel[0], 0); 
    ASSERT_EQ(vel[1], 0); 
    ASSERT_EQ(vel[2], 0);  

    // check acc = 0
    Eigen::Vector3d acc = Eigen::Vector3d::Random();
    ptr_quadrotor->GetAcc(acc);
    
    ASSERT_EQ(acc[0], 0); 
    ASSERT_EQ(acc[1], 0); 
    ASSERT_EQ(acc[2], 0);  


    // check attitude = 0
    Eigen::Quaterniond att = Eigen::Quaterniond::UnitRandom();
    ptr_quadrotor->GetAttitude(att);
    ASSERT_EQ(att.x(), 0); 
    ASSERT_EQ(att.y(), 0); 
    ASSERT_EQ(att.z(), 0);  
    ASSERT_EQ(att.w(), 1);  

    // check bodyrate =0
    Eigen::Vector3d bodyrate = Eigen::Vector3d::Random();
    ptr_quadrotor->GetBodyrate(bodyrate);
    ASSERT_EQ(bodyrate[0], 0); 
    ASSERT_EQ(bodyrate[1], 0); 
    ASSERT_EQ(bodyrate[2], 0);  

    // check bodyrate_acc =0
    Eigen::Vector3d bodyrate_acc = Eigen::Vector3d::Random();
    ptr_quadrotor->GetBodyRateAcc(bodyrate_acc);
    ASSERT_EQ(bodyrate_acc[0], 0); 
    ASSERT_EQ(bodyrate_acc[1], 0); 
    ASSERT_EQ(bodyrate_acc[2], 0);  



    //check time step = dt
    double step_current = 0;
    ptr_quadrotor->GetCurrentTimeStep(step_current);
    ASSERT_EQ(step_current, 0.01);  
}


// input a consant thrust to move in Z direction
TEST_F(rotorTMQuadrotorTest, calApplyConstThrust){

    // define force to compensite gravity and zero torque
    const double thrust  = 20;

    const Eigen::Vector3d torque{0, 0, 0};
    
    // input force and torque
    // ptr_quadrotor->InputThurst(thrust);
    // ptr_quadrotor->InputTorque(torque);
    ptr_quadrotor->InputDroneThrustTorque(thrust, torque);

    // do one integration
    ptr_quadrotor->DoOneStepInt();

    // check position = 0.5at^2
    Eigen::Vector3d pos = Eigen::Vector3d::Random();

    ptr_quadrotor->GetPosition(pos);

    ASSERT_EQ(pos[0], 0); 
    ASSERT_EQ(pos[1], 0); 
    ASSERT_FLOAT_EQ(pos[2], 0.5*(thrust - 9.8) * pow(0.01,2));   

    // check vel = at
    Eigen::Vector3d vel = Eigen::Vector3d::Random();
    ptr_quadrotor->GetVel(vel);

    ASSERT_EQ(vel[0], 0); 
    ASSERT_EQ(vel[1], 0); 
    ASSERT_EQ(vel[2], (thrust - 9.8) * 0.01);  

    // check acc = a
    Eigen::Vector3d acc = Eigen::Vector3d::Random();
    ptr_quadrotor->GetAcc(acc);
    
    ASSERT_EQ(acc[0], 0); 
    ASSERT_EQ(acc[1], 0); 
    ASSERT_EQ(acc[2], (thrust - 9.8));  


    // check attitude = 0
    Eigen::Quaterniond att = Eigen::Quaterniond::UnitRandom();
    ptr_quadrotor->GetAttitude(att);
    ASSERT_EQ(att.x(), 0); 
    ASSERT_EQ(att.y(), 0); 
    ASSERT_EQ(att.z(), 0);  
    ASSERT_EQ(att.w(), 1);  

    // check bodyrate =0
    Eigen::Vector3d bodyrate = Eigen::Vector3d::Random();
    ptr_quadrotor->GetBodyrate(bodyrate);
    ASSERT_EQ(bodyrate[0], 0); 
    ASSERT_EQ(bodyrate[1], 0); 
    ASSERT_EQ(bodyrate[2], 0);  

    // check bodyrate_acc =0
    Eigen::Vector3d bodyrate_acc = Eigen::Vector3d::Random();
    ptr_quadrotor->GetBodyRateAcc(bodyrate_acc);
    ASSERT_EQ(bodyrate_acc[0], 0); 
    ASSERT_EQ(bodyrate_acc[1], 0); 
    ASSERT_EQ(bodyrate_acc[2], 0);  



    //check time step = dt
    double step_current = 0;
    ptr_quadrotor->GetCurrentTimeStep(step_current);
    ASSERT_EQ(step_current, 0.01);  
}




// input a consant thrust with X rotation of 90 degress
TEST_F(rotorTMQuadrotorTest, calApplyConstThrustRotX90d){

    // define force to compensite gravity and zero torque
    const double thrust  = 20;

    const Eigen::Vector3d torque{0, 0, 0};
    
    // set quadrotot with X rotation
    ptr_quadrotor->SetInitialAttitude(90.0/180*M_PI,0,0);

    // input force and torque
    // ptr_quadrotor->InputThurst(thrust);
    // ptr_quadrotor->InputTorque(torque);
    ptr_quadrotor->InputDroneThrustTorque(thrust, torque);

    // do one integration
    ptr_quadrotor->DoOneStepInt();

    // check position = 0.5at^2
    Eigen::Vector3d pos = Eigen::Vector3d::Random();

    ptr_quadrotor->GetPosition(pos);

    ASSERT_FLOAT_EQ(pos[0], 0); 
    ASSERT_FLOAT_EQ(pos[1], -0.5 * thrust * pow(0.01,2)); 
    ASSERT_FLOAT_EQ(pos[2], -0.5 * 9.8 * pow(0.01,2));   

    // check vel = at
    Eigen::Vector3d vel = Eigen::Vector3d::Random();
    ptr_quadrotor->GetVel(vel);

    ASSERT_FLOAT_EQ(vel[0], 0); 
    ASSERT_FLOAT_EQ(vel[1], -thrust * 0.01); 
    ASSERT_FLOAT_EQ(vel[2], -9.8 * 0.01);  

    // check acc = a
    Eigen::Vector3d acc = Eigen::Vector3d::Random();
    ptr_quadrotor->GetAcc(acc);
    
    ASSERT_FLOAT_EQ(acc[0], 0); 
    ASSERT_FLOAT_EQ(acc[1], -thrust); 
    ASSERT_FLOAT_EQ(acc[2], -ptr_quadrotor->gravity_);  


    // check attitude = RotX(90d)
    Eigen::Quaterniond att = Eigen::Quaterniond::UnitRandom();
    ptr_quadrotor->GetAttitude(att);
    ASSERT_FLOAT_EQ(att.x(), 0.7071068); 
    ASSERT_FLOAT_EQ(att.y(), 0); 
    ASSERT_FLOAT_EQ(att.z(), 0);  
    ASSERT_FLOAT_EQ(att.w(), 0.7071068);  

    // check bodyrate =0
    Eigen::Vector3d bodyrate = Eigen::Vector3d::Random();
    ptr_quadrotor->GetBodyrate(bodyrate);
    ASSERT_EQ(bodyrate[0], 0); 
    ASSERT_EQ(bodyrate[1], 0); 
    ASSERT_EQ(bodyrate[2], 0);  

    // check bodyrate_acc =0
    Eigen::Vector3d bodyrate_acc = Eigen::Vector3d::Random();
    ptr_quadrotor->GetBodyRateAcc(bodyrate_acc);
    ASSERT_EQ(bodyrate_acc[0], 0); 
    ASSERT_EQ(bodyrate_acc[1], 0); 
    ASSERT_EQ(bodyrate_acc[2], 0);  



    //check time step = dt
    double step_current = 0;
    ptr_quadrotor->GetCurrentTimeStep(step_current);
    ASSERT_EQ(step_current, 0.01);  
}





// input a random thrust with X rotation of 90 degress
TEST_F(rotorTMQuadrotorTest, calApplyRandThrustRotX90d){

    // define force to compensite gravity and zero torque
    const double thrust  = RandomGenerate(-10, 10);
    // std::cout<< "[----------] random thrust is " << thrust << std::endl;

    const Eigen::Vector3d torque{0, 0, 0};
    
    // set quadrotot with X rotation
    ptr_quadrotor->SetInitialAttitude(90.0/180*M_PI,0,0);

    // input force and torque
    // ptr_quadrotor->InputThurst(thrust);
    // ptr_quadrotor->InputTorque(torque);
    ptr_quadrotor->InputDroneThrustTorque(thrust, torque);

    // do one integration
    ptr_quadrotor->DoOneStepInt();

    // check position = 0.5at^2
    Eigen::Vector3d pos = Eigen::Vector3d::Random();

    ptr_quadrotor->GetPosition(pos);

    ASSERT_FLOAT_EQ(pos[0], 0); 
    ASSERT_FLOAT_EQ(pos[1], -0.5 * thrust * pow(0.01,2)); 
    ASSERT_FLOAT_EQ(pos[2], -0.5 * 9.8 * pow(0.01,2));   

    // check vel = at
    Eigen::Vector3d vel = Eigen::Vector3d::Random();
    ptr_quadrotor->GetVel(vel);

    ASSERT_FLOAT_EQ(vel[0], 0); 
    ASSERT_FLOAT_EQ(vel[1], -thrust * 0.01); 
    ASSERT_FLOAT_EQ(vel[2], -9.8 * 0.01);  

    // check acc = a
    Eigen::Vector3d acc = Eigen::Vector3d::Random();
    ptr_quadrotor->GetAcc(acc);
    
    ASSERT_FLOAT_EQ(acc[0], 0); 
    ASSERT_FLOAT_EQ(acc[1], -thrust); 
    ASSERT_FLOAT_EQ(acc[2], -ptr_quadrotor->gravity_);  


    // check attitude = RotX(90d)
    Eigen::Quaterniond att = Eigen::Quaterniond::UnitRandom();
    ptr_quadrotor->GetAttitude(att);
    ASSERT_FLOAT_EQ(att.x(), 0.7071068); 
    ASSERT_FLOAT_EQ(att.y(), 0); 
    ASSERT_FLOAT_EQ(att.z(), 0);  
    ASSERT_FLOAT_EQ(att.w(), 0.7071068);  

    // check bodyrate =0
    Eigen::Vector3d bodyrate = Eigen::Vector3d::Random();
    ptr_quadrotor->GetBodyrate(bodyrate);
    ASSERT_EQ(bodyrate[0], 0); 
    ASSERT_EQ(bodyrate[1], 0); 
    ASSERT_EQ(bodyrate[2], 0);  

    // check bodyrate_acc =0
    Eigen::Vector3d bodyrate_acc = Eigen::Vector3d::Random();
    ptr_quadrotor->GetBodyRateAcc(bodyrate_acc);
    ASSERT_EQ(bodyrate_acc[0], 0); 
    ASSERT_EQ(bodyrate_acc[1], 0); 
    ASSERT_EQ(bodyrate_acc[2], 0);  



    //check time step = dt
    double step_current = 0;
    ptr_quadrotor->GetCurrentTimeStep(step_current);
    ASSERT_EQ(step_current, 0.01);  
}




// input a random thrust with Y rotation of 90 degress
TEST_F(rotorTMQuadrotorTest, calApplyRandThrustRotY90d){

    // define force to compensite gravity and zero torque
    const double thrust  = RandomGenerate(-10, 10);
    // std::cout<< "[----------] random thrust is " << thrust << std::endl;

    const Eigen::Vector3d torque{0, 0, 0};
    
    // set quadrotot with X rotation
    ptr_quadrotor->SetInitialAttitude(0,90.0/180*M_PI,0);

    // input force and torque
    // ptr_quadrotor->InputThurst(thrust);
    // ptr_quadrotor->InputTorque(torque);
    ptr_quadrotor->InputDroneThrustTorque(thrust, torque);

    // do one integration
    ptr_quadrotor->DoOneStepInt();

    // check position = 0.5at^2
    Eigen::Vector3d pos = Eigen::Vector3d::Random();

    ptr_quadrotor->GetPosition(pos);

    ASSERT_FLOAT_EQ(pos[0], 0.5 * thrust * pow(0.01,2)); 
    ASSERT_FLOAT_EQ(pos[1], 0); 
    ASSERT_FLOAT_EQ(pos[2], -0.5 * 9.8 * pow(0.01,2));   

    // check vel = at
    Eigen::Vector3d vel = Eigen::Vector3d::Random();
    ptr_quadrotor->GetVel(vel);

    ASSERT_FLOAT_EQ(vel[0], thrust * 0.01); 
    ASSERT_FLOAT_EQ(vel[1], 0); 
    ASSERT_FLOAT_EQ(vel[2], -9.8 * 0.01);  

    // check acc = a
    Eigen::Vector3d acc = Eigen::Vector3d::Random();
    ptr_quadrotor->GetAcc(acc);
    
    ASSERT_FLOAT_EQ(acc[0], thrust); 
    ASSERT_FLOAT_EQ(acc[1], 0); 
    ASSERT_FLOAT_EQ(acc[2], -ptr_quadrotor->gravity_);  


    // check attitude = RotY(90d)
    Eigen::Quaterniond att = Eigen::Quaterniond::UnitRandom();
    ptr_quadrotor->GetAttitude(att);
    ASSERT_FLOAT_EQ(att.x(), 0); 
    ASSERT_FLOAT_EQ(att.y(), 0.7071068); 
    ASSERT_FLOAT_EQ(att.z(), 0);  
    ASSERT_FLOAT_EQ(att.w(), 0.7071068);  

    // check bodyrate =0
    Eigen::Vector3d bodyrate = Eigen::Vector3d::Random();
    ptr_quadrotor->GetBodyrate(bodyrate);
    ASSERT_EQ(bodyrate[0], 0); 
    ASSERT_EQ(bodyrate[1], 0); 
    ASSERT_EQ(bodyrate[2], 0);  

    // check bodyrate_acc =0
    Eigen::Vector3d bodyrate_acc = Eigen::Vector3d::Random();
    ptr_quadrotor->GetBodyRateAcc(bodyrate_acc);
    ASSERT_EQ(bodyrate_acc[0], 0); 
    ASSERT_EQ(bodyrate_acc[1], 0); 
    ASSERT_EQ(bodyrate_acc[2], 0);  

    //check time step = dt
    double step_current = 0;
    ptr_quadrotor->GetCurrentTimeStep(step_current);
    ASSERT_EQ(step_current, 0.01);  
}



// input a random thrust with Y rotation of 90 degress
TEST_F(rotorTMQuadrotorTest, calApplyRandThrustRotZ90d){

    // define force to compensite gravity and zero torque
    const double thrust  = RandomGenerate(-10, 10);
    // std::cout<< "[----------] random thrust is " << thrust << std::endl;

    const Eigen::Vector3d torque{0, 0, 0};
    
    // set quadrotot with X rotation
    ptr_quadrotor->SetInitialAttitude(0,0,90.0/180*M_PI);

    // input force and torque
    // ptr_quadrotor->InputThurst(thrust);
    // ptr_quadrotor->InputTorque(torque);
    ptr_quadrotor->InputDroneThrustTorque(thrust, torque);

    // do one integration
    ptr_quadrotor->DoOneStepInt();

    // check position = 0.5at^2
    Eigen::Vector3d pos = Eigen::Vector3d::Random();

    ptr_quadrotor->GetPosition(pos);

    ASSERT_FLOAT_EQ(pos[0], 0); 
    ASSERT_FLOAT_EQ(pos[1], 0); 
    ASSERT_FLOAT_EQ(pos[2], 0.5 * (thrust - ptr_quadrotor->gravity_) * pow(0.01,2));   

    // check vel = at
    Eigen::Vector3d vel = Eigen::Vector3d::Random();
    ptr_quadrotor->GetVel(vel);

    ASSERT_FLOAT_EQ(vel[0], 0); 
    ASSERT_FLOAT_EQ(vel[1], 0); 
    ASSERT_FLOAT_EQ(vel[2], (thrust-ptr_quadrotor->gravity_) * 0.01);  

    // check acc = a
    Eigen::Vector3d acc = Eigen::Vector3d::Random();
    ptr_quadrotor->GetAcc(acc);
    
    ASSERT_FLOAT_EQ(acc[0], 0); 
    ASSERT_FLOAT_EQ(acc[1], 0); 
    ASSERT_FLOAT_EQ(acc[2], (thrust-ptr_quadrotor->gravity_));  


    // check attitude = RotY(90d)
    Eigen::Quaterniond att = Eigen::Quaterniond::UnitRandom();
    ptr_quadrotor->GetAttitude(att);
    ASSERT_FLOAT_EQ(att.x(), 0); 
    ASSERT_FLOAT_EQ(att.y(), 0); 
    ASSERT_FLOAT_EQ(att.z(), 0.7071068);  
    ASSERT_FLOAT_EQ(att.w(), 0.7071068);  

    // check bodyrate =0
    Eigen::Vector3d bodyrate = Eigen::Vector3d::Random();
    ptr_quadrotor->GetBodyrate(bodyrate);
    ASSERT_EQ(bodyrate[0], 0); 
    ASSERT_EQ(bodyrate[1], 0); 
    ASSERT_EQ(bodyrate[2], 0);  

    // check bodyrate_acc =0
    Eigen::Vector3d bodyrate_acc = Eigen::Vector3d::Random();
    ptr_quadrotor->GetBodyRateAcc(bodyrate_acc);
    ASSERT_EQ(bodyrate_acc[0], 0); 
    ASSERT_EQ(bodyrate_acc[1], 0); 
    ASSERT_EQ(bodyrate_acc[2], 0);  

    //check time step = dt
    double step_current = 0;
    ptr_quadrotor->GetCurrentTimeStep(step_current);
    ASSERT_EQ(step_current, 0.01);  
}



int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}


double RandomGenerate(const double &minValue, const double &maxValue)
{

    std::random_device rd;  // Use hardware entropy to seed the random number generator
    std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()

    // Define the range for the random double value
    // double minValue = 5.0;
    // double maxValue = 10.0;

    // Define the distribution for double values within the specified range
    std::uniform_real_distribution<double> distribution(minValue, maxValue);

    // Generate a random double value within the specified range
    double randomValue = distribution(gen);

    return randomValue;
}