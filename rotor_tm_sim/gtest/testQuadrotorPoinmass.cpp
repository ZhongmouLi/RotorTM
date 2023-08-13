#include <iostream> 
#include <gtest/gtest.h>
#include <memory>
#include <cmath>
#include <cstdlib>
#include <random>
#include "rotor_tm_sim/lib_quadrotor_pointmass.hpp"

class rotorTMQuadPmTest : public ::testing::Test
{
public:

rotorTMQuadPmTest(){
    Eigen::Matrix3d m_inertia = Eigen::Matrix3d::Identity(3,3);

    double UAV_mass =1;

    double payload_mass = 0.1;

    double dt = 0.01;

    double cable_length = 1;

    ptr_rotorTM = std::make_shared<rotorTMQuadrotorPointMass>(UAV_mass, m_inertia, payload_mass, cable_length, dt);

}

~rotorTMQuadPmTest(){
}

protected:
    std::shared_ptr<rotorTMQuadrotorPointMass> ptr_rotorTM;
};


TEST_F(rotorTMQuadPmTest, checkGTest){
    // test if gTest is well integrated
    ASSERT_TRUE(true);
}

TEST_F(rotorTMQuadPmTest, checkInstanceClass){
    // test if instance is created
    ASSERT_TRUE(ptr_rotorTM!=nullptr);
}

TEST_F(rotorTMQuadPmTest, checkInitPosition){
    // var to get drone init post
    Eigen::Vector3d drone_init_post = Eigen::Vector3d::Random();

    // payload init post
    const Eigen::Vector3d payload_init_post = Eigen::Vector3d::Random();

    ptr_rotorTM->setInitPost(payload_init_post);

    ptr_rotorTM->quadrotor->getPosition(drone_init_post);


    ASSERT_EQ(payload_init_post[0], drone_init_post[0]);
    ASSERT_EQ(payload_init_post[1], drone_init_post[1]);
    ASSERT_EQ(payload_init_post[2]+ 1, drone_init_post[2] );
}

TEST_F(rotorTMQuadPmTest, checkHovering){
    // var to get drone init post
    Eigen::Vector3d drone_post = Eigen::Vector3d::Random();
    Eigen::Vector3d payload_post = Eigen::Vector3d::Random();

    // payload init post
    const Eigen::Vector3d payload_init_post = Eigen::Vector3d::Random();

    ptr_rotorTM->setInitPost(payload_init_post);


    // input control signal
    ptr_rotorTM->inputMAVThrust(1.1*9.8);

    ptr_rotorTM->inputMAVTorque(Eigen::Vector3d::Zero());

        // do two steps integration
    const double dt = 0.01;
    for(double t=0.0 ; t<1000*dt ; t+= dt)
    {
            ptr_rotorTM->doOneStepint();
            // printf("current step is %.3f", t);
    }
    // ptr_rotorTM->doOneStepint();

    // obtain positions of drone and payload
    ptr_rotorTM->quadrotor->getPosition(drone_post);
    ptr_rotorTM->pm_payload->getPosition(payload_post); 

    ASSERT_EQ(payload_init_post[0], payload_post[0]);
    ASSERT_EQ(payload_init_post[1], payload_post[1]);
    ASSERT_EQ(payload_init_post[2], payload_post[2] );    

    ASSERT_EQ(payload_init_post[0], drone_post[0]);
    ASSERT_EQ(payload_init_post[1], drone_post[1]);
    ASSERT_EQ(payload_init_post[2]+ 1, drone_post[2] );
}


TEST_F(rotorTMQuadPmTest, applyThrustInZdirectionOneStep){


    // payload init post
    const Eigen::Vector3d payload_init_post = Eigen::Vector3d::Random();

    ptr_rotorTM->setInitPost(payload_init_post);


    // input control signal
    // Getting a random double value
    double lower_bound = 0.0;
    double upper_bound = 100.0;
 
    std::uniform_real_distribution<double> unif(lower_bound,
                                           upper_bound);
    std::default_random_engine re;
 
    const double thurst = unif(re);

    ptr_rotorTM->inputMAVThrust(thurst);

    ptr_rotorTM->inputMAVTorque(Eigen::Vector3d::Zero());

    // do one step integration
    const double dt = 0.01;
    for(double t=dt ; t<1000*dt ; t+= dt)
    {
            ptr_rotorTM->doOneStepint();
            // printf("current step is %.3f", t);
    }

    // check position = 0.5at^2
    // var to get drone init post
    Eigen::Vector3d drone_post = Eigen::Vector3d::Random();
    Eigen::Vector3d payload_post = Eigen::Vector3d::Random();    

    double acc_z = thurst/1.1 - 9.8;
    // obtain positions of drone and payload
    ptr_rotorTM->quadrotor->getPosition(drone_post);
    ptr_rotorTM->pm_payload->getPosition(payload_post); 

    // std::cout<< "drone post" <<drone_post.transpose()<<std::endl;
    // std::cout<< "payload post" <<payload_post.transpose()<<std::endl;

    ASSERT_EQ(payload_init_post[0], payload_post[0]);
    ASSERT_EQ(payload_init_post[1], payload_post[1]);
    ASSERT_FLOAT_EQ(payload_init_post[2] + 0.5 * acc_z * pow(0.01*1000,2), payload_post[2] );    

    ASSERT_EQ(payload_init_post[0], drone_post[0]);
    ASSERT_EQ(payload_init_post[1], drone_post[1]);
    ASSERT_FLOAT_EQ(payload_init_post[2]+ 1 + 0.5 * acc_z * pow(0.01*1000,2), drone_post[2] );

    // check vel = at
    // obtain vels of drone and payload
    // var to get drone init post
    Eigen::Vector3d drone_vel = Eigen::Vector3d::Random();
    Eigen::Vector3d payload_vel = Eigen::Vector3d::Random();

    ptr_rotorTM->quadrotor->getVel(drone_vel);
    ptr_rotorTM->pm_payload->getVel(payload_vel); 

    // std::cout<< "drone vel" <<drone_vel.transpose()<<std::endl;
    // std::cout<< "payload vel" <<payload_vel.transpose()<<std::endl;

    ASSERT_EQ(drone_vel[0],0) ;
    ASSERT_EQ(drone_vel[1], 0);
    ASSERT_FLOAT_EQ(drone_vel[2], acc_z * 0.01 * 1000);    

    ASSERT_EQ(payload_vel[0], 0);
    ASSERT_EQ(payload_vel[1], 0);
    ASSERT_FLOAT_EQ(payload_vel[2], acc_z * 0.01 * 1000);    
}


TEST_F(rotorTMQuadPmTest, applyThrustInZdirectionOneStepThousandTimes){


    // payload init post
    const Eigen::Vector3d payload_init_post = Eigen::Vector3d::Random();

    ptr_rotorTM->setInitPost(payload_init_post);


    // input control signal
    // Getting a random double value
    double lower_bound = 0.0;
    double upper_bound = 100.0;
 
    std::uniform_real_distribution<double> unif(lower_bound,
                                           upper_bound);
    std::default_random_engine re;
 
    const double thurst = unif(re);

    ptr_rotorTM->inputMAVThrust(thurst);

    ptr_rotorTM->inputMAVTorque(Eigen::Vector3d::Zero());

    // do one step integration
    ptr_rotorTM->doOneStepint();

    // check position = 0.5at^2
    // var to get drone init post
    Eigen::Vector3d drone_post = Eigen::Vector3d::Random();
    Eigen::Vector3d payload_post = Eigen::Vector3d::Random();    

    double acc_z = thurst/1.1 - 9.8;
    // obtain positions of drone and payload
    ptr_rotorTM->quadrotor->getPosition(drone_post);
    ptr_rotorTM->pm_payload->getPosition(payload_post); 

    ASSERT_EQ(payload_init_post[0], payload_post[0]);
    ASSERT_EQ(payload_init_post[1], payload_post[1]);
    ASSERT_FLOAT_EQ(payload_init_post[2] +  0.5 * acc_z * pow(0.01,2), payload_post[2] );    

    ASSERT_EQ(payload_init_post[0], drone_post[0]);
    ASSERT_EQ(payload_init_post[1], drone_post[1]);
    ASSERT_FLOAT_EQ(payload_init_post[2]+ 1 + 0.5 * acc_z * pow(0.01,2), drone_post[2] );

    // check vel = at
    // obtain vels of drone and payload
    // var to get drone init post
    Eigen::Vector3d drone_vel = Eigen::Vector3d::Random();
    Eigen::Vector3d payload_vel = Eigen::Vector3d::Random();

    ptr_rotorTM->quadrotor->getVel(drone_vel);
    ptr_rotorTM->pm_payload->getVel(payload_vel); 

    // std::cout<< "drone vel" <<drone_vel.transpose()<<std::endl;
    // std::cout<< "payload vel" <<payload_vel.transpose()<<std::endl;

    ASSERT_EQ(drone_vel[0],0) ;
    ASSERT_EQ(drone_vel[1], 0);
    ASSERT_FLOAT_EQ(drone_vel[2], acc_z * 0.01);    

    ASSERT_EQ(payload_vel[0], 0);
    ASSERT_EQ(payload_vel[1], 0);
    ASSERT_FLOAT_EQ(payload_vel[2], acc_z * 0.01);    
}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}