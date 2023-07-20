#include <iostream> 
#include <gtest/gtest.h>
#include <memory>
#include <cmath>
#include <cstdlib>
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


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}