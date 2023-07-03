#include <iostream> 
#include <gtest/gtest.h>
#include <memory>
#include <cmath>
#include <cstdlib>
#include "rotor_tm_sim/lib_quadrotor_pointmass.hpp"

class rotorTMTest4QuadPm : public ::testing::Test
{
public:

rotorTMTest4QuadPm(){
    Eigen::Matrix3d m_inertia = Eigen::Matrix3d::Identity(3,3);

    double UAV_mass =1;

    double payload_mass = 0.1;

    double dt = 0.01;

    ptr_rotorTM = std::make_shared<rotorTMQuadrotorPointMass>(UAV_mass, m_inertia, payload_mass, dt);
}

~rotorTMTest4QuadPm(){
}

protected:
    std::shared_ptr<rotorTMQuadrotorPointMass> ptr_rotorTM;
};


TEST_F(rotorTMTest4QuadPm, checkGTest){
    // test if gTest is well integrated
    ASSERT_TRUE(true);
}

TEST_F(rotorTMTest4QuadPm, checkInstanceClass){
    // test if instance is created
    ASSERT_TRUE(ptr_rotorTM!=nullptr);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}