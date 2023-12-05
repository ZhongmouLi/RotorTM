#include <iostream> 
#include <gtest/gtest.h>
#include <memory>
#include <cmath>
#include <cstdlib>
#include "rotor_tm_sim/lib_payload.hpp"


class rotorTMPayloadTest : public ::testing::Test
{
public:

rotorTMPayloadTest(){
    Eigen::Matrix3d m_inertia = Eigen::Matrix3d::Identity(3,3);

    double mass =1;

    double dt = 0.01;

    std::vector<Eigen::Vector3d> v_attach_point_post{{1,2,3},{4,5,6}};

    ptr_payload = std::make_shared<Payload>(v_attach_point_post, mass, m_inertia, dt);
}

~rotorTMPayloadTest(){
}

protected:
    std::shared_ptr<Payload> ptr_payload;
};


TEST_F(rotorTMPayloadTest, checkGTest){
    // test if gTest is well integrated
    ASSERT_TRUE(true);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
