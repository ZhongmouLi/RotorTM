#include <iostream> 
#include <gtest/gtest.h>
#include <memory>
#include <cmath>
#include <cstdlib>
#include <random>
#include "rotor_tm_sim/lib_payload.hpp"
#include "rotor_tm_sim/lib_uav_cable.hpp"

double RandomGenerate(const double &minValue, const double &maxValue);

Eigen::Vector3d RandomUnitVector3d();

class rotorTMPayloadTest : public ::testing::Test
{
public:

rotorTMPayloadTest(){

    double mav_mass = 1;
    auto mav_inertia = Eigen::Matrix3d::Identity();

    MassProperty mav_mass_property = {mav_mass, mav_inertia};

    double dt=0.01;

    double cable_lenth = 1;

    Eigen::Vector3d attach_point_post_bf = {0,0,0};

    ptr_joint = std::make_shared<Joint>(attach_point_post_bf);

    ptr_uavcable = std::make_shared<UAVCable>(mav_mass_property, cable_lenth, ptr_joint, dt);


    double payload_mass = 1;
    auto payload_inertia = Eigen::Matrix3d::Identity();

    MassProperty payload_mass_property = {payload_mass, payload_inertia};


    ptr_payload = std::make_shared<Payload>(payload_mass_property, ptr_joint, dt);

}

~rotorTMPayloadTest(){
}

protected:
    std::shared_ptr<Payload> ptr_payload;
    std::shared_ptr<UAVCable> ptr_uavcable;
    std::shared_ptr<Joint> ptr_joint;
};


TEST_F(rotorTMPayloadTest, checkGTest){
    // test if gTest is well integrated
    ASSERT_TRUE(true);
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

Eigen::Vector3d RandomUnitVector3d()
{
    // Create a random number generator engine
    std::random_device rd;  // Use hardware entropy to seed the random number generator
    std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()

    // Define the distribution for each component of the vector
    std::uniform_real_distribution<double> distribution(-1.0, 1.0);

    // Generate random values for each component
    double x = distribution(gen);
    double y = distribution(gen);
    double z = distribution(gen);

    // Create an Eigen vector from the generated values
    Eigen::Vector3d randomVector(x, y, z);

    // Normalize the vector to make it a unit vector
    randomVector.normalize();

    // Print the generated unit vector
    return randomVector;
}