#include <iostream> 
#include <gtest/gtest.h>
#include <memory>
#include <cmath>
#include <cstdlib>
#include <random>
#include "rotor_tm_sim/lib_cable.hpp"

double RandomGenerate(const double &minValue, const double &maxValue);

Eigen::Vector3d RandomUnitVector3d();

class rotorTMCableTest : public ::testing::Test
{
public:

rotorTMCableTest(){

    double length =1;

    ptr_cable = std::make_shared<Cable>(length);
}

~rotorTMCableTest(){
}

protected:
    std::shared_ptr<Cable> ptr_cable;
};

// test if gTest is well integrated
TEST_F(rotorTMCableTest, checkGTest){
    ASSERT_TRUE(true);
}

// test if instance is created
TEST_F(rotorTMCableTest, checkInstanceClass){
    ASSERT_TRUE(ptr_cable!=nullptr);
}



// input Z positions of attach points for a cable
TEST_F(rotorTMCableTest, calZDireAttachPoints){
    
    // input two attach points (one is robot)
    Eigen::Vector3d attachpoint_post{0,0,0};

    double length =0;
    ptr_cable->GetCableLength(length);
    
    auto robot_post_z = RandomGenerate(-length, length);

    Eigen::Vector3d robot_post{0,0,robot_post_z};

    std::cout<<robot_post.transpose()<<std::endl;

    // compute cable direction and result is stored in xi_
    ptr_cable->ComputeCableDirection(attachpoint_post,robot_post);

    Eigen::Vector3d cable_direction;

    ptr_cable->GetCableDirection(cable_direction);

    ASSERT_FLOAT_EQ(cable_direction[0], -robot_post.normalized()[0]); 
    ASSERT_FLOAT_EQ(cable_direction[1], -robot_post.normalized()[1]); 
    ASSERT_FLOAT_EQ(cable_direction[2], -robot_post.normalized()[2]);  
}



// input X direction positions of attach points for a cable
TEST_F(rotorTMCableTest, calXDireAttachPoints){
    
    // input two attach points (one is robot)
    Eigen::Vector3d attachpoint_post{0,0,0};

    double length =0;
    ptr_cable->GetCableLength(length);
    
    auto robot_post_x = RandomGenerate(-length, length);
    
    Eigen::Vector3d robot_post{robot_post_x,0,0};

    // compute cable direction and result is stored in xi_
    ptr_cable->ComputeCableDirection(attachpoint_post,robot_post);

    Eigen::Vector3d cable_direction;

    ptr_cable->GetCableDirection(cable_direction);

    ASSERT_FLOAT_EQ(cable_direction[0], -robot_post.normalized()[0]); 
    ASSERT_FLOAT_EQ(cable_direction[1], -robot_post.normalized()[1]); 
    ASSERT_FLOAT_EQ(cable_direction[2], -robot_post.normalized()[2]);  
}


// input Y direction positions of attach points for a cable
TEST_F(rotorTMCableTest, calYDireAttachPoints){
    
    // input two attach points (one is robot)
    Eigen::Vector3d attachpoint_post{0,0,0};

    double length =0;
    ptr_cable->GetCableLength(length);
    
    auto robot_post_y = RandomGenerate(-length, length);
    
    Eigen::Vector3d robot_post{0,robot_post_y,0};

    // compute cable direction and result is stored in xi_
    ptr_cable->ComputeCableDirection(attachpoint_post,robot_post);

    Eigen::Vector3d cable_direction;

    ptr_cable->GetCableDirection(cable_direction);


    ASSERT_FLOAT_EQ(cable_direction[0], -robot_post.normalized()[0]); 
    ASSERT_FLOAT_EQ(cable_direction[1], -robot_post.normalized()[1]); 
    ASSERT_FLOAT_EQ(cable_direction[2], -robot_post.normalized()[2]);  
}


// input radom robot position (unit vector)
TEST_F(rotorTMCableTest, calRandomAttachPoints){
    
    // input two attach points (one is robot)
    Eigen::Vector3d attachpoint_post{0,0,0};

    Eigen::Vector3d robot_post = RandomUnitVector3d();

    // compute cable direction and result is stored in xi_
    ptr_cable->ComputeCableDirection(attachpoint_post,robot_post);

    Eigen::Vector3d cable_direction;

    ptr_cable->GetCableDirection(cable_direction);


    ASSERT_FLOAT_EQ(cable_direction[0], -robot_post.normalized()[0]); 
    ASSERT_FLOAT_EQ(cable_direction[1], -robot_post.normalized()[1]); 
    ASSERT_FLOAT_EQ(cable_direction[2], -robot_post.normalized()[2]);  
}



// check vertical taut
TEST_F(rotorTMCableTest, calTautVeriticalNoMotion){

    double length =0;
    ptr_cable->GetCableLength(length);

    // input two attach points (one is robot)
    Eigen::Vector3d attachpoint_post{0,0,0};

    Eigen::Vector3d robot_post = attachpoint_post + Eigen::Vector3d::UnitZ()*(length+2e-3);

    // 
    ptr_cable->ComputeCableDirection(attachpoint_post,robot_post);
    ptr_cable->CheckTaut(attachpoint_post, robot_post, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    bool cable_taut;

    ptr_cable->GetCableTautStatus(cable_taut);

    ASSERT_TRUE(!cable_taut);
 
}


// check vertical taut
TEST_F(rotorTMCableTest, calTautVeriticalRobotMotionClose){

    double length =0;
    ptr_cable->GetCableLength(length);

    // input two attach points (one is robot)
    Eigen::Vector3d attachpoint_post{0,0,0};

    Eigen::Vector3d robot_post = attachpoint_post + Eigen::Vector3d::UnitZ()*(length+2e-3);
    // robot and pose vel
    Eigen::Vector3d robot_vel{0,0,-1};

    // 
    ptr_cable->ComputeCableDirection(attachpoint_post,robot_post);
    ptr_cable->CheckTaut(attachpoint_post, robot_post, Eigen::Vector3d::Zero(), robot_vel);

    bool cable_taut;

    ptr_cable->GetCableTautStatus(cable_taut);

    ASSERT_TRUE(!cable_taut);
 
}

// check vertical taut
TEST_F(rotorTMCableTest, calTautVeriticalRobotMotionRandClose){

    double length =0;
    ptr_cable->GetCableLength(length);

    // input two attach points (one is robot)
    Eigen::Vector3d attachpoint_post{0,0,0};

    Eigen::Vector3d robot_post = attachpoint_post + Eigen::Vector3d::UnitZ()*(length+2e-3);

    // robot and pose vel
    double robot_z_vel = RandomGenerate(-5, -0.01);
    Eigen::Vector3d robot_vel{0,0,robot_z_vel};

    // 
    ptr_cable->ComputeCableDirection(attachpoint_post,robot_post);
    ptr_cable->CheckTaut(attachpoint_post, robot_post, Eigen::Vector3d::Zero(), robot_vel);

    bool cable_taut;

    ptr_cable->GetCableTautStatus(cable_taut);

    ASSERT_TRUE(!cable_taut);
 
}


// check vertical taut
TEST_F(rotorTMCableTest, calTautVeriticalRobotMotionRandLeave){

    double length =0;
    ptr_cable->GetCableLength(length);

    // input two attach points (one is robot)
    Eigen::Vector3d attachpoint_post{0,0,0};

    Eigen::Vector3d robot_post = attachpoint_post + Eigen::Vector3d::UnitZ()*(length+2e-3);
    // robot and pose vel
    double robot_z_vel = RandomGenerate(0.05, 5);
    Eigen::Vector3d robot_vel{0,0,robot_z_vel};

    // 
    ptr_cable->ComputeCableDirection(attachpoint_post,robot_post);
    ptr_cable->CheckTaut(attachpoint_post, robot_post, Eigen::Vector3d::Zero(), robot_vel);

    bool cable_taut;

    ptr_cable->GetCableTautStatus(cable_taut);

    ASSERT_TRUE(cable_taut);
 
}

// check vertical taut when they are relative static
TEST_F(rotorTMCableTest, calTautVeriticalRobotAttachPointMotionStatic){

    double length =0;
    ptr_cable->GetCableLength(length);

    // input two attach points (one is robot)
    Eigen::Vector3d attachpoint_post{0,0,0};

    Eigen::Vector3d robot_post = attachpoint_post + Eigen::Vector3d::UnitZ()*(length+2e-3);

    // robot vel
    double robot_z_vel = RandomGenerate(0.05, 5);
    Eigen::Vector3d robot_vel{0,0,robot_z_vel};

    // attach point vel
    double attachpoint_z_vel = robot_z_vel;
    Eigen::Vector3d attachpoint_vel{0,0,attachpoint_z_vel};    

    // 
    ptr_cable->ComputeCableDirection(attachpoint_post,robot_post);
    ptr_cable->CheckTaut(attachpoint_post, robot_post, attachpoint_vel, robot_vel);

    bool cable_taut;

    ptr_cable->GetCableTautStatus(cable_taut);

    ASSERT_TRUE(!cable_taut);
 
}



// check vertical taut when they are further
TEST_F(rotorTMCableTest, calTautVeriticalRobotAttachPointMotionFar){

    double length =0;
    ptr_cable->GetCableLength(length);

    // input two attach points (one is robot)
    Eigen::Vector3d attachpoint_post{0,0,0};

    Eigen::Vector3d robot_post = attachpoint_post + Eigen::Vector3d::UnitZ()*(length+2e-3);

    // robot vel
    double robot_z_vel = RandomGenerate(0.05, 5);
    Eigen::Vector3d robot_vel{0,0,robot_z_vel};

    // attach point vel
    double attachpoint_z_vel = robot_z_vel - RandomGenerate(0.05, 5);;
    Eigen::Vector3d attachpoint_vel{0,0,attachpoint_z_vel};    

    // 
    ptr_cable->ComputeCableDirection(attachpoint_post,robot_post);
    ptr_cable->CheckTaut(attachpoint_post, robot_post, attachpoint_vel, robot_vel);

    bool cable_taut;

    ptr_cable->GetCableTautStatus(cable_taut);

    ASSERT_TRUE(cable_taut);
 
}

// check vertical taut when they are closer
TEST_F(rotorTMCableTest, calTautVeriticalRobotAttachPointMotionClose){

    double length =0;
    ptr_cable->GetCableLength(length);

    // input two attach points (one is robot)
    Eigen::Vector3d attachpoint_post{0,0,0};

    Eigen::Vector3d robot_post = attachpoint_post + Eigen::Vector3d::UnitZ()*(length+2e-3);

    // robot vel
    double robot_z_vel = RandomGenerate(0.05, 5);
    Eigen::Vector3d robot_vel{0,0,robot_z_vel};

    // attach point vel
    double attachpoint_z_vel = robot_z_vel + RandomGenerate(0.05, 5);;
    Eigen::Vector3d attachpoint_vel{0,0,attachpoint_z_vel};    

    // 
    ptr_cable->ComputeCableDirection(attachpoint_post,robot_post);
    ptr_cable->CheckTaut(attachpoint_post, robot_post, attachpoint_vel, robot_vel);

    bool cable_taut;

    ptr_cable->GetCableTautStatus(cable_taut);

    ASSERT_TRUE(!cable_taut);
 
}



// check tension force for static equilibrium in vertical direction
TEST_F(rotorTMCableTest, calTensionForceStatic){

    double length =0;
    ptr_cable->GetCableLength(length);

    // input two attach points (one is robot)
    Eigen::Vector3d attachpoint_post{0,0,0};

    Eigen::Vector3d robot_post = attachpoint_post + Eigen::Vector3d::UnitZ()*(length+2e-3);

    // robot vel
    double robot_z_vel = RandomGenerate(0.05, 5);
    Eigen::Vector3d robot_vel{0,0,robot_z_vel};

    // attach point vel
    double attachpoint_z_vel = robot_z_vel - RandomGenerate(0.05, 0.1);;
    Eigen::Vector3d attachpoint_vel{0,0,attachpoint_z_vel};    

    // 
    ptr_cable->ComputeCableDirection(attachpoint_post,robot_post);
    ptr_cable->CheckTaut(attachpoint_post, robot_post, attachpoint_vel, robot_vel);

    bool cable_taut;

    ptr_cable->GetCableTautStatus(cable_taut);

    double robot_mass = 1;
    Eigen::Vector3d robot_force{0,0,9.8*robot_mass};

    ptr_cable->ComputeCableTensionForce(robot_mass, robot_force, Eigen::Vector3d::Zero());

    Eigen::Vector3d tension_force;
    ptr_cable->GetCableTensionForce(tension_force);

    ASSERT_TRUE(cable_taut);

    ASSERT_FLOAT_EQ(tension_force[0], robot_force[0]); 
    ASSERT_FLOAT_EQ(tension_force[1], robot_force[1]); 
    ASSERT_FLOAT_EQ(tension_force[2], -robot_force[2]);      
}


// check tension force to accelerate in vertical direction
TEST_F(rotorTMCableTest, calTensionForceVerticalAcclerate){

    double length =0;
    ptr_cable->GetCableLength(length);

    // input two attach points (one is robot)
    Eigen::Vector3d attachpoint_post{0,0,0};

    Eigen::Vector3d robot_post = attachpoint_post + Eigen::Vector3d::UnitZ()*(length+2e-3);

    // robot vel
    double robot_z_vel = RandomGenerate(0.05, 5);
    Eigen::Vector3d robot_vel{0,0,robot_z_vel};

    // attach point vel
    double attachpoint_z_vel = robot_z_vel - RandomGenerate(0.05, 0.1);;
    Eigen::Vector3d attachpoint_vel{0,0,attachpoint_z_vel};    

    // 
    ptr_cable->ComputeCableDirection(attachpoint_post,robot_post);
    ptr_cable->CheckTaut(attachpoint_post, robot_post, attachpoint_vel, robot_vel);

    bool cable_taut;

    ptr_cable->GetCableTautStatus(cable_taut);

    double robot_mass = 1;
    Eigen::Vector3d robot_force{0,0,2*9.8*robot_mass};

    Eigen::Vector3d attach_point_acc{0,0,3.266666667 + 9.8};

    ptr_cable->ComputeCableTensionForce(robot_mass, robot_force, attach_point_acc);

    Eigen::Vector3d tension_force;
    ptr_cable->GetCableTensionForce(tension_force);

    ASSERT_TRUE(cable_taut);

    ASSERT_FLOAT_EQ(tension_force[0], 0); 
    ASSERT_FLOAT_EQ(tension_force[1], 0); 
    ASSERT_FLOAT_EQ(tension_force[2], -6.533333333);      
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