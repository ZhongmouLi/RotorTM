#include <iostream> 
#include <gtest/gtest.h>
#include <memory>
#include <cmath>
#include <cstdlib>
#include <random>
#include "rotor_tm_sim/lib_uav_cable.hpp"
#include "rotor_tm_sim/lib_joint.hpp"

double RandomGenerate(const double &minValue, const double &maxValue);

Eigen::Vector3d RandomUnitVector3d();


class rotorTMUAVCableTestTwo : public ::testing::Test
{
public:

rotorTMUAVCableTestTwo(){
    // double mass =1;
    // Eigen::Matrix3d m_inertia = Eigen::Matrix3d::Identity(3,3);

    // MassProperty mav_mass_property(1, Eigen::Matrix3d::Identity(3,3));
    MassProperty mav_mass_property = {0.25, Eigen::Matrix3d::Identity()};
    double cable_length =0.5;
    double step_size = 0.01;

    ptr_joint = std::make_shared<Joint>(Eigen::Vector3d(0.3048, -0.3048, 0.2286));

    ptr_UAVCable = std::make_shared<UAVCable>(mav_mass_property, cable_length, ptr_joint, step_size);
}

~rotorTMUAVCableTestTwo(){
}

protected:
    std::shared_ptr<UAVCable> ptr_UAVCable;
    std::shared_ptr<Joint> ptr_joint;
};

// test if gTest is well integrated
TEST_F(rotorTMUAVCableTestTwo, checkGTest){
    ASSERT_TRUE(true);
}

// test if instance is created
TEST_F(rotorTMUAVCableTestTwo, checkInstanceClass){
    ASSERT_TRUE(ptr_UAVCable!=nullptr);
}


TEST_F(rotorTMUAVCableTestTwo, checkAttachWrenchPythonData){
        // 1. set posts of mav and payload to make collision
        const Eigen::Vector3d mav_post = {0.0252059738, -0.291245909,   0.73277827};

        const Eigen::Vector3d mav_vel = {0.0491868057, -0.185771834, -0.0756362028};

        const Eigen::Quaterniond mav_att = {0.998905756,   -0.0250219106,   0.0394938854,   0.00116508073};

        
        ptr_UAVCable->mav_.SetInitialPost(mav_post);
        ptr_UAVCable->mav_.SetLinearVel(mav_vel);
        ptr_UAVCable->mav_.SetAttitude(mav_att);
        

        
        // 2. set post and vel of payload in vertical direction
        const Eigen::Vector3d attachpoint_vel = {0.1102021080856959,  -0.5136190059861094,  -0.06803463173326546};
        const Eigen::Vector3d attachpoint_post = {-0.05065964393581873, -0.31721938106230557,  0.2392774917034387};

        ptr_joint->SetInitPost(attachpoint_post);
        ptr_joint->SetLinearVel(attachpoint_vel); 

        // 4. set control input for quadrotor
        const double mav_thrust = 3.08103444;
        // std::cout<<"[----------] mav_thrust is " << mav_thrust<<std::endl;
        
        ptr_UAVCable->InputControllerInput(mav_thrust,Eigen::Vector3d::Zero());

        Eigen::Vector3d attach_point_acc = {1.836306748058199, 0.36052611117358,  9.563723859150802};
        ptr_joint->SetLinearAcc(attach_point_acc);


        // compute interaction wrenches
        const Eigen::Quaterniond payload_attitude = {9.99669715e-01,  1.01764791e-02, -2.32300150e-02, -4.15289677e-03};
        const Eigen::Vector3d payload_bodyrate = {1.88742302e-01,   3.53695967e-02, -1.79684901e-02};
   
        ptr_UAVCable->ComputeInteractionWrenches(payload_attitude, payload_bodyrate);

        // joint post
        // dbg(ptr_UAVCable->ptr_joint()->pose().post);
        EXPECT_DOUBLE_EQ(ptr_UAVCable->ptr_joint()->pose().post[0], -0.05065964393581873); 
        EXPECT_DOUBLE_EQ(ptr_UAVCable->ptr_joint()->pose().post[1], -0.31721938106230557); 
        EXPECT_DOUBLE_EQ(ptr_UAVCable->ptr_joint()->pose().post[2], 0.2392774917034387); 

        // joint vel
        // dbg(ptr_UAVCable->ptr_joint()->vels().linear_vel);
        EXPECT_DOUBLE_EQ(ptr_UAVCable->ptr_joint()->vels().linear_vel[0], 0.1102021080856959); 
        EXPECT_DOUBLE_EQ(ptr_UAVCable->ptr_joint()->vels().linear_vel[1], -0.5136190059861094); 
        EXPECT_DOUBLE_EQ(ptr_UAVCable->ptr_joint()->vels().linear_vel[2], -0.0680346317332654); 


        // mav post
        // dbg(ptr_UAVCable->mav_.pose().post);
        EXPECT_DOUBLE_EQ(ptr_UAVCable->mav_.pose().post[0], 0.0252059738);
        EXPECT_DOUBLE_EQ(ptr_UAVCable->mav_.pose().post[1], -0.291245909);
        EXPECT_DOUBLE_EQ(ptr_UAVCable->mav_.pose().post[2], 0.73277827);

        // mave vel
        // dbg(ptr_UAVCable->mav_.vels().linear_vel);
        EXPECT_DOUBLE_EQ(ptr_UAVCable->mav_.vels().linear_vel[0], 0.0491868057);
        EXPECT_DOUBLE_EQ(ptr_UAVCable->mav_.vels().linear_vel[1], -0.185771834);
        EXPECT_DOUBLE_EQ(ptr_UAVCable->mav_.vels().linear_vel[2], -0.0756362028);
        
        // cable direction
        // dbg(ptr_UAVCable->cable_.direction());   
        EXPECT_DOUBLE_EQ(ptr_UAVCable->cable_.direction()[0], -0.15173935939417824);
        EXPECT_DOUBLE_EQ(ptr_UAVCable->cable_.direction()[1], -0.051949725443493695);
        EXPECT_DOUBLE_EQ(ptr_UAVCable->cable_.direction()[2], -0.9870544021668664);
         
         // cable bodyrate
        // dbg(ptr_UAVCable->cable_.bodyrate());
        ASSERT_NEAR(ptr_UAVCable->cable_.bodyrate()[0], -0.6479957877534063, 1e-19);
        ASSERT_NEAR(ptr_UAVCable->cable_.bodyrate()[1], -0.11814393059057604, 1e-19);
        ASSERT_NEAR(ptr_UAVCable->cable_.bodyrate()[2], 0.10583409612630774, 1e-19);

        // attach point force


        // cable tension
        // dbg(ptr_UAVCable->cable_.tension());
        ASSERT_NEAR(ptr_UAVCable->cable_.tension(), 0.6940406304117104, 1e-19);

        // cable tension force
        // dbg(ptr_UAVCable->cable_.tensionForce());
        ASSERT_NEAR(ptr_UAVCable->cable_.tensionForce()[0], -0.10531328065220455, 1e-19);
        ASSERT_NEAR(ptr_UAVCable->cable_.tensionForce()[1], -0.036055220196517634, 1e-19);
        ASSERT_NEAR(ptr_UAVCable->cable_.tensionForce()[2], -0.6850558595305458, 1e-19);
        
        // net force applied to attach point
        // dbg(ptr_UAVCable->mav_thrust_force_along_cable());
        ASSERT_NEAR(ptr_UAVCable->mav_thrust_force_along_cable()[0], 0.46625403655131975 , 1e-15);
        ASSERT_NEAR(ptr_UAVCable->mav_thrust_force_along_cable()[1], 0.15962746437356481, 1e-15);
        ASSERT_NEAR(ptr_UAVCable->mav_thrust_force_along_cable()[2], 3.0329513788873177, 1e-15);
        

        // dbg(ptr_UAVCable->attach_point_wrench().force);

        // dbg(Utils::FortmatEigen4DBG(ptr_UAVCable->attach_point_wrench().torque, 15));

        ASSERT_NEAR(ptr_UAVCable->attach_point_wrench().force[0], 0.4750282172168813, 1e-10);
        ASSERT_NEAR(ptr_UAVCable->attach_point_wrench().force[1], 0.16263140664923664, 1e-10);
        ASSERT_NEAR(ptr_UAVCable->attach_point_wrench().force[2], 3.0900268383193827, 1e-10);
        // 4.74695626866511177777e-01 1.62517540494431039777e-01 3.08786336029521235602e+00

        // attach point torque
        // dbg(Utils::FortmatEigen4DBG(ptr_UAVCable->attach_point_wrench().torque, 15));
        ASSERT_NEAR(ptr_UAVCable->attach_point_wrench().torque[0], -0.985420032857682, 1e-9);
        ASSERT_NEAR(ptr_UAVCable->attach_point_wrench().torque[1], -0.7920005701511341, 1e-9);
        ASSERT_NEAR(ptr_UAVCable->attach_point_wrench().torque[2],  0.2578926169420638, 1e-8);            
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