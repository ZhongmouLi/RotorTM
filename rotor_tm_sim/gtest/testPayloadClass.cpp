#include <iostream> 
#include <gtest/gtest.h>
#include <memory>
#include <cmath>
#include <cstdlib>
#include <random>
#include "rotor_tm_sim/lib_payload.hpp"


double RandomGenerate(const double &minValue, const double &maxValue);

Eigen::Vector3d RandomUnitVector3d();

class rotorTMPayloadTest : public ::testing::Test
{
public:

rotorTMPayloadTest(){
    Eigen::Matrix3d m_inertia = Eigen::Matrix3d::Identity(3,3);

    double mass =1;

    double dt = 0.01;

    std::vector<Eigen::Vector3d> v_attach_point_post{{1,0,0},{0,1,0}, {0,0,1}};

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

TEST_F(rotorTMPayloadTest, computeAttachPointPost){

    std::cout<< "fuck point 1"<< std::endl;
    // set initial post to be origin and initial attitude to be no rotation
    Eigen::Vector3d init_post = Eigen::Vector3d::Zero();

    std::cout<< "fuck point 2"<< std::endl;

    ptr_payload->SetInitialPost(init_post);

    std::cout<< "fuck point 3"<< std::endl;
    ptr_payload->SetInitialAttitude(0,0,0);

    // compute attach point posts and vels in world frame
    std::cout<< "fuck point 4"<< std::endl;
    ptr_payload->ComputeAttachPointsKinematics();

    // obtain posts
    Eigen::Vector3d attach_point_1;
    Eigen::Vector3d attach_point_2;
    Eigen::Vector3d attach_point_3;
    std::cout<< "fuck point 1"<< std::endl;
    ptr_payload->GetOneAttachPointPost(0, attach_point_1);
    std::cout<< "fuck point 2"<< std::endl;
    ptr_payload->GetOneAttachPointPost(1, attach_point_2);
    ptr_payload->GetOneAttachPointPost(2, attach_point_3);

    EXPECT_FLOAT_EQ(attach_point_1[0], init_post[0]+1); 
    EXPECT_FLOAT_EQ(attach_point_1[1], init_post[1]+0); 
    EXPECT_FLOAT_EQ(attach_point_1[2], init_post[2]+0);  

    EXPECT_FLOAT_EQ(attach_point_2[0], init_post[0]+0); 
    EXPECT_FLOAT_EQ(attach_point_2[1], init_post[1]+1); 
    EXPECT_FLOAT_EQ(attach_point_2[2], init_post[2]+0);   
    
    EXPECT_FLOAT_EQ(attach_point_3[0], init_post[0]+0); 
    EXPECT_FLOAT_EQ(attach_point_3[1], init_post[1]+0); 
    EXPECT_FLOAT_EQ(attach_point_3[2], init_post[2]+1);                       
}


// TEST_F(rotorTMPayloadTest, computeAttachPointPostX90){

//     // set initial post to be origin and initial attitude to be no rotation
//     Eigen::Vector3d init_post = Eigen::Vector3d::Zero();

//     ptr_payload->SetInitialPost(init_post);
//     ptr_payload->SetInitialAttitude(90.0/180*M_PI,0,0);

//     // compute attach point posts and vels in world frame
//     ptr_payload->ComputeAttachPointsKinematics();

//     // obtain posts
//     Eigen::Vector3d attach_point_1;
//     Eigen::Vector3d attach_point_2;
//     Eigen::Vector3d attach_point_3;
//     ptr_payload->GetOneAttachPointPost(0, attach_point_1);
//     ptr_payload->GetOneAttachPointPost(1, attach_point_2);
//     ptr_payload->GetOneAttachPointPost(2, attach_point_3);

//     EXPECT_NEAR(attach_point_1[0], init_post[0]+1, 1e-10); 
//     EXPECT_NEAR(attach_point_1[1], init_post[1]+0, 1e-10); 
//     EXPECT_NEAR(attach_point_1[2], init_post[2]+0, 1e-10);   

//     EXPECT_NEAR(attach_point_2[0], init_post[0]+0, 1e-10); 
//     EXPECT_NEAR(attach_point_2[1], init_post[1]+0, 1e-10); 
//     EXPECT_NEAR(attach_point_2[2], init_post[2]+1, 1e-10);  
    
//     EXPECT_NEAR(attach_point_3[0], init_post[0]+0, 1e-10); 
//     EXPECT_NEAR(attach_point_3[1], init_post[1]-1, 1e-10); 
//     EXPECT_NEAR(attach_point_3[2], init_post[2]+0, 1e-10);                        
// }


// TEST_F(rotorTMPayloadTest, computeAttachPointPostY90){

//     // set initial post to be origin and initial attitude to be no rotation
//     Eigen::Vector3d init_post = Eigen::Vector3d::Zero();

//     ptr_payload->SetInitialPost(init_post);
//     ptr_payload->SetInitialAttitude(0, 90.0/180*M_PI, 0);

//     // compute attach point posts and vels in world frame
//     ptr_payload->ComputeAttachPointsKinematics();

//     // obtain posts
//     Eigen::Vector3d attach_point_1;
//     Eigen::Vector3d attach_point_2;
//     Eigen::Vector3d attach_point_3;
//     ptr_payload->GetOneAttachPointPost(0, attach_point_1);
//     ptr_payload->GetOneAttachPointPost(1, attach_point_2);
//     ptr_payload->GetOneAttachPointPost(2, attach_point_3);

//     EXPECT_NEAR(attach_point_1[0], init_post[0]+0, 1e-10); 
//     EXPECT_NEAR(attach_point_1[1], init_post[1]+0, 1e-10); 
//     EXPECT_NEAR(attach_point_1[2], init_post[2]-1, 1e-10);   

//     EXPECT_NEAR(attach_point_2[0], init_post[0]+0, 1e-10); 
//     EXPECT_NEAR(attach_point_2[1], init_post[1]+1, 1e-10); 
//     EXPECT_NEAR(attach_point_2[2], init_post[2]+0, 1e-10);  
    
//     EXPECT_NEAR(attach_point_3[0], init_post[0]+1, 1e-10); 
//     EXPECT_NEAR(attach_point_3[1], init_post[1]+0, 1e-10); 
//     EXPECT_NEAR(attach_point_3[2], init_post[2]+0, 1e-10);                           
// }



// TEST_F(rotorTMPayloadTest, computeAttachPointPostZ90){

//     // set initial post to be origin and initial attitude to be no rotation
//     Eigen::Vector3d init_post = Eigen::Vector3d::Zero();

//     ptr_payload->SetInitialPost(init_post);
//     ptr_payload->SetInitialAttitude(0, 0, 90.0/180*M_PI);

//     // compute attach point posts and vels in world frame
//     ptr_payload->ComputeAttachPointsKinematics();

//     // obtain posts
//     Eigen::Vector3d attach_point_1;
//     Eigen::Vector3d attach_point_2;
//     Eigen::Vector3d attach_point_3;
//     ptr_payload->GetOneAttachPointPost(0, attach_point_1);
//     ptr_payload->GetOneAttachPointPost(1, attach_point_2);
//     ptr_payload->GetOneAttachPointPost(2, attach_point_3);

//     EXPECT_NEAR(attach_point_1[0], init_post[0]+0, 1e-10); 
//     EXPECT_NEAR(attach_point_1[1], init_post[1]+1, 1e-10); 
//     EXPECT_NEAR(attach_point_1[2], init_post[2]+0, 1e-10);   

//     EXPECT_NEAR(attach_point_2[0], init_post[0]-1, 1e-10); 
//     EXPECT_NEAR(attach_point_2[1], init_post[1]+0, 1e-10); 
//     EXPECT_NEAR(attach_point_2[2], init_post[2]+0, 1e-10);  
    
//     EXPECT_NEAR(attach_point_3[0], init_post[0]+0, 1e-10); 
//     EXPECT_NEAR(attach_point_3[1], init_post[1]+0, 1e-10); 
//     EXPECT_NEAR(attach_point_3[2], init_post[2]+1, 1e-10);                           
// }


// TEST_F(rotorTMPayloadTest, computeAttachPointPostX90TRanPost){

//     // set initial post to be origin and initial attitude to be no rotation
//     Eigen::Vector3d init_post = RandomUnitVector3d();

//     ptr_payload->SetInitialPost(init_post);
//     ptr_payload->SetInitialAttitude(90.0/180*M_PI,0,0);

//     // compute attach point posts and vels in world frame
//     ptr_payload->ComputeAttachPointsKinematics();

//     // obtain posts
//     Eigen::Vector3d attach_point_1;
//     Eigen::Vector3d attach_point_2;
//     Eigen::Vector3d attach_point_3;
//     ptr_payload->GetOneAttachPointPost(0, attach_point_1);
//     ptr_payload->GetOneAttachPointPost(1, attach_point_2);
//     ptr_payload->GetOneAttachPointPost(2, attach_point_3);

//     EXPECT_NEAR(attach_point_1[0], init_post[0]+1, 1e-10); 
//     EXPECT_NEAR(attach_point_1[1], init_post[1]+0, 1e-10); 
//     EXPECT_NEAR(attach_point_1[2], init_post[2]+0, 1e-10);   

//     EXPECT_NEAR(attach_point_2[0], init_post[0]+0, 1e-10); 
//     EXPECT_NEAR(attach_point_2[1], init_post[1]+0, 1e-10); 
//     EXPECT_NEAR(attach_point_2[2], init_post[2]+1, 1e-10);  
    
//     EXPECT_NEAR(attach_point_3[0], init_post[0]+0, 1e-10); 
//     EXPECT_NEAR(attach_point_3[1], init_post[1]-1, 1e-10); 
//     EXPECT_NEAR(attach_point_3[2], init_post[2]+0, 1e-10);                        
// }


// TEST_F(rotorTMPayloadTest, computeAttachPointPostY90RanPost){

//     // set initial post to be origin and initial attitude to be no rotation
//     Eigen::Vector3d init_post = RandomUnitVector3d();

//     ptr_payload->SetInitialPost(init_post);
//     ptr_payload->SetInitialAttitude(0, 90.0/180*M_PI, 0);

//     // compute attach point posts and vels in world frame
//     ptr_payload->ComputeAttachPointsKinematics();

//     // obtain posts
//     Eigen::Vector3d attach_point_1;
//     Eigen::Vector3d attach_point_2;
//     Eigen::Vector3d attach_point_3;
//     ptr_payload->GetOneAttachPointPost(0, attach_point_1);
//     ptr_payload->GetOneAttachPointPost(1, attach_point_2);
//     ptr_payload->GetOneAttachPointPost(2, attach_point_3);

//     EXPECT_NEAR(attach_point_1[0], init_post[0]+0, 1e-10); 
//     EXPECT_NEAR(attach_point_1[1], init_post[1]+0, 1e-10); 
//     EXPECT_NEAR(attach_point_1[2], init_post[2]-1, 1e-10);   

//     EXPECT_NEAR(attach_point_2[0], init_post[0]+0, 1e-10); 
//     EXPECT_NEAR(attach_point_2[1], init_post[1]+1, 1e-10); 
//     EXPECT_NEAR(attach_point_2[2], init_post[2]+0, 1e-10);  
    
//     EXPECT_NEAR(attach_point_3[0], init_post[0]+1, 1e-10); 
//     EXPECT_NEAR(attach_point_3[1], init_post[1]+0, 1e-10); 
//     EXPECT_NEAR(attach_point_3[2], init_post[2]+0, 1e-10);                           
// }



// TEST_F(rotorTMPayloadTest, computeAttachPointPostZ90RanPost){

//     // set initial post to be origin and initial attitude to be no rotation
//     Eigen::Vector3d init_post = RandomUnitVector3d();

//     ptr_payload->SetInitialPost(init_post);
//     ptr_payload->SetInitialAttitude(0, 0, 90.0/180*M_PI);

//     // compute attach point posts and vels in world frame
//     ptr_payload->ComputeAttachPointsKinematics();

//     // obtain posts
//     Eigen::Vector3d attach_point_1;
//     Eigen::Vector3d attach_point_2;
//     Eigen::Vector3d attach_point_3;
//     ptr_payload->GetOneAttachPointPost(0, attach_point_1);
//     ptr_payload->GetOneAttachPointPost(1, attach_point_2);
//     ptr_payload->GetOneAttachPointPost(2, attach_point_3);

//     EXPECT_NEAR(attach_point_1[0], init_post[0]+0, 1e-10); 
//     EXPECT_NEAR(attach_point_1[1], init_post[1]+1, 1e-10); 
//     EXPECT_NEAR(attach_point_1[2], init_post[2]+0, 1e-10);   

//     EXPECT_NEAR(attach_point_2[0], init_post[0]-1, 1e-10); 
//     EXPECT_NEAR(attach_point_2[1], init_post[1]+0, 1e-10); 
//     EXPECT_NEAR(attach_point_2[2], init_post[2]+0, 1e-10);  
    
//     EXPECT_NEAR(attach_point_3[0], init_post[0]+0, 1e-10); 
//     EXPECT_NEAR(attach_point_3[1], init_post[1]+0, 1e-10); 
//     EXPECT_NEAR(attach_point_3[2], init_post[2]+1, 1e-10);                           
// }



// TEST_F(rotorTMPayloadTest, computeAttachPointPostVels){

//     // set initial post to be origin and initial attitude to be no rotation
//     Eigen::Vector3d init_post = Eigen::Vector3d::Zero();
//     ptr_payload->SetInitialPost(init_post);

//     ptr_payload->SetInitialAttitude(0,0,0);

//     // sest vel of payload
//     Eigen::Vector3d init_vel = RandomUnitVector3d();
//     ptr_payload->SetVel(init_vel);

//     // compute attach point posts and vels in world frame
//     ptr_payload->ComputeAttachPointsKinematics();

//     // obtain posts
//     Eigen::Vector3d attach_point_1;
//     Eigen::Vector3d attach_point_2;
//     Eigen::Vector3d attach_point_3;
//     ptr_payload->GetOneAttachPointPost(0, attach_point_1);
//     ptr_payload->GetOneAttachPointPost(1, attach_point_2);
//     ptr_payload->GetOneAttachPointPost(2, attach_point_3);

//     EXPECT_FLOAT_EQ(attach_point_1[0], init_post[0]+1); 
//     EXPECT_FLOAT_EQ(attach_point_1[1], init_post[1]+0); 
//     EXPECT_FLOAT_EQ(attach_point_1[2], init_post[2]+0);  

//     EXPECT_FLOAT_EQ(attach_point_2[0], init_post[0]+0); 
//     EXPECT_FLOAT_EQ(attach_point_2[1], init_post[1]+1); 
//     EXPECT_FLOAT_EQ(attach_point_2[2], init_post[2]+0);   
    
//     EXPECT_FLOAT_EQ(attach_point_3[0], init_post[0]+0); 
//     EXPECT_FLOAT_EQ(attach_point_3[1], init_post[1]+0); 
//     EXPECT_FLOAT_EQ(attach_point_3[2], init_post[2]+1);        

//     // obtain vels
//     Eigen::Vector3d attach_point_vel_1;
//     Eigen::Vector3d attach_point_vel_2;
//     Eigen::Vector3d attach_point_vel_3;
//     ptr_payload->GetOneAttachPointVel(0, attach_point_vel_1);
//     ptr_payload->GetOneAttachPointVel(1, attach_point_vel_2);
//     ptr_payload->GetOneAttachPointVel(2, attach_point_vel_3);


//     EXPECT_FLOAT_EQ(attach_point_vel_1[0], init_vel[0]); 
//     EXPECT_FLOAT_EQ(attach_point_vel_1[1], init_vel[1]); 
//     EXPECT_FLOAT_EQ(attach_point_vel_1[2], init_vel[2]);   

//     EXPECT_FLOAT_EQ(attach_point_vel_2[0], init_vel[0]); 
//     EXPECT_FLOAT_EQ(attach_point_vel_2[1], init_vel[1]); 
//     EXPECT_FLOAT_EQ(attach_point_vel_2[2], init_vel[2]);  

//     EXPECT_FLOAT_EQ(attach_point_vel_3[0], init_vel[0]); 
//     EXPECT_FLOAT_EQ(attach_point_vel_3[1], init_vel[1]); 
//     EXPECT_FLOAT_EQ(attach_point_vel_3[2], init_vel[2]);          
                   
// }


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