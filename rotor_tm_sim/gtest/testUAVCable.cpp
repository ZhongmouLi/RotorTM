#include <iostream> 
#include <gtest/gtest.h>
#include <memory>
#include <cmath>
#include <cstdlib>
#include <random>
#include "rotor_tm_sim/lib_uav_cable.hpp"

double RandomGenerate(const double &minValue, const double &maxValue);

Eigen::Vector3d RandomUnitVector3d();


class rotorTMUAVCableTest : public ::testing::Test
{
public:

rotorTMUAVCableTest(){
    double mass =1;
    Eigen::Matrix3d m_inertia = Eigen::Matrix3d::Identity(3,3);
    double cable_length =1;
    double step_size = 0.01;

    ptr_UAVCable = std::make_shared<UAVCable>(mass, m_inertia, cable_length, step_size);
}

~rotorTMUAVCableTest(){
}

protected:
    std::shared_ptr<UAVCable> ptr_UAVCable;
};

// test if gTest is well integrated
TEST_F(rotorTMUAVCableTest, checkGTest){
    ASSERT_TRUE(true);
}

// test if instance is created
TEST_F(rotorTMUAVCableTest, checkInstanceClass){
    ASSERT_TRUE(ptr_UAVCable!=nullptr);
}



/*****************************Collision test****************************/

// test vertical init post
TEST_F(rotorTMUAVCableTest, checkSetInitPost){

    // 1. set post and vel of mav in vertical direction
    double cable_length;
    ptr_UAVCable->cable_.GetCableLength(cable_length);

    const Eigen::Vector3d mav_post{0,0,cable_length};

    ptr_UAVCable->SetMAVInitPost(mav_post);

    // 2. set post and vel of payload in vertical direction
    Eigen::Vector3d attachpoint_post{0,0,0};

    // 3. check collision
    ptr_UAVCable->mav_.GetPosition(attachpoint_post);

    // 4.
    ASSERT_FLOAT_EQ(attachpoint_post[0], 0); 
    ASSERT_FLOAT_EQ(attachpoint_post[1], 0); 
    ASSERT_FLOAT_EQ(attachpoint_post[2], cable_length);  
}

// test vertical init post
TEST_F(rotorTMUAVCableTest, checkSetInitPostWithPayload){

    // 1. set post and vel of mav in vertical direction
    double cable_length;
    ptr_UAVCable->cable_.GetCableLength(cable_length);

    ptr_UAVCable->SetMAVInitPostCableTautWithAttachPointPost(Eigen::Vector3d::Zero());

    // 2. set post and vel of payload in vertical direction
    Eigen::Vector3d attachpoint_post{0,0,0};

    // 3. check collision
    ptr_UAVCable->mav_.GetPosition(attachpoint_post);

    // 4.
    ASSERT_FLOAT_EQ(attachpoint_post[0], 0); 
    ASSERT_FLOAT_EQ(attachpoint_post[1], 0); 
    ASSERT_FLOAT_EQ(attachpoint_post[2], cable_length);  
}

// test vertical static condition
TEST_F(rotorTMUAVCableTest, checkVerticalStatic){

    // 1. set post and vel of mav in vertical direction
    double cable_length;
    ptr_UAVCable->cable_.GetCableLength(cable_length);

    const Eigen::Vector3d mav_post{0,0,cable_length};
    const Eigen::Vector3d mav_vel{0,0,0};

    ptr_UAVCable->SetMAVInitPost(mav_post);
    ptr_UAVCable->mav_.SetVel(mav_vel);

    // 2. set post and vel of payload in vertical direction
    const Eigen::Vector3d attachpoint_post{0,0,0};
    const Eigen::Vector3d attachpoint_vel{0,0,0};

    // 3. check collision
    bool isCollided = ptr_UAVCable->IsCollided(attachpoint_post, attachpoint_vel);

    // 4.
    ASSERT_TRUE(!isCollided);
}

// test vertical collision
TEST_F(rotorTMUAVCableTest, checkVerticalCollision){

    // 1. set post and vel of mav in vertical direction
    double cable_length;
    ptr_UAVCable->cable_.GetCableLength(cable_length);

    const Eigen::Vector3d mav_post{0,0,cable_length+RandomGenerate(0,0.1)};
    const Eigen::Vector3d mav_vel{0,0,RandomGenerate(0,0.1)};

    ptr_UAVCable->SetMAVInitPost(mav_post);
    ptr_UAVCable->mav_.SetVel(mav_vel);

    // 2. set post and vel of payload in vertical direction
    const Eigen::Vector3d attachpoint_post{0,0,0};
    const Eigen::Vector3d attachpoint_vel{0,0,RandomGenerate(-0.1,0)};

    // 3. check collision
    bool isCollided = ptr_UAVCable->IsCollided(attachpoint_post, attachpoint_vel);

    // 4.
    ASSERT_TRUE(isCollided);
}


// test vertical collision and vel redistribution
TEST_F(rotorTMUAVCableTest, checkVerticalVelDistribution){

    // 1. set post and vel of mav in vertical direction
    const double mav_vel_z = 1;
    const Eigen::Vector3d mav_vel{0,0,mav_vel_z};
    ptr_UAVCable->mav_.SetVel(mav_vel);

    double cable_length;
    ptr_UAVCable->cable_.GetCableLength(cable_length);
    const Eigen::Vector3d mav_post{0,0,cable_length+RandomGenerate(0,0.1)};
    ptr_UAVCable->SetMAVInitPost(mav_post);
    
    // 2. set post and vel of payload in vertical direction
    const double payload_vel_z = -1;
    const Eigen::Vector3d attachpoint_vel{0,0,payload_vel_z};
    const Eigen::Vector3d attachpoint_post{0,0,0};
    

    // 3. check collision
    bool isCollided = ptr_UAVCable->IsCollided(attachpoint_post, attachpoint_vel);

    ASSERT_TRUE(isCollided);

    // 4. compute vels after collision
    ptr_UAVCable->UpdateVelCollidedMAVVel(Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),  Eigen::Vector3d::Zero());

    // 
    Eigen::Vector3d mav_vel_collided;

    ptr_UAVCable->mav_.GetVel(mav_vel_collided);

    ASSERT_FLOAT_EQ(mav_vel_collided[0], 0); 
    ASSERT_FLOAT_EQ(mav_vel_collided[1], 0); 
    ASSERT_FLOAT_EQ(mav_vel_collided[2], -0);  
}


// test vertical collision and vel redistribution with impluse-momentum
TEST_F(rotorTMUAVCableTest, checkVerticalVelDistributionWithPayloadMotion){


    // m_q *v_q + m_p * v_p =  m_q *v_q_collided + m_p * v_p_collided
    // we assume collision is inelastic, then v_p_collided = v_q_collided
    // assign values to m_q, m_p, v_p, v_p_collided
    //  v_q_collided = (m_q *v_q + m_p * v_p)/(m_q+m_p)
    // e.g. m_q =1; v_p = 1, v_p_collided = 0.2

    // 1. set post and vel of mav in vertical direction
    const double mav_vel_z = 1;

    const Eigen::Vector3d mav_vel{0,0,mav_vel_z};
    ptr_UAVCable->mav_.SetVel(mav_vel);

    double cable_length;
    ptr_UAVCable->cable_.GetCableLength(cable_length);
    const Eigen::Vector3d mav_post{0,0,cable_length+RandomGenerate(0,0.1)};
    ptr_UAVCable->SetMAVInitPost(mav_post);
    
    // 2. set post and vel of payload in vertical direction
    // make payload a negative vel to ensure collision happen 
    const double payload_vel_z = RandomGenerate(-1, 0);
    const Eigen::Vector3d attachpoint_vel{0,0,payload_vel_z};

    const Eigen::Vector3d attachpoint_post{0,0,0};
    
    // assume payload mass payload vel after collision
    double payload_vel_z_collised = 0.2;

    const Eigen::Vector3d payload_vel_collided{0,0,payload_vel_z_collised};

    // 3. check collision
    bool isCollided = ptr_UAVCable->IsCollided(attachpoint_post, attachpoint_vel);

    ASSERT_TRUE(isCollided);

    // 4. compute vels after collision
    ptr_UAVCable->UpdateVelCollidedMAVVel(Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), payload_vel_collided,  Eigen::Vector3d::Zero());

    // 
    Eigen::Vector3d mav_vel_collided;

    ptr_UAVCable->mav_.GetVel(mav_vel_collided);

    ASSERT_FLOAT_EQ(mav_vel_collided[0], 0); 
    ASSERT_FLOAT_EQ(mav_vel_collided[1], 0); 
    ASSERT_FLOAT_EQ(mav_vel_collided[2], 0.2);  
}

// test vertical collision and vel redistribution with impluse-momentum
TEST_F(rotorTMUAVCableTest, checkVerticalVelDistributionWithRandomMotion){


    // m_q *v_q + m_p * v_p =  m_q *v_q_collided + m_p * v_p_collided
    // we assume collision is inelastic, then v_p_collided = v_q_collided
    // assign values to m_q, m_p, v_p, v_p_collided
    //  v_q_collided = (m_q *v_q + m_p * v_p)/(m_q+m_p)
    // e.g. m_q =1; v_p = 1, v_p_collided = 0.2

    // 1. set post and vel of mav in vertical direction
    const double mav_vel_z = RandomGenerate(0,5);

    const Eigen::Vector3d mav_vel{0,0,mav_vel_z};
    ptr_UAVCable->mav_.SetVel(mav_vel);

    double cable_length;
    ptr_UAVCable->cable_.GetCableLength(cable_length);

    const Eigen::Vector3d mav_post{0,0,cable_length+RandomGenerate(0,0.1)};
    ptr_UAVCable->SetMAVInitPost(mav_post);
    
    // 2. set post and vel of payload in vertical direction
    // make payload a negative vel to ensure collision happen 
    const double payload_vel_z = RandomGenerate(-1, 0);

    const Eigen::Vector3d attachpoint_vel{0,0,payload_vel_z};
    const Eigen::Vector3d attachpoint_post{0,0,0};
    
    // assume payload mass payload vel after collision
    double payload_vel_z_collised = RandomGenerate(0,4);

    const Eigen::Vector3d payload_vel_collided{0,0,payload_vel_z_collised};

    // 3. check collision
    bool isCollided = ptr_UAVCable->IsCollided(attachpoint_post, attachpoint_vel);

    ASSERT_TRUE(isCollided);

    // 4. compute vels after collision
    ptr_UAVCable->UpdateVelCollidedMAVVel(Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), payload_vel_collided,  Eigen::Vector3d::Zero());

    // 
    Eigen::Vector3d mav_vel_collided;

    ptr_UAVCable->mav_.GetVel(mav_vel_collided);

    ASSERT_FLOAT_EQ(mav_vel_collided[0], 0); 
    ASSERT_FLOAT_EQ(mav_vel_collided[1], 0); 
    ASSERT_FLOAT_EQ(mav_vel_collided[2], payload_vel_z_collised);  
}

// test vertical collision and vel redistribution with impluse-momentum
TEST_F(rotorTMUAVCableTest, checkTiltedXVelDistribution){
    // collision along cable direction
    // mav vel = vel along cable + vel prependicular to cable
    // for vel along cable direction    
    // m_q *v_q + m_p * v_p =  m_q *v_q_collided + m_p * v_p_collided
    // we assume collision is inelastic, then v_p_collided = v_q_collided
    // assign values to m_q, m_p, v_p, v_p_collided
    //  v_q_collided = (m_q *v_q + m_p * v_p)/(m_q+m_p)
    // e.g. m_q =1; v_p = 1, v_p_collided = 0.2

    // 1. set post and vel of mav in vertical direction
    const double mav_vel_z = 1;
    const double mav_vel_x = 1;

    const Eigen::Vector3d mav_vel{mav_vel_x,0,mav_vel_z};

    ptr_UAVCable->mav_.SetVel(mav_vel);

    double cable_length;
    ptr_UAVCable->cable_.GetCableLength(cable_length);
    const Eigen::Vector3d mav_post{0,0,cable_length+RandomGenerate(0,0.1)};
    ptr_UAVCable->SetMAVInitPost(mav_post);
    
    // 2. set post and vel of payload in vertical direction
    // make payload a negative vel to ensure collision happen 
    const double payload_vel_z = RandomGenerate(-0.5, 0);

    const Eigen::Vector3d attachpoint_vel{0,0,payload_vel_z};
    const Eigen::Vector3d attachpoint_post{0,0,0};
    
    // assume payload mass payload vel after collision
    double payload_vel_z_collised = 0.2;

    const Eigen::Vector3d payload_vel_collided{0,0,payload_vel_z_collised};

    // 3. check collision
    bool isCollided = ptr_UAVCable->IsCollided(attachpoint_post, attachpoint_vel);

    ASSERT_TRUE(isCollided);

    // 4. compute vels after collision
    ptr_UAVCable->UpdateVelCollidedMAVVel(Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), payload_vel_collided,  Eigen::Vector3d::Zero());

    // 
    Eigen::Vector3d mav_vel_collided;

    ptr_UAVCable->mav_.GetVel(mav_vel_collided);

    ASSERT_FLOAT_EQ(mav_vel_collided[0], 1); 
    ASSERT_FLOAT_EQ(mav_vel_collided[1], 0); 
    ASSERT_FLOAT_EQ(mav_vel_collided[2], 0.2);  
}


// test vertical collision and vel redistribution with impluse-momentum
TEST_F(rotorTMUAVCableTest, checkTiltedXVelDistributionRandomMotion){
    // collision along cable direction
    // mav vel = vel along cable + vel prependicular to cable
    // for vel along cable direction    
    // m_q *v_q + m_p * v_p =  m_q *v_q_collided + m_p * v_p_collided
    // we assume collision is inelastic, then v_p_collided = v_q_collided
    // assign values to m_q, m_p, v_p, v_p_collided
    //  v_q_collided = (m_q *v_q + m_p * v_p)/(m_q+m_p)
    // e.g. m_q =1; v_p = 1, v_p_collided = 0.2

    // 1. set post and vel of mav in vertical direction
    const double mav_vel_z = RandomGenerate(0,4);
    const double mav_vel_x = RandomGenerate(0,5);

    const Eigen::Vector3d mav_vel{mav_vel_x,0,mav_vel_z};

    ptr_UAVCable->mav_.SetVel(mav_vel);

    double cable_length;
    ptr_UAVCable->cable_.GetCableLength(cable_length);
    const Eigen::Vector3d mav_post{0,0,cable_length+RandomGenerate(0,0.1)};
    ptr_UAVCable->SetMAVInitPost(mav_post);
    
    // 2. set post and vel of payload in vertical direction
    // make payload a negative vel to ensure collision happen 
    const double payload_vel_z = RandomGenerate(-1, 0);

    const Eigen::Vector3d attachpoint_vel{0,0,payload_vel_z};
    const Eigen::Vector3d attachpoint_post{0,0,0};
    
    // assume payload mass payload vel after collision
    double payload_vel_z_collised = RandomGenerate(0,2);

    const Eigen::Vector3d payload_vel_collided{0,0,payload_vel_z_collised};

    // 3. check collision
    bool isCollided = ptr_UAVCable->IsCollided(attachpoint_post, attachpoint_vel);

    ASSERT_TRUE(isCollided);

    // 4. compute vels after collision
    ptr_UAVCable->UpdateVelCollidedMAVVel(Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), payload_vel_collided,  Eigen::Vector3d::Zero());

    // 
    Eigen::Vector3d mav_vel_collided;

    ptr_UAVCable->mav_.GetVel(mav_vel_collided);

    ASSERT_FLOAT_EQ(mav_vel_collided[0], mav_vel_x); 
    ASSERT_FLOAT_EQ(mav_vel_collided[1], 0); 
    ASSERT_FLOAT_EQ(mav_vel_collided[2], payload_vel_z_collised);  
}


// test vertical collision and vel redistribution with impluse-momentum
TEST_F(rotorTMUAVCableTest, checkTiltedYVelDistribution){
    // collision along cable direction
    // mav vel = vel along cable + vel prependicular to cable
    // for vel along cable direction
    // m_q *v_q + m_p * v_p =  m_q *v_q_collided + m_p * v_p_collided
    // we assume collision is inelastic, then v_p_collided = v_q_collided
    // assign values to m_q, m_p, v_p, v_p_collided
    //  v_q_collided = (m_q *v_q + m_p * v_p)/(m_q+m_p)
    // e.g. m_q =1; v_p = 1, v_p_collided = 0.2

    // 1. set post and vel of mav in vertical direction
    const double mav_vel_z = 1;
    const double mav_vel_y = 1;

    const Eigen::Vector3d mav_vel{0,mav_vel_y,mav_vel_z};

    ptr_UAVCable->mav_.SetVel(mav_vel);

    double cable_length;
    ptr_UAVCable->cable_.GetCableLength(cable_length);
    const Eigen::Vector3d mav_post{0,0,cable_length+RandomGenerate(0,0.1)};
    ptr_UAVCable->SetMAVInitPost(mav_post);
    
    // 2. set post and vel of payload in vertical direction
    // make payload a negative vel to ensure collision happen 
    const double payload_vel_z = RandomGenerate(-1, 0);

    const Eigen::Vector3d attachpoint_vel{0,0,payload_vel_z};
    const Eigen::Vector3d attachpoint_post{0,0,0};
    
    // assume payload mass payload vel after collision
    double payload_mass = 1;
    double payload_vel_z_collised = 0.2;

    const Eigen::Vector3d payload_vel_collided{0,0,payload_vel_z_collised};

    // 3. check collision
    bool isCollided = ptr_UAVCable->IsCollided(attachpoint_post, attachpoint_vel);

    ASSERT_TRUE(isCollided);

    // 4. compute vels after collision
    ptr_UAVCable->UpdateVelCollidedMAVVel(Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), payload_vel_collided,  Eigen::Vector3d::Zero());

    // 
    Eigen::Vector3d mav_vel_collided;

    ptr_UAVCable->mav_.GetVel(mav_vel_collided);

    ASSERT_FLOAT_EQ(mav_vel_collided[0], 0); 
    ASSERT_FLOAT_EQ(mav_vel_collided[1], 1); 
    ASSERT_FLOAT_EQ(mav_vel_collided[2], 0.2);  
}



// test vertical collision and vel redistribution with impluse-momentum
TEST_F(rotorTMUAVCableTest, checkTiltedYVelDistributionRandomMotion){
    // collision along cable direction
    // mav vel = vel along cable + vel prependicular to cable
    // for vel along cable direction
    // m_q *v_q + m_p * v_p =  m_q *v_q_collided + m_p * v_p_collided
    // we assume collision is inelastic, then v_p_collided = v_q_collided
    // assign values to m_q, m_p, v_p, v_p_collided
    //  v_q_collided = (m_q *v_q + m_p * v_p)/(m_q+m_p)
    // e.g. m_q =1; v_p = 1, v_p_collided = 0.2

    // 1. set post and vel of mav in vertical direction
    const double mav_vel_z = RandomGenerate(0,4);;
    const double mav_vel_y = RandomGenerate(0,5);;

    const Eigen::Vector3d mav_vel{0,mav_vel_y,mav_vel_z};

    ptr_UAVCable->mav_.SetVel(mav_vel);

    double cable_length;
    ptr_UAVCable->cable_.GetCableLength(cable_length);
    const Eigen::Vector3d mav_post{0,0,cable_length+RandomGenerate(0,0.1)};
    ptr_UAVCable->SetMAVInitPost(mav_post);
    
    // 2. set post and vel of payload in vertical direction
    // make payload a negative vel to ensure collision happen 
    const double payload_vel_z = RandomGenerate(-1, 0);

    const Eigen::Vector3d attachpoint_vel{0,0,payload_vel_z};
    const Eigen::Vector3d attachpoint_post{0,0,0};
    
    // assume payload mass payload vel after collision
    double payload_mass = 1;
    double payload_vel_z_collised = RandomGenerate(0,6);

    const Eigen::Vector3d payload_vel_collided{0,0,payload_vel_z_collised};

    // 3. check collision
    bool isCollided = ptr_UAVCable->IsCollided(attachpoint_post, attachpoint_vel);

    ASSERT_TRUE(isCollided);

    // 4. compute vels after collision
    ptr_UAVCable->UpdateVelCollidedMAVVel(Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), payload_vel_collided,  Eigen::Vector3d::Zero());

    // 
    Eigen::Vector3d mav_vel_collided;

    ptr_UAVCable->mav_.GetVel(mav_vel_collided);

    ASSERT_FLOAT_EQ(mav_vel_collided[0], 0); 
    ASSERT_FLOAT_EQ(mav_vel_collided[1], mav_vel_y); 
    ASSERT_FLOAT_EQ(mav_vel_collided[2], payload_vel_z_collised);  
}




/*****************************Dynamic test****************************/

TEST_F(rotorTMUAVCableTest, checkVerticalStaticHovering){
        // 1. set posts of mav and payload to make collision
        const double mav_vel_z = 0;
        const Eigen::Vector3d mav_vel{0,0,mav_vel_z};
        ptr_UAVCable->mav_.SetVel(mav_vel);

        double cable_length;
        ptr_UAVCable->cable_.GetCableLength(cable_length);
        const Eigen::Vector3d mav_post{0,0,cable_length+0.1};
        ptr_UAVCable->SetMAVInitPost(mav_post);
        
        // 2. set post and vel of payload in vertical direction
        const double payload_vel_z = 0;
        const Eigen::Vector3d attachpoint_vel{0,0,payload_vel_z};
        const Eigen::Vector3d attachpoint_post{0,0,0};


        // 4. set control input for quadrotor
        // given that m_q = 1kg and assume m_p = 1kg
        // apply a thrust force 2*9.8N = 19.6N
        // acc = (19.6 - 2*9.8)/2 = 0
        // take payload a an object
        // -t - 1 * 9.8 = 0 ===> -t = 9.8N
        const double mav_thrust = 9.8*2;
        std::cout<<"[----------] mav_thrust is " << mav_thrust<<std::endl;
        
        ptr_UAVCable->InputControllerInput(mav_thrust,Eigen::Vector3d::Zero());

        Eigen::Vector3d attach_point_acc{0,0,9.8};
        ptr_UAVCable->ComputeControlInputs4MAV(attach_point_acc);


        // 5. compute wrenches at attach point when whole system is at static equilibrium
        // ComputeAttachPointWrenches(const Eigen::Vector3d &attach_point_post, const Eigen::Vector3d &attach_point_vel, const Eigen::Quaterniond &payload_attitude, Eigen::Vector3d &payload_bodyrate)
        Eigen::Vector3d zeroVector = Eigen::Vector3d::Zero();
        Eigen::Quaterniond identityQuat = Eigen::Quaterniond::Identity();

        // ComputeAttachPointWrenches(const Eigen::Vector3d &attach_point_post, const Eigen::Vector3d &attach_point_vel, const Eigen::Quaterniond &payload_attitude, Eigen::Vector3d &payload_bodyrate)
        ptr_UAVCable->ComputeAttachPointWrenches(zeroVector, zeroVector, identityQuat, zeroVector);

        // 6. one step dynamic simulation
        ptr_UAVCable->DoOneStepInt();

        // 7. obtain quadrotor acc, vel, post
        // check position = 0
        Eigen::Vector3d pos = Eigen::Vector3d::Random();

        ptr_UAVCable->mav_.GetPosition(pos);

        EXPECT_FLOAT_EQ(pos[0], mav_post[0]); 
        EXPECT_FLOAT_EQ(pos[1], mav_post[1]); 
        EXPECT_FLOAT_EQ(pos[2], mav_post[2]);   

        // check vel = 0
        Eigen::Vector3d vel = Eigen::Vector3d::Random();
        ptr_UAVCable->mav_.GetVel(vel);

        EXPECT_FLOAT_EQ(vel[0], mav_vel[0]); 
        EXPECT_FLOAT_EQ(vel[1], mav_vel[1]); 
        EXPECT_FLOAT_EQ(vel[2], mav_vel[2]);  

        // check acc = 0
        Eigen::Vector3d acc = Eigen::Vector3d::Random();
        ptr_UAVCable->mav_.GetAcc(acc);
        EXPECT_FLOAT_EQ(acc[0], 0); 
        EXPECT_FLOAT_EQ(acc[1], 0); 
        EXPECT_FLOAT_EQ(acc[2], 0);  

        // check bodyrate
        Eigen::Vector3d bodyrate = Eigen::Vector3d::Random();
        ptr_UAVCable->mav_.GetBodyrate(bodyrate);
        EXPECT_FLOAT_EQ(bodyrate[0], 0); 
        EXPECT_FLOAT_EQ(bodyrate[1], 0); 
        EXPECT_FLOAT_EQ(bodyrate[2], 0);  

        // check attitude
        Eigen::Quaterniond att = Eigen::Quaterniond::UnitRandom();
        ptr_UAVCable->mav_.GetAttitude(att);
        EXPECT_FLOAT_EQ(att.x(), 0);    
        EXPECT_FLOAT_EQ(att.y(), 0);  
        EXPECT_FLOAT_EQ(att.z(), 0);  
        EXPECT_FLOAT_EQ(att.w(), 1);  

        // check bodyrate_acc
        Eigen::Vector3d bodyrate_acc = Eigen::Vector3d::Random();
        ptr_UAVCable->mav_.GetBodyRateAcc(bodyrate_acc);
        EXPECT_FLOAT_EQ(bodyrate_acc[0], 0); 
        EXPECT_FLOAT_EQ(bodyrate_acc[1], 0); 
        EXPECT_FLOAT_EQ(bodyrate_acc[2], 0);  
}












TEST_F(rotorTMUAVCableTest, checkVerticalStaticHovering100Steps){
        // 1. set posts of mav and payload to make collision
        const double mav_vel_z = 0;
        const Eigen::Vector3d mav_vel{0,0,mav_vel_z};
        ptr_UAVCable->mav_.SetVel(mav_vel);

        double cable_length;
        ptr_UAVCable->cable_.GetCableLength(cable_length);
        const Eigen::Vector3d mav_post{0,0,cable_length+0.1};
        ptr_UAVCable->SetMAVInitPost(mav_post);
        
        // 2. set post and vel of payload in vertical direction
        const double payload_vel_z = 0;
        const Eigen::Vector3d attachpoint_vel{0,0,payload_vel_z};
        const Eigen::Vector3d attachpoint_post{0,0,0};


        // 4. set control input for quadrotor
        // given that m_q = 1kg and assume m_p = 1kg
        // apply a thrust force 2*9.8N = 19.6N
        // acc = (19.6 - 2*9.8)/2 = 0
        // take payload a an object
        // -t - 1 * 9.8 = 0 ===> -t = 9.8N
        const double mav_thrust = 9.8*2;
        std::cout<<"[----------] mav_thrust is " << mav_thrust<<std::endl;
        
        ptr_UAVCable->InputControllerInput(mav_thrust,Eigen::Vector3d::Zero());

        Eigen::Vector3d attach_point_acc{0,0,9.8}; // here is gravity
        ptr_UAVCable->ComputeControlInputs4MAV(attach_point_acc);


        // 5. compute wrenches at attach point when whole system is at static equilibrium
        // ComputeAttachPointWrenches(const Eigen::Vector3d &attach_point_post, const Eigen::Vector3d &attach_point_vel, const Eigen::Quaterniond &payload_attitude, Eigen::Vector3d &payload_bodyrate)
        Eigen::Vector3d zeroVector = Eigen::Vector3d::Zero();
        Eigen::Quaterniond identityQuat = Eigen::Quaterniond::Identity();

        // ComputeAttachPointWrenches(const Eigen::Vector3d &attach_point_post, const Eigen::Vector3d &attach_point_vel, const Eigen::Quaterniond &payload_attitude, Eigen::Vector3d &payload_bodyrate)
        ptr_UAVCable->ComputeAttachPointWrenches(zeroVector, zeroVector, identityQuat, zeroVector);

        // 6. 100 steps dynamic simulation
        const double dt = 0.01;
        for(double t=dt ; t<=100*dt ; t+= dt)
        {
                ptr_UAVCable->DoOneStepInt();;
                // printf("current step is %.3f \n", t);
        }
        
        // 7. obtain quadrotor acc, vel, post
        // check position = 0
        Eigen::Vector3d pos = Eigen::Vector3d::Random();

        ptr_UAVCable->mav_.GetPosition(pos);

        EXPECT_FLOAT_EQ(pos[0], mav_post[0]); 
        EXPECT_FLOAT_EQ(pos[1], mav_post[1]); 
        EXPECT_FLOAT_EQ(pos[2], mav_post[2]);   

        // check vel = 0
        Eigen::Vector3d vel = Eigen::Vector3d::Random();
        ptr_UAVCable->mav_.GetVel(vel);

        EXPECT_FLOAT_EQ(vel[0], mav_vel[0]); 
        EXPECT_FLOAT_EQ(vel[1], mav_vel[1]); 
        EXPECT_FLOAT_EQ(vel[2], mav_vel[2]);  

        // check acc = 0
        Eigen::Vector3d acc = Eigen::Vector3d::Random();
        ptr_UAVCable->mav_.GetAcc(acc);
        EXPECT_FLOAT_EQ(acc[0], 0); 
        EXPECT_FLOAT_EQ(acc[1], 0); 
        EXPECT_FLOAT_EQ(acc[2], 0);  

        // check bodyrate
        Eigen::Vector3d bodyrate = Eigen::Vector3d::Random();
        ptr_UAVCable->mav_.GetBodyrate(bodyrate);
        EXPECT_FLOAT_EQ(bodyrate[0], 0); 
        EXPECT_FLOAT_EQ(bodyrate[1], 0); 
        EXPECT_FLOAT_EQ(bodyrate[2], 0);  

        // check attitude
        Eigen::Quaterniond att = Eigen::Quaterniond::UnitRandom();
        ptr_UAVCable->mav_.GetAttitude(att);
        EXPECT_FLOAT_EQ(att.x(), 0);    
        EXPECT_FLOAT_EQ(att.y(), 0);  
        EXPECT_FLOAT_EQ(att.z(), 0);  
        EXPECT_FLOAT_EQ(att.w(), 1);  

        // check bodyrate_acc
        Eigen::Vector3d bodyrate_acc = Eigen::Vector3d::Random();
        ptr_UAVCable->mav_.GetBodyRateAcc(bodyrate_acc);
        EXPECT_FLOAT_EQ(bodyrate_acc[0], 0); 
        EXPECT_FLOAT_EQ(bodyrate_acc[1], 0); 
        EXPECT_FLOAT_EQ(bodyrate_acc[2], 0);  
}







TEST_F(rotorTMUAVCableTest, checkVerticalConstAcc){
        // 1. set posts of mav and payload to make collision
        const double mav_vel_z = 0;
        const Eigen::Vector3d mav_init_vel{0,0,mav_vel_z};
        ptr_UAVCable->mav_.SetVel(mav_init_vel);

        double cable_length;
        ptr_UAVCable->cable_.GetCableLength(cable_length);
        const Eigen::Vector3d mav_init_post{0,0,cable_length+0.1};
        ptr_UAVCable->SetMAVInitPost(mav_init_post);
        
        // 2. set post and vel of payload in vertical direction
        const double payload_vel_z = 0;
        const Eigen::Vector3d attachpoint_vel{0,0,payload_vel_z};
        const Eigen::Vector3d attachpoint_post{0,0,0};


        // 4. set control input for quadrotor
        // given that m_q = 1kg and assume m_p = 1kg
        // apply a thrust force 3*9.8N = 29.4N
        // acc = (29.4 - 2*9.8)/2 = 4.9
        // take payload a an object
        // -t - 1 * 9.8 = 4.9 ===> -t = 14.7N
        const double mav_thrust = 9.8*3;
        std::cout<<"[----------] mav_thrust is " << mav_thrust<<std::endl;
        
        ptr_UAVCable->InputControllerInput(mav_thrust,Eigen::Vector3d::Zero());

        Eigen::Vector3d attach_point_acc{0,0,9.8+4.9};
        ptr_UAVCable->ComputeControlInputs4MAV(attach_point_acc);


        // 5. compute wrenches at attach point when whole system is at static equilibrium
        // ComputeAttachPointWrenches(const Eigen::Vector3d &attach_point_post, const Eigen::Vector3d &attach_point_vel, const Eigen::Quaterniond &payload_attitude, Eigen::Vector3d &payload_bodyrate)
        Eigen::Vector3d zeroVector = Eigen::Vector3d::Zero();
        Eigen::Quaterniond identityQuat = Eigen::Quaterniond::Identity();

        // ComputeAttachPointWrenches(const Eigen::Vector3d &attach_point_post, const Eigen::Vector3d &attach_point_vel, const Eigen::Quaterniond &payload_attitude, Eigen::Vector3d &payload_bodyrate)
        ptr_UAVCable->ComputeAttachPointWrenches(zeroVector, zeroVector, identityQuat, zeroVector);

        // 6. 1 steps dynamic simulation
        // const double dt = 0.01;
        // for(double t=dt ; t<=100*dt ; t+= dt)
        // {
                ptr_UAVCable->DoOneStepInt();;
                // printf("current step is %.3f \n", t);
        // }
        
        // 7. obtain quadrotor acc, vel, post
        // check position = 0.5* a*t^2
        Eigen::Vector3d pos = Eigen::Vector3d::Random();

        ptr_UAVCable->mav_.GetPosition(pos);

        EXPECT_FLOAT_EQ(pos[0], mav_init_post[0]); 
        EXPECT_FLOAT_EQ(pos[1], mav_init_post[1]); 
        EXPECT_FLOAT_EQ(pos[2], mav_init_post[2] + 0.5 * 4.9 * pow(0.01, 2));   

        // check vel = at
        Eigen::Vector3d vel = Eigen::Vector3d::Random();
        ptr_UAVCable->mav_.GetVel(vel);

        EXPECT_FLOAT_EQ(vel[0], mav_init_vel[0]); 
        EXPECT_FLOAT_EQ(vel[1], mav_init_vel[1]); 
        EXPECT_FLOAT_EQ(vel[2], mav_init_vel[2] + 4.9 * 0.01);  

        // check acc = a
        Eigen::Vector3d acc = Eigen::Vector3d::Random();
        ptr_UAVCable->mav_.GetAcc(acc);
        EXPECT_FLOAT_EQ(acc[0], 0); 
        EXPECT_FLOAT_EQ(acc[1], 0); 
        EXPECT_FLOAT_EQ(acc[2], 4.9);  

        // check bodyrate
        Eigen::Vector3d bodyrate = Eigen::Vector3d::Random();
        ptr_UAVCable->mav_.GetBodyrate(bodyrate);
        EXPECT_FLOAT_EQ(bodyrate[0], 0); 
        EXPECT_FLOAT_EQ(bodyrate[1], 0); 
        EXPECT_FLOAT_EQ(bodyrate[2], 0);  

        // check attitude
        Eigen::Quaterniond att = Eigen::Quaterniond::UnitRandom();
        ptr_UAVCable->mav_.GetAttitude(att);
        EXPECT_FLOAT_EQ(att.x(), 0);    
        EXPECT_FLOAT_EQ(att.y(), 0);  
        EXPECT_FLOAT_EQ(att.z(), 0);  
        EXPECT_FLOAT_EQ(att.w(), 1);  

        // check bodyrate_acc
        Eigen::Vector3d bodyrate_acc = Eigen::Vector3d::Random();
        ptr_UAVCable->mav_.GetBodyRateAcc(bodyrate_acc);
        EXPECT_FLOAT_EQ(bodyrate_acc[0], 0); 
        EXPECT_FLOAT_EQ(bodyrate_acc[1], 0); 
        EXPECT_FLOAT_EQ(bodyrate_acc[2], 0);  
}




TEST_F(rotorTMUAVCableTest, checkVerticalConstAccTenSteps){
        // 1. set posts of mav and payload to make collision
        const double mav_vel_z = 0;
        const Eigen::Vector3d mav_init_vel{0,0,mav_vel_z};
        ptr_UAVCable->mav_.SetVel(mav_init_vel);

        double cable_length;
        ptr_UAVCable->cable_.GetCableLength(cable_length);
        const Eigen::Vector3d mav_init_post{0,0,cable_length+0.1};
        ptr_UAVCable->SetMAVInitPost(mav_init_post);
        
        // 2. set post and vel of payload in vertical direction
        const double payload_vel_z = 0;
        const Eigen::Vector3d attachpoint_vel{0,0,payload_vel_z};
        const Eigen::Vector3d attachpoint_post{0,0,0};


        // 4. set control input for quadrotor
        // given that m_q = 1kg and assume m_p = 1kg
        // apply a thrust force 3*9.8N = 29.4N
        // acc = (29.4 - 2*9.8)/2 = 4.9
        // take payload a an object
        // -t - 1 * 9.8 = 4.9 ===> -t = 14.7N
        const double mav_thrust = 9.8*3;
        std::cout<<"[----------] mav_thrust is " << mav_thrust<<std::endl;
        
        ptr_UAVCable->InputControllerInput(mav_thrust,Eigen::Vector3d::Zero());

        Eigen::Vector3d attach_point_acc{0,0,9.8+4.9};
        ptr_UAVCable->ComputeControlInputs4MAV(attach_point_acc);


        // 5. compute wrenches at attach point when whole system is at static equilibrium
        // ComputeAttachPointWrenches(const Eigen::Vector3d &attach_point_post, const Eigen::Vector3d &attach_point_vel, const Eigen::Quaterniond &payload_attitude, Eigen::Vector3d &payload_bodyrate)
        Eigen::Vector3d zeroVector = Eigen::Vector3d::Zero();
        Eigen::Quaterniond identityQuat = Eigen::Quaterniond::Identity();

        // ComputeAttachPointWrenches(const Eigen::Vector3d &attach_point_post, const Eigen::Vector3d &attach_point_vel, const Eigen::Quaterniond &payload_attitude, Eigen::Vector3d &payload_bodyrate)
        ptr_UAVCable->ComputeAttachPointWrenches(zeroVector, zeroVector, identityQuat, zeroVector);

        // 6. 2 steps dynamic simulation
        const double dt = 0.01;
        for(double t=dt ; t  - 1e-10 <=10*dt ; t+= dt)
        {
                ptr_UAVCable->DoOneStepInt();;
                // printf("current step is %.3f \n", t);
        }
        // 7. obtain quadrotor acc, vel, post
        // check position = 0.5* a*t^2
        Eigen::Vector3d pos = Eigen::Vector3d::Random();

        ptr_UAVCable->mav_.GetPosition(pos);

        EXPECT_FLOAT_EQ(pos[0], mav_init_post[0]); 
        EXPECT_FLOAT_EQ(pos[1], mav_init_post[1]); 
        EXPECT_FLOAT_EQ(pos[2], mav_init_post[2] + 0.5 * 4.9 * pow(0.01*10, 2));   

        std::cout<<"[----------] mav z translatio is " << mav_init_post[2] + 0.5 * 4.9 * pow(0.01*10, 2) <<std::endl;

        // check vel = at
        Eigen::Vector3d vel = Eigen::Vector3d::Random();
        ptr_UAVCable->mav_.GetVel(vel);

        EXPECT_FLOAT_EQ(vel[0], mav_init_vel[0]); 
        EXPECT_FLOAT_EQ(vel[1], mav_init_vel[1]); 
        EXPECT_FLOAT_EQ(vel[2], mav_init_vel[2] + 4.9 * 0.01*10);  

        // check acc = a
        Eigen::Vector3d acc = Eigen::Vector3d::Random();
        ptr_UAVCable->mav_.GetAcc(acc);
        EXPECT_FLOAT_EQ(acc[0], 0); 
        EXPECT_FLOAT_EQ(acc[1], 0); 
        EXPECT_FLOAT_EQ(acc[2], 4.9);  

        // check bodyrate
        Eigen::Vector3d bodyrate = Eigen::Vector3d::Random();
        ptr_UAVCable->mav_.GetBodyrate(bodyrate);
        EXPECT_FLOAT_EQ(bodyrate[0], 0); 
        EXPECT_FLOAT_EQ(bodyrate[1], 0); 
        EXPECT_FLOAT_EQ(bodyrate[2], 0);  

        // check attitude
        Eigen::Quaterniond att = Eigen::Quaterniond::UnitRandom();
        ptr_UAVCable->mav_.GetAttitude(att);
        EXPECT_FLOAT_EQ(att.x(), 0);    
        EXPECT_FLOAT_EQ(att.y(), 0);  
        EXPECT_FLOAT_EQ(att.z(), 0);  
        EXPECT_FLOAT_EQ(att.w(), 1);  

        // check bodyrate_acc
        Eigen::Vector3d bodyrate_acc = Eigen::Vector3d::Random();
        ptr_UAVCable->mav_.GetBodyRateAcc(bodyrate_acc);
        EXPECT_FLOAT_EQ(bodyrate_acc[0], 0); 
        EXPECT_FLOAT_EQ(bodyrate_acc[1], 0); 
        EXPECT_FLOAT_EQ(bodyrate_acc[2], 0);  
}

TEST_F(rotorTMUAVCableTest, checkVerticalConstAccHundSteps){
        // 1. set posts of mav and payload to make collision
        const double mav_vel_z = 0;
        const Eigen::Vector3d mav_init_vel{0,0,mav_vel_z};
        ptr_UAVCable->mav_.SetVel(mav_init_vel);

        double cable_length;
        ptr_UAVCable->cable_.GetCableLength(cable_length);
        const Eigen::Vector3d mav_init_post{0,0,cable_length+0.1};
        ptr_UAVCable->SetMAVInitPost(mav_init_post);
        
        // 2. set post and vel of payload in vertical direction
        const double payload_vel_z = 0;
        const Eigen::Vector3d attachpoint_vel{0,0,payload_vel_z};
        const Eigen::Vector3d attachpoint_post{0,0,0};


        // 4. set control input for quadrotor
        // given that m_q = 1kg and assume m_p = 1kg
        // apply a thrust force 3*9.8N = 29.4N
        // acc = (29.4 - 2*9.8)/2 = 4.9
        // take payload a an object
        // -t - 1 * 9.8 = 4.9 ===> -t = 14.7N
        const double mav_thrust = 9.8*3;
        std::cout<<"[----------] mav_thrust is " << mav_thrust<<std::endl;
        
        ptr_UAVCable->InputControllerInput(mav_thrust,Eigen::Vector3d::Zero());

        Eigen::Vector3d attach_point_acc{0,0,9.8+4.9};
        ptr_UAVCable->ComputeControlInputs4MAV(attach_point_acc);


        // 5. compute wrenches at attach point when whole system is at static equilibrium
        // ComputeAttachPointWrenches(const Eigen::Vector3d &attach_point_post, const Eigen::Vector3d &attach_point_vel, const Eigen::Quaterniond &payload_attitude, Eigen::Vector3d &payload_bodyrate)
        Eigen::Vector3d zeroVector = Eigen::Vector3d::Zero();
        Eigen::Quaterniond identityQuat = Eigen::Quaterniond::Identity();

        // ComputeAttachPointWrenches(const Eigen::Vector3d &attach_point_post, const Eigen::Vector3d &attach_point_vel, const Eigen::Quaterniond &payload_attitude, Eigen::Vector3d &payload_bodyrate)
        ptr_UAVCable->ComputeAttachPointWrenches(zeroVector, zeroVector, identityQuat, zeroVector);

        // 6. 2 steps dynamic simulation
        const double dt = 0.01;
        for(double t=dt ; t - 1e-10<=100*dt ; t+= dt)
        {
                ptr_UAVCable->DoOneStepInt();;
                // printf("current step is %.3f \n", t);
        }
        // 7. obtain quadrotor acc, vel, post
        // check position = 0.5* a*t^2
        Eigen::Vector3d pos = Eigen::Vector3d::Random();

        ptr_UAVCable->mav_.GetPosition(pos);

        EXPECT_FLOAT_EQ(pos[0], mav_init_post[0]); 
        EXPECT_FLOAT_EQ(pos[1], mav_init_post[1]); 
        EXPECT_FLOAT_EQ(pos[2], mav_init_post[2] + 0.5 * 4.9 * pow(0.01*100, 2));   

        std::cout<<"[----------] mav z translatio is " << mav_init_post[2] + 0.5 * 4.9 * pow(0.01*1000, 2) <<std::endl;

        // check vel = at
        Eigen::Vector3d vel = Eigen::Vector3d::Random();
        ptr_UAVCable->mav_.GetVel(vel);

        EXPECT_FLOAT_EQ(vel[0], mav_init_vel[0]); 
        EXPECT_FLOAT_EQ(vel[1], mav_init_vel[1]); 
        EXPECT_FLOAT_EQ(vel[2], mav_init_vel[2] + 4.9 * 0.01*100);  

        // check acc = a
        Eigen::Vector3d acc = Eigen::Vector3d::Random();
        ptr_UAVCable->mav_.GetAcc(acc);
        EXPECT_FLOAT_EQ(acc[0], 0); 
        EXPECT_FLOAT_EQ(acc[1], 0); 
        EXPECT_FLOAT_EQ(acc[2], 4.9);  

        // check bodyrate
        Eigen::Vector3d bodyrate = Eigen::Vector3d::Random();
        ptr_UAVCable->mav_.GetBodyrate(bodyrate);
        EXPECT_FLOAT_EQ(bodyrate[0], 0); 
        EXPECT_FLOAT_EQ(bodyrate[1], 0); 
        EXPECT_FLOAT_EQ(bodyrate[2], 0);  

        // check attitude
        Eigen::Quaterniond att = Eigen::Quaterniond::UnitRandom();
        ptr_UAVCable->mav_.GetAttitude(att);
        EXPECT_FLOAT_EQ(att.x(), 0);    
        EXPECT_FLOAT_EQ(att.y(), 0);  
        EXPECT_FLOAT_EQ(att.z(), 0);  
        EXPECT_FLOAT_EQ(att.w(), 1);  

        // check bodyrate_acc
        Eigen::Vector3d bodyrate_acc = Eigen::Vector3d::Random();
        ptr_UAVCable->mav_.GetBodyRateAcc(bodyrate_acc);
        EXPECT_FLOAT_EQ(bodyrate_acc[0], 0); 
        EXPECT_FLOAT_EQ(bodyrate_acc[1], 0); 
        EXPECT_FLOAT_EQ(bodyrate_acc[2], 0);  
}

TEST_F(rotorTMUAVCableTest, checkVerticalConstAccThousandSteps){
        // 1. set posts of mav and payload to make collision
        const double mav_vel_z = 0;
        const Eigen::Vector3d mav_init_vel{0,0,mav_vel_z};
        ptr_UAVCable->mav_.SetVel(mav_init_vel);

        double cable_length;
        ptr_UAVCable->cable_.GetCableLength(cable_length);
        const Eigen::Vector3d mav_init_post{0,0,cable_length+0.1};
        ptr_UAVCable->SetMAVInitPost(mav_init_post);
        
        // 2. set post and vel of payload in vertical direction
        const double payload_vel_z = 0;
        const Eigen::Vector3d attachpoint_vel{0,0,payload_vel_z};
        const Eigen::Vector3d attachpoint_post{0,0,0};


        // 4. set control input for quadrotor
        // given that m_q = 1kg and assume m_p = 1kg
        // apply a thrust force 3*9.8N = 29.4N
        // acc = (29.4 - 2*9.8)/2 = 4.9
        // take payload a an object
        // -t - 1 * 9.8 = 4.9 ===> -t = 14.7N
        const double mav_thrust = 9.8*3;
        std::cout<<"[----------] mav_thrust is " << mav_thrust<<std::endl;
        
        ptr_UAVCable->InputControllerInput(mav_thrust,Eigen::Vector3d::Zero());

        Eigen::Vector3d attach_point_acc{0,0,9.8+4.9};
        ptr_UAVCable->ComputeControlInputs4MAV(attach_point_acc);


        // 5. compute wrenches at attach point when whole system is at static equilibrium
        // ComputeAttachPointWrenches(const Eigen::Vector3d &attach_point_post, const Eigen::Vector3d &attach_point_vel, const Eigen::Quaterniond &payload_attitude, Eigen::Vector3d &payload_bodyrate)
        Eigen::Vector3d zeroVector = Eigen::Vector3d::Zero();
        Eigen::Quaterniond identityQuat = Eigen::Quaterniond::Identity();

        // ComputeAttachPointWrenches(const Eigen::Vector3d &attach_point_post, const Eigen::Vector3d &attach_point_vel, const Eigen::Quaterniond &payload_attitude, Eigen::Vector3d &payload_bodyrate)
        ptr_UAVCable->ComputeAttachPointWrenches(zeroVector, zeroVector, identityQuat, zeroVector);

        // 6. 2 steps dynamic simulation
        const double dt = 0.01;
        for(double t=dt ; t - 1e-10<=1000*dt ; t+= dt)
        {
                ptr_UAVCable->DoOneStepInt();;
                // printf("current step is %.3f \n", t);
        }
        // 7. obtain quadrotor acc, vel, post
        // check position = 0.5* a*t^2
        Eigen::Vector3d pos = Eigen::Vector3d::Random();

        ptr_UAVCable->mav_.GetPosition(pos);

        EXPECT_FLOAT_EQ(pos[0], mav_init_post[0]); 
        EXPECT_FLOAT_EQ(pos[1], mav_init_post[1]); 
        EXPECT_FLOAT_EQ(pos[2], mav_init_post[2] + 0.5 * 4.9 * pow(0.01*1000, 2));   

        std::cout<<"[----------] mav z translatio is " << mav_init_post[2] + 0.5 * 4.9 * pow(0.01*1000, 2) <<std::endl;

        // check vel = at
        Eigen::Vector3d vel = Eigen::Vector3d::Random();
        ptr_UAVCable->mav_.GetVel(vel);

        EXPECT_FLOAT_EQ(vel[0], mav_init_vel[0]); 
        EXPECT_FLOAT_EQ(vel[1], mav_init_vel[1]); 
        EXPECT_FLOAT_EQ(vel[2], mav_init_vel[2] + 4.9 * 0.01*1000);  

        // check acc = a
        Eigen::Vector3d acc = Eigen::Vector3d::Random();
        ptr_UAVCable->mav_.GetAcc(acc);
        EXPECT_FLOAT_EQ(acc[0], 0); 
        EXPECT_FLOAT_EQ(acc[1], 0); 
        EXPECT_FLOAT_EQ(acc[2], 4.9);  

        // check bodyrate
        Eigen::Vector3d bodyrate = Eigen::Vector3d::Random();
        ptr_UAVCable->mav_.GetBodyrate(bodyrate);
        EXPECT_FLOAT_EQ(bodyrate[0], 0); 
        EXPECT_FLOAT_EQ(bodyrate[1], 0); 
        EXPECT_FLOAT_EQ(bodyrate[2], 0);  

        // check attitude
        Eigen::Quaterniond att = Eigen::Quaterniond::UnitRandom();
        ptr_UAVCable->mav_.GetAttitude(att);
        EXPECT_FLOAT_EQ(att.x(), 0);    
        EXPECT_FLOAT_EQ(att.y(), 0);  
        EXPECT_FLOAT_EQ(att.z(), 0);  
        EXPECT_FLOAT_EQ(att.w(), 1);  

        // check bodyrate_acc
        Eigen::Vector3d bodyrate_acc = Eigen::Vector3d::Random();
        ptr_UAVCable->mav_.GetBodyRateAcc(bodyrate_acc);
        EXPECT_FLOAT_EQ(bodyrate_acc[0], 0); 
        EXPECT_FLOAT_EQ(bodyrate_acc[1], 0); 
        EXPECT_FLOAT_EQ(bodyrate_acc[2], 0);  
}


TEST_F(rotorTMUAVCableTest, checkVerticalConstAccTenThousandSteps){
        // 1. set posts of mav and payload to make collision
        const double mav_vel_z = 0;
        const Eigen::Vector3d mav_init_vel{0,0,mav_vel_z};
        ptr_UAVCable->mav_.SetVel(mav_init_vel);

        double cable_length;
        ptr_UAVCable->cable_.GetCableLength(cable_length);
        const Eigen::Vector3d mav_init_post{0,0,cable_length+0.1};
        ptr_UAVCable->SetMAVInitPost(mav_init_post);
        
        // 2. set post and vel of payload in vertical direction
        const double payload_vel_z = 0;
        const Eigen::Vector3d attachpoint_vel{0,0,payload_vel_z};
        const Eigen::Vector3d attachpoint_post{0,0,0};


        // 4. set control input for quadrotor
        // given that m_q = 1kg and assume m_p = 1kg
        // apply a thrust force 3*9.8N = 29.4N
        // acc = (29.4 - 2*9.8)/2 = 4.9
        // take payload a an object
        // -t - 1 * 9.8 = 4.9 ===> -t = 14.7N
        const double mav_thrust = 9.8*3;
        std::cout<<"[----------] mav_thrust is " << mav_thrust<<std::endl;
        
        ptr_UAVCable->InputControllerInput(mav_thrust,Eigen::Vector3d::Zero());

        Eigen::Vector3d attach_point_acc{0,0,9.8+4.9};
        ptr_UAVCable->ComputeControlInputs4MAV(attach_point_acc);


        // 5. compute wrenches at attach point when whole system is at static equilibrium
        // ComputeAttachPointWrenches(const Eigen::Vector3d &attach_point_post, const Eigen::Vector3d &attach_point_vel, const Eigen::Quaterniond &payload_attitude, Eigen::Vector3d &payload_bodyrate)
        Eigen::Vector3d zeroVector = Eigen::Vector3d::Zero();
        Eigen::Quaterniond identityQuat = Eigen::Quaterniond::Identity();

        // ComputeAttachPointWrenches(const Eigen::Vector3d &attach_point_post, const Eigen::Vector3d &attach_point_vel, const Eigen::Quaterniond &payload_attitude, Eigen::Vector3d &payload_bodyrate)
        ptr_UAVCable->ComputeAttachPointWrenches(zeroVector, zeroVector, identityQuat, zeroVector);

        // 6. 1 100 steps dynamic simulation
        const double dt = 0.01;
        for(double t=dt ; t- 1e-10<=10000*dt ; t+= dt)
        {
                ptr_UAVCable->DoOneStepInt();;
                // printf("current step is %.3f \n", t);
        }
        
        // 7. obtain quadrotor acc, vel, post
        // check position = 0.5* a*t^2
        Eigen::Vector3d pos = Eigen::Vector3d::Random();

        ptr_UAVCable->mav_.GetPosition(pos);

        EXPECT_FLOAT_EQ(pos[0], mav_init_post[0]); 
        EXPECT_FLOAT_EQ(pos[1], mav_init_post[1]); 
        EXPECT_FLOAT_EQ(pos[2], mav_init_post[2] + 0.5 * 4.9 * pow(0.01*10000, 2));   

        // check vel = at
        Eigen::Vector3d vel = Eigen::Vector3d::Random();
        ptr_UAVCable->mav_.GetVel(vel);

        EXPECT_FLOAT_EQ(vel[0], mav_init_vel[0]); 
        EXPECT_FLOAT_EQ(vel[1], mav_init_vel[1]); 
        EXPECT_FLOAT_EQ(vel[2], mav_init_vel[2] + 4.9 * 0.01 * 10000);  

        // check acc = a
        Eigen::Vector3d acc = Eigen::Vector3d::Random();
        ptr_UAVCable->mav_.GetAcc(acc);
        EXPECT_FLOAT_EQ(acc[0], 0); 
        EXPECT_FLOAT_EQ(acc[1], 0); 
        EXPECT_FLOAT_EQ(acc[2], 4.9);  

        // check bodyrate
        Eigen::Vector3d bodyrate = Eigen::Vector3d::Random();
        ptr_UAVCable->mav_.GetBodyrate(bodyrate);
        EXPECT_FLOAT_EQ(bodyrate[0], 0); 
        EXPECT_FLOAT_EQ(bodyrate[1], 0); 
        EXPECT_FLOAT_EQ(bodyrate[2], 0);  

        // check attitude
        Eigen::Quaterniond att = Eigen::Quaterniond::UnitRandom();
        ptr_UAVCable->mav_.GetAttitude(att);
        EXPECT_FLOAT_EQ(att.x(), 0);    
        EXPECT_FLOAT_EQ(att.y(), 0);  
        EXPECT_FLOAT_EQ(att.z(), 0);  
        EXPECT_FLOAT_EQ(att.w(), 1);  

        // check bodyrate_acc
        Eigen::Vector3d bodyrate_acc = Eigen::Vector3d::Random();
        ptr_UAVCable->mav_.GetBodyRateAcc(bodyrate_acc);
        EXPECT_FLOAT_EQ(bodyrate_acc[0], 0); 
        EXPECT_FLOAT_EQ(bodyrate_acc[1], 0); 
        EXPECT_FLOAT_EQ(bodyrate_acc[2], 0);  
}



TEST_F(rotorTMUAVCableTest, checkVerticalConstAcc1ms2){
        // 1. set posts of mav and payload to make collision
        const double mav_vel_z = 0;
        const Eigen::Vector3d mav_init_vel{0,0,mav_vel_z};
        ptr_UAVCable->mav_.SetVel(mav_init_vel);

        double cable_length;
        ptr_UAVCable->cable_.GetCableLength(cable_length);
        const Eigen::Vector3d mav_init_post{0,0,cable_length+0.1};
        ptr_UAVCable->SetMAVInitPost(mav_init_post);
        
        // 2. set post and vel of payload in vertical direction
        const double payload_vel_z = 0;
        const Eigen::Vector3d attachpoint_vel{0,0,payload_vel_z};
        const Eigen::Vector3d attachpoint_post{0,0,0};


        // 4. set control input for quadrotor
        // given that m_q = 1kg and assume m_p = 1kg
        // apply a thrust force 21.6
        // acc = (21.6 - 2*9.8)/2 = 1
        // take payload a an object
        // -t - 1 * 9.8 = 1 ===> -t = 10.8N
        const double mav_thrust = 21.6;
        std::cout<<"[----------] mav_thrust is " << mav_thrust<<std::endl;
        
        ptr_UAVCable->InputControllerInput(mav_thrust,Eigen::Vector3d::Zero());

        Eigen::Vector3d attach_point_acc{0,0,9.8+1};
        ptr_UAVCable->ComputeControlInputs4MAV(attach_point_acc);


        // 5. compute wrenches at attach point when whole system is at static equilibrium
        // ComputeAttachPointWrenches(const Eigen::Vector3d &attach_point_post, const Eigen::Vector3d &attach_point_vel, const Eigen::Quaterniond &payload_attitude, Eigen::Vector3d &payload_bodyrate)
        Eigen::Vector3d zeroVector = Eigen::Vector3d::Zero();
        Eigen::Quaterniond identityQuat = Eigen::Quaterniond::Identity();

        // ComputeAttachPointWrenches(const Eigen::Vector3d &attach_point_post, const Eigen::Vector3d &attach_point_vel, const Eigen::Quaterniond &payload_attitude, Eigen::Vector3d &payload_bodyrate)
        ptr_UAVCable->ComputeAttachPointWrenches(zeroVector, zeroVector, identityQuat, zeroVector);

        // 6. 1 100 steps dynamic simulation
        const double dt = 0.01;
        for(double t=dt ; t- 1e-10<=10000*dt ; t+= dt)
        {
                ptr_UAVCable->DoOneStepInt();;
                // printf("current step is %.3f \n", t);
        }
        
        // 7. obtain quadrotor acc, vel, post
        // check position = 0.5* a*t^2
        Eigen::Vector3d pos = Eigen::Vector3d::Random();

        ptr_UAVCable->mav_.GetPosition(pos);

        EXPECT_FLOAT_EQ(pos[0], mav_init_post[0]); 
        EXPECT_FLOAT_EQ(pos[1], mav_init_post[1]); 
        EXPECT_FLOAT_EQ(pos[2], mav_init_post[2] + 0.5 * 1 * pow(0.01*10000, 2));   

        // check vel = at
        Eigen::Vector3d vel = Eigen::Vector3d::Random();
        ptr_UAVCable->mav_.GetVel(vel);

        EXPECT_FLOAT_EQ(vel[0], mav_init_vel[0]); 
        EXPECT_FLOAT_EQ(vel[1], mav_init_vel[1]); 
        EXPECT_FLOAT_EQ(vel[2], mav_init_vel[2] + 1 * 0.01 * 10000);  

        // check acc = a
        Eigen::Vector3d acc = Eigen::Vector3d::Random();
        ptr_UAVCable->mav_.GetAcc(acc);
        EXPECT_FLOAT_EQ(acc[0], 0); 
        EXPECT_FLOAT_EQ(acc[1], 0); 
        EXPECT_FLOAT_EQ(acc[2], 1);  

        // check bodyrate
        Eigen::Vector3d bodyrate = Eigen::Vector3d::Random();
        ptr_UAVCable->mav_.GetBodyrate(bodyrate);
        EXPECT_FLOAT_EQ(bodyrate[0], 0); 
        EXPECT_FLOAT_EQ(bodyrate[1], 0); 
        EXPECT_FLOAT_EQ(bodyrate[2], 0);  

        // check attitude
        Eigen::Quaterniond att = Eigen::Quaterniond::UnitRandom();
        ptr_UAVCable->mav_.GetAttitude(att);
        EXPECT_FLOAT_EQ(att.x(), 0);    
        EXPECT_FLOAT_EQ(att.y(), 0);  
        EXPECT_FLOAT_EQ(att.z(), 0);  
        EXPECT_FLOAT_EQ(att.w(), 1);  

        // check bodyrate_acc
        Eigen::Vector3d bodyrate_acc = Eigen::Vector3d::Random();
        ptr_UAVCable->mav_.GetBodyRateAcc(bodyrate_acc);
        EXPECT_FLOAT_EQ(bodyrate_acc[0], 0); 
        EXPECT_FLOAT_EQ(bodyrate_acc[1], 0); 
        EXPECT_FLOAT_EQ(bodyrate_acc[2], 0);  
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