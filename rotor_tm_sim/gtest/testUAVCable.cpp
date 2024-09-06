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


class rotorTMUAVCableTest : public ::testing::Test
{
public:

rotorTMUAVCableTest(){
    // double mass =1;
    // Eigen::Matrix3d m_inertia = Eigen::Matrix3d::Identity(3,3);

    // MassProperty mav_mass_property(1, Eigen::Matrix3d::Identity(3,3));
    MassProperty mav_mass_property = {1, Eigen::Matrix3d::Identity()};
    double cable_length =1;
    double step_size = 0.01;

    ptr_joint = std::make_shared<Joint>(Eigen::Vector3d::Zero());

    ptr_UAVCable = std::make_shared<UAVCable>(mav_mass_property, cable_length, ptr_joint, step_size);
}

~rotorTMUAVCableTest(){
}

protected:
    std::shared_ptr<UAVCable> ptr_UAVCable;
    std::shared_ptr<Joint> ptr_joint;
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
    // 
    // // ptr_UAVCable->cable_.GetCableLength(cable_length);

    // const Eigen::Vector3d mav_post{0,0,cable_length};
    // const Eigen::Vector3d mav_post{0,0,ptr_UAVCable->cable_.length()};
    

    ptr_UAVCable->SetMAVInitPostCableTautWithAttachPointPost(Eigen::Vector3d::Zero());

    // 2. set post and vel of payload in vertical direction
    Eigen::Vector3d attachpoint_post{0,0,0};

    // 3. check collision
    ;

    // 4.
    ASSERT_FLOAT_EQ(ptr_UAVCable->mav_.pose().post[0], 0); 
    ASSERT_FLOAT_EQ(ptr_UAVCable->mav_.pose().post[1], 0); 
    ASSERT_FLOAT_EQ(ptr_UAVCable->mav_.pose().post[2], ptr_UAVCable->cable_.length());  
}



// test vertical static condition
TEST_F(rotorTMUAVCableTest, checkVerticalStatic){

    // 1. set post and vel of mav in vertical direction
    ptr_UAVCable->SetMAVInitPostCableTautWithAttachPointPost(Eigen::Vector3d::Zero());

    const Eigen::Vector3d mav_vel{0,0,0};

    ptr_UAVCable->mav_.SetVel(mav_vel);

    // 2. set post and vel of attach point in vertical direction
    const Eigen::Vector3d attachpoint_vel{0,0,0};
    ptr_joint->SetLinearVel(attachpoint_vel);


    // 3. update cable status
    // ptr_UAVCable->UpdateCable();
    ptr_UAVCable->UpdateCable();


    // 4. cable should be slack or there is no collision
    ASSERT_TRUE(!ptr_UAVCable->cable_.tautStatus());
}

// test vertical collision
TEST_F(rotorTMUAVCableTest, checkVerticalCollision){

    // 1. set post and vel of mav in vertical direction
    const Eigen::Vector3d mav_post{0,0,ptr_UAVCable->cable_.length()+RandomGenerate(0,0.1)};
    const Eigen::Vector3d mav_vel{0,0,RandomGenerate(0,0.1)};

    ptr_UAVCable->SetMAVInitPost(mav_post);
    ptr_UAVCable->mav_.SetVel(mav_vel);

    // 2. set vel of attach point in vertical direction
    const Eigen::Vector3d attachpoint_vel{0,0,RandomGenerate(-0.1,0)};
    ptr_joint->SetLinearVel(attachpoint_vel);


    // 3. update cable status
    ptr_UAVCable->UpdateCable();

    // 4. cable should be taut or there is collision
    ASSERT_TRUE(ptr_UAVCable->cable_.tautStatus());
}


// test vertical collision and vel redistribution
TEST_F(rotorTMUAVCableTest, checkVerticalVelDistribution){

    // 1. set post and vel of mav in vertical direction
    const double mav_vel_z = 1;
    const Eigen::Vector3d mav_vel{0,0,mav_vel_z};
    ptr_UAVCable->mav_.SetVel(mav_vel);

    double cable_length = ptr_UAVCable->cable_.length();
    const Eigen::Vector3d mav_post{0,0,ptr_UAVCable->cable_.length()+RandomGenerate(0,0.1)};
    ptr_UAVCable->SetMAVInitPost(mav_post);
    
    // 2. set post and vel of payload in vertical direction
    const double payload_vel_z = -1;
    const Eigen::Vector3d attachpoint_vel{0,0,payload_vel_z};
    const Eigen::Vector3d attachpoint_post{0,0,0};
    

    ptr_joint->SetInitPost(attachpoint_post);
    ptr_joint->SetLinearVel(attachpoint_vel);

    // 3. check collision
    ptr_UAVCable->UpdateCable();

    ASSERT_TRUE(ptr_UAVCable->cable_.tautStatus());

    // 4. compute vels after collision
    ptr_UAVCable->UpdateMAVVelCollided(Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(),  Eigen::Vector3d::Zero());

    // 
    // // ptr_UAVCable->mav_.GetVel(ptr_UAVCable->mav_.vels().linear_vel);

    ASSERT_FLOAT_EQ(ptr_UAVCable->mav_.vels().linear_vel[0], 0); 
    ASSERT_FLOAT_EQ(ptr_UAVCable->mav_.vels().linear_vel[1], 0); 
    ASSERT_FLOAT_EQ(ptr_UAVCable->mav_.vels().linear_vel[2], 0);  
}


// test vertical collision and vel redistribution with impluse-momentum
TEST_F(rotorTMUAVCableTest, checkVerticalVelDistributionWithPayloadMotion){


    // m_q *v_q + m_p * v_p =  m_q *v_q_collided + m_p * v_p_collided
    // we assume collision is inelastic, then v_p_collided = v_q_collided
    // assign values to m_q, m_p, v_p, v_p_collided
    //  v_q_collided = (m_q *v_q + m_p * v_p)/(m_q+m_p)
    // e.g. m_q =1; v_p = 1, v_p_collided = 0.2

    // 1. set post and vel of mav in vertical direction

    // 
    // // ptr_UAVCable->cable_.GetCableLength(cable_length);
    const Eigen::Vector3d mav_post{0,0,ptr_UAVCable->cable_.length()+RandomGenerate(0,0.1)};
    ptr_UAVCable->SetMAVInitPost(mav_post);

    const double mav_vel_z = 1;

    const Eigen::Vector3d mav_vel{0,0,mav_vel_z};
    ptr_UAVCable->mav_.SetVel(mav_vel);


    // 2. set post and vel of payload in vertical direction
    // make payload a negative vel to ensure collision happen 
    const double payload_vel_z = RandomGenerate(-1, 0);
    const Eigen::Vector3d attachpoint_vel{0,0,payload_vel_z};
    const Eigen::Vector3d attachpoint_post{0,0,0};

    ptr_joint->SetInitPost(attachpoint_post);
    ptr_joint->SetLinearVel(attachpoint_vel);    

    // assume payload mass payload vel after collision
    double payload_vel_z_collised = 0.2;

    const Eigen::Vector3d payload_vel_collided{0,0,payload_vel_z_collised};

    // 3. check collision
    // ptr_UAVCable->UpdateCable();
    ptr_UAVCable->UpdateCable();

    ASSERT_TRUE(ptr_UAVCable->cable_.tautStatus());

    // 4. compute vels after collision
    //  ptr_UAVCable->UpdateMAVVelCollided(Eigen::Quaterniond::Identity(), payload_vel_collided,  Eigen::Vector3d::Zero());;

    // // 
    // // Eigen::Vector3d ptr_UAVCable->mav_.vels().linear_vel;

    // // ptr_UAVCable->mav_.GetVel(ptr_UAVCable->mav_.vels().linear_vel);

    // ASSERT_FLOAT_EQ(ptr_UAVCable->mav_.vels().linear_vel[0], 0); 
    // ASSERT_FLOAT_EQ(ptr_UAVCable->mav_.vels().linear_vel[1], 0); 
    // ASSERT_FLOAT_EQ(ptr_UAVCable->mav_.vels().linear_vel[2], 0.2);  
    ptr_UAVCable->UpdateMAVVelCollided(Eigen::Quaterniond::Identity(), payload_vel_collided,  Eigen::Vector3d::Zero());

    // 
    // // ptr_UAVCable->mav_.GetVel(ptr_UAVCable->mav_.vels().linear_vel);

    ASSERT_FLOAT_EQ(ptr_UAVCable->mav_.vels().linear_vel[0], 0); 
    ASSERT_FLOAT_EQ(ptr_UAVCable->mav_.vels().linear_vel[1], 0); 
    ASSERT_FLOAT_EQ(ptr_UAVCable->mav_.vels().linear_vel[2], 0.2);  

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

    
    // ptr_UAVCable->cable_.GetCableLength(cable_length);

    const Eigen::Vector3d mav_post{0,0,ptr_UAVCable->cable_.length()+RandomGenerate(0,0.1)};
    ptr_UAVCable->SetMAVInitPost(mav_post);
    
    // 2. set post and vel of payload in vertical direction
    // make payload a negative vel to ensure collision happen
    const Eigen::Vector3d attachpoint_post{0,0,0}; 
    const double payload_vel_z = RandomGenerate(-1, 0);

    const Eigen::Vector3d attachpoint_vel{0,0,payload_vel_z};

    ptr_joint->SetInitPost(attachpoint_post);
    ptr_joint->SetLinearVel(attachpoint_vel);       
    
    // assume payload mass payload vel after collision
    double payload_vel_z_collised = RandomGenerate(0,4);

    const Eigen::Vector3d payload_vel_collided{0,0,payload_vel_z_collised};

    // 3. check collision
    ptr_UAVCable->UpdateCable();

    ASSERT_TRUE(ptr_UAVCable->cable_.tautStatus());

    // 4. compute vels after collision
    ptr_UAVCable->UpdateMAVVelCollided(Eigen::Quaterniond::Identity(), payload_vel_collided,  Eigen::Vector3d::Zero());

    // 
    // // Eigen::Vector3d ptr_UAVCable->mav_.vels().linear_vel;

    // // ptr_UAVCable->mav_.GetVel(ptr_UAVCable->mav_.vels().linear_vel);

    ASSERT_FLOAT_EQ(ptr_UAVCable->mav_.vels().linear_vel[0], 0); 
    ASSERT_FLOAT_EQ(ptr_UAVCable->mav_.vels().linear_vel[1], 0); 
    ASSERT_FLOAT_EQ(ptr_UAVCable->mav_.vels().linear_vel[2], payload_vel_z_collised);  
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

    
    // ptr_UAVCable->cable_.GetCableLength(cable_length);
    const Eigen::Vector3d mav_post{0,0,ptr_UAVCable->cable_.length()+RandomGenerate(0,0.1)};
    ptr_UAVCable->SetMAVInitPost(mav_post);
    
    // 2. set post and vel of payload in vertical direction
    // make payload a negative vel to ensure collision happen 
    const double payload_vel_z = RandomGenerate(-0.5, 0);

    const Eigen::Vector3d attachpoint_vel{0,0,payload_vel_z};
    const Eigen::Vector3d attachpoint_post{0,0,0};
    
    ptr_joint->SetInitPost(attachpoint_post);
    ptr_joint->SetLinearVel(attachpoint_vel); 

    // assume payload mass payload vel after collision
    double payload_vel_z_collised = 0.2;

    const Eigen::Vector3d payload_vel_collided{0,0,payload_vel_z_collised};

    // 3. check collision
    ptr_UAVCable->UpdateCable();

    ASSERT_TRUE(ptr_UAVCable->cable_.tautStatus());

    // 4. compute vels after collision
    ptr_UAVCable->UpdateMAVVelCollided(Eigen::Quaterniond::Identity(), payload_vel_collided,  Eigen::Vector3d::Zero());

    // 
    // // Eigen::Vector3d ptr_UAVCable->mav_.vels().linear_vel;

    // // ptr_UAVCable->mav_.GetVel(ptr_UAVCable->mav_.vels().linear_vel);

    // ASSERT_FLOAT_EQ(ptr_UAVCable->mav_.vels().linear_vel[0], 1); 
    // ASSERT_FLOAT_EQ(ptr_UAVCable->mav_.vels().linear_vel[1], 0); 
    // ASSERT_FLOAT_EQ(ptr_UAVCable->mav_.vels().linear_vel[2], 0.2);  

    ASSERT_FLOAT_EQ(ptr_UAVCable->mav_.vels().linear_vel[0], 1); 
    ASSERT_FLOAT_EQ(ptr_UAVCable->mav_.vels().linear_vel[1], 0); 
    ASSERT_FLOAT_EQ(ptr_UAVCable->mav_.vels().linear_vel[2], 0.2);  
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

    
    // ptr_UAVCable->cable_.GetCableLength(cable_length);
    const Eigen::Vector3d mav_post{0,0,ptr_UAVCable->cable_.length()+RandomGenerate(0,0.1)};
    ptr_UAVCable->SetMAVInitPost(mav_post);
    
    // 2. set post and vel of payload in vertical direction
    // make payload a negative vel to ensure collision happen 
    const double payload_vel_z = RandomGenerate(-1, 0);

    const Eigen::Vector3d attachpoint_vel{0,0,payload_vel_z};
    const Eigen::Vector3d attachpoint_post{0,0,0};
    
    ptr_joint->SetInitPost(attachpoint_post);
    ptr_joint->SetLinearVel(attachpoint_vel); 

    // assume payload mass payload vel after collision
    double payload_vel_z_collised = RandomGenerate(0,2);

    const Eigen::Vector3d payload_vel_collided{0,0,payload_vel_z_collised};

    // 3. check collision
    ptr_UAVCable->UpdateCable();

    ASSERT_TRUE(ptr_UAVCable->cable_.tautStatus());

    // 4. compute vels after collision
    ptr_UAVCable->UpdateMAVVelCollided(Eigen::Quaterniond::Identity(), payload_vel_collided,  Eigen::Vector3d::Zero());

    // 
    // Eigen::Vector3d ptr_UAVCable->mav_.vels().linear_vel;

    // ptr_UAVCable->mav_.GetVel(ptr_UAVCable->mav_.vels().linear_vel);

    ASSERT_FLOAT_EQ(ptr_UAVCable->mav_.vels().linear_vel[0], mav_vel_x); 
    ASSERT_FLOAT_EQ(ptr_UAVCable->mav_.vels().linear_vel[1], 0); 
    ASSERT_FLOAT_EQ(ptr_UAVCable->mav_.vels().linear_vel[2], payload_vel_z_collised);  
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

    
    // ptr_UAVCable->cable_.GetCableLength(cable_length);
    const Eigen::Vector3d mav_post{0,0,ptr_UAVCable->cable_.length()+RandomGenerate(0,0.1)};
    ptr_UAVCable->SetMAVInitPost(mav_post);
    
    // 2. set post and vel of payload in vertical direction
    // make payload a negative vel to ensure collision happen 
    const double payload_vel_z = RandomGenerate(-1, 0);

    const Eigen::Vector3d attachpoint_vel{0,0,payload_vel_z};
    const Eigen::Vector3d attachpoint_post{0,0,0};

    ptr_joint->SetInitPost(attachpoint_post);
    ptr_joint->SetLinearVel(attachpoint_vel);     
    
    // assume payload mass payload vel after collision
    double payload_mass = 1;
    double payload_vel_z_collised = 0.2;

    const Eigen::Vector3d payload_vel_collided{0,0,payload_vel_z_collised};

    // 3. check collision
    ptr_UAVCable->UpdateCable();

    ASSERT_TRUE(ptr_UAVCable->cable_.tautStatus());

    // 4. compute vels after collision
    ptr_UAVCable->UpdateMAVVelCollided(Eigen::Quaterniond::Identity(), payload_vel_collided,  Eigen::Vector3d::Zero());

    // 
    // Eigen::Vector3d ptr_UAVCable->mav_.vels().linear_vel;

    // ptr_UAVCable->mav_.GetVel(ptr_UAVCable->mav_.vels().linear_vel);

    ASSERT_FLOAT_EQ(ptr_UAVCable->mav_.vels().linear_vel[0], 0); 
    ASSERT_FLOAT_EQ(ptr_UAVCable->mav_.vels().linear_vel[1], 1); 
    ASSERT_FLOAT_EQ(ptr_UAVCable->mav_.vels().linear_vel[2], 0.2);  
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

    
    // ptr_UAVCable->cable_.GetCableLength(cable_length);
    const Eigen::Vector3d mav_post{0,0,ptr_UAVCable->cable_.length()+RandomGenerate(0,0.1)};
    ptr_UAVCable->SetMAVInitPost(mav_post);
    
    // 2. set post and vel of payload in vertical direction
    // make payload a negative vel to ensure collision happen 
    const double payload_vel_z = RandomGenerate(-1, 0);

    const Eigen::Vector3d attachpoint_vel{0,0,payload_vel_z};
    const Eigen::Vector3d attachpoint_post{0,0,0};
    
    ptr_joint->SetInitPost(attachpoint_post);
    ptr_joint->SetLinearVel(attachpoint_vel); 

    // assume payload mass payload vel after collision
    double payload_mass = 1;
    double payload_vel_z_collised = RandomGenerate(0,6);

    const Eigen::Vector3d payload_vel_collided{0,0,payload_vel_z_collised};

    // 3. check collision
    ptr_UAVCable->UpdateCable();

    ASSERT_TRUE(ptr_UAVCable->cable_.tautStatus());

    // 4. compute vels after collision
    ptr_UAVCable->UpdateMAVVelCollided(Eigen::Quaterniond::Identity(), payload_vel_collided,  Eigen::Vector3d::Zero());;

    // 
    // Eigen::Vector3d ptr_UAVCable->mav_.vels().linear_vel;

    // ptr_UAVCable->mav_.GetVel(ptr_UAVCable->mav_.vels().linear_vel);

    ASSERT_FLOAT_EQ(ptr_UAVCable->mav_.vels().linear_vel[0], 0); 
    ASSERT_FLOAT_EQ(ptr_UAVCable->mav_.vels().linear_vel[1], mav_vel_y); 
    ASSERT_FLOAT_EQ(ptr_UAVCable->mav_.vels().linear_vel[2], payload_vel_z_collised);  
}




// /*****************************Dynamic test****************************/

TEST_F(rotorTMUAVCableTest, checkVerticalStaticHovering){
        // 1. set posts of mav and payload to remain cable taut
        const double mav_vel_z = 0;
        const Eigen::Vector3d mav_vel{0,0,mav_vel_z};
        ptr_UAVCable->mav_.SetVel(mav_vel);

        
        // ptr_UAVCable->cable_.GetCableLength(cable_length);
        const Eigen::Vector3d mav_post{0,0,ptr_UAVCable->cable_.length()+0.1};
        ptr_UAVCable->SetMAVInitPost(mav_post);
        
        // 2. set post and vel of payload in vertical direction
        const double payload_vel_z = 0;
        const Eigen::Vector3d attachpoint_vel{0,0,payload_vel_z};
        const Eigen::Vector3d attachpoint_post{0,0,0};

        ptr_joint->SetInitPost(attachpoint_post);
        ptr_joint->SetLinearVel(attachpoint_vel); 

        // 4. set control input for quadrotor
        // given that m_q = 1kg and assume m_p = 1kg
        // apply a thrust force 2*9.8N = 19.6N
        // acc = (19.6 - 2*9.8)/2 = 0
        // take payload a an object
        // -t - 1 * 9.8 = 0 ===> -t = 9.8N
        const double mav_thrust = 9.8*2;
        // std::cout<<"[----------] mav_thrust is " << mav_thrust<<std::endl;
        
        ptr_UAVCable->InputControllerInput(mav_thrust,Eigen::Vector3d::Zero());

        Eigen::Vector3d attach_point_acc{0,0,9.8};
        ptr_joint->SetLinearAcc(attach_point_acc);

        // ptr_UAVCable->ComputeNetWrenchApplied2MAV(attach_point_acc);

        // updae cable 
        ptr_UAVCable->UpdateCable();

        ASSERT_TRUE(ptr_UAVCable->cable_.tautStatus());

        

        // 5. compute wrenches at attach point when whole system is at static equilibrium
        // ComputeInteractionWrenches(const Eigen::Vector3d &attach_point_post, const Eigen::Vector3d &attach_point_vel, const Eigen::Quaterniond &payload_attitude, Eigen::Vector3d &payload_bodyrate)
        Eigen::Vector3d zeroVector = Eigen::Vector3d::Zero();
        Eigen::Quaterniond identityQuat = Eigen::Quaterniond::Identity();

        // ComputeInteractionWrenches(const Eigen::Vector3d &attach_point_post, const Eigen::Vector3d &attach_point_vel, const Eigen::Quaterniond &payload_attitude, Eigen::Vector3d &payload_bodyrate)
        ptr_UAVCable->ComputeInteractionWrenches(identityQuat, zeroVector);

        // 6. one step dynamic simulation
        ptr_UAVCable->DoOneStepInt();

        // 7. obtain quadrotor acc, vel, post
        // check position = 0
        //Eigen::Vector3d ptr_UAVCable->mav_.pose().post = Eigen::Vector3d::Random();

        // ptr_UAVCable->mav_.GetPosition(ptr_UAVCable->mav_.pose().post);

        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.pose().post[0], mav_post[0]); 
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.pose().post[1], mav_post[1]); 
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.pose().post[2], mav_post[2]);   

        // check vel = 0
        // Eigen::Vector3d vel = Eigen::Vector3d::Random();
        //ptr_UAVCable->mav_.GetVel(vel);

        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.vels().linear_vel[0], mav_vel[0]); 
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.vels().linear_vel[1], mav_vel[1]); 
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.vels().linear_vel[2], mav_vel[2]);  

        // check acc = 0
        // Eigen::Vector3d acc = Eigen::Vector3d::Random();
        //ptr_UAVCable->mav_.GetAcc(acc);
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.accs().linear_acc[0], 0); 
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.accs().linear_acc[1], 0); 
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.accs().linear_acc[2], 0);  

        // check bodyrate
        // check attitude
        //Eigen::Quaterniond att = Eigen::Quaterniond::UnitRandom();
        //ptr_UAVCable->mav_.GetAttitude(att);
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.pose().att.x(), 0);    
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.pose().att.y(), 0);  
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.pose().att.z(), 0);  
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.pose().att.w(), 1);  


        //Eigen::Vector3d bodyrate = Eigen::Vector3d::Random();
        //ptr_UAVCable->mav_.GetBodyrate(bodyrate);
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.vels().bodyrate[0], 0); 
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.vels().bodyrate[1], 0); 
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.vels().bodyrate[2], 0);  

        // check bodyrate_acc
        //Eigen::Vector3d bodyrate_acc = Eigen::Vector3d::Random();
        //ptr_UAVCable->mav_.GetBodyRateAcc(bodyrate_acc);
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.accs().angular_acc[0], 0); 
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.accs().angular_acc[1], 0); 
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.accs().angular_acc[2], 0);  
}







TEST_F(rotorTMUAVCableTest, checkVerticalStaticHovering100Steps){
        // 1. set posts of mav and payload to remain cable taut
        const double mav_vel_z = 0;
        const Eigen::Vector3d mav_vel{0,0,mav_vel_z};
        ptr_UAVCable->mav_.SetVel(mav_vel);

        
        const Eigen::Vector3d mav_post{0,0,ptr_UAVCable->cable_.length()+0.1};
        ptr_UAVCable->SetMAVInitPost(mav_post);
        
        // 2. set post and vel of payload in vertical direction
        const double payload_vel_z = 0;
        const Eigen::Vector3d attachpoint_vel{0,0,payload_vel_z};
        const Eigen::Vector3d attachpoint_post{0,0,0};

        ptr_joint->SetInitPost(attachpoint_post);
        ptr_joint->SetLinearVel(attachpoint_vel); 

        // 4. set control input for quadrotor
        // given that m_q = 1kg and assume m_p = 1kg
        // apply a thrust force 2*9.8N = 19.6N
        // acc = (19.6 - 2*9.8)/2 = 0
        // take payload a an object
        // -t - 1 * 9.8 = 0 ===> -t = 9.8N
        const double mav_thrust = 9.8*2;
        // std::cout<<"[----------] mav_thrust is " << mav_thrust<<std::endl;
        
        ptr_UAVCable->InputControllerInput(mav_thrust,Eigen::Vector3d::Zero());

        Eigen::Vector3d attach_point_acc{0,0,9.8};
        ptr_joint->SetLinearAcc(attach_point_acc);

        // ptr_UAVCable->ComputeNetWrenchApplied2MAV(attach_point_acc);

        // updae cable 
        ptr_UAVCable->UpdateCable();

        ASSERT_TRUE(ptr_UAVCable->cable_.tautStatus());

        

        // 5. compute wrenches at attach point when whole system is at static equilibrium
        // ComputeInteractionWrenches(const Eigen::Vector3d &attach_point_post, const Eigen::Vector3d &attach_point_vel, const Eigen::Quaterniond &payload_attitude, Eigen::Vector3d &payload_bodyrate)
        Eigen::Vector3d zeroVector = Eigen::Vector3d::Zero();
        Eigen::Quaterniond identityQuat = Eigen::Quaterniond::Identity();

        // ComputeInteractionWrenches(const Eigen::Vector3d &attach_point_post, const Eigen::Vector3d &attach_point_vel, const Eigen::Quaterniond &payload_attitude, Eigen::Vector3d &payload_bodyrate)
        ptr_UAVCable->ComputeInteractionWrenches(identityQuat, zeroVector);

        // 6. 100 step dynamic simulation
        const double dt = 0.01;
        for(double t=dt ; t<=100*dt ; t+= dt)
        {
                ptr_UAVCable->DoOneStepInt();;
                // printf("current step is %.3f \n", t);
        }

        // 7. obtain quadrotor acc, vel, post
        // check position = 0
        //Eigen::Vector3d ptr_UAVCable->mav_.pose().post = Eigen::Vector3d::Random();

        // ptr_UAVCable->mav_.GetPosition(ptr_UAVCable->mav_.pose().post);

        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.pose().post[0], mav_post[0]); 
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.pose().post[1], mav_post[1]); 
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.pose().post[2], mav_post[2]);   

        // check vel = 0
        // Eigen::Vector3d vel = Eigen::Vector3d::Random();
        //ptr_UAVCable->mav_.GetVel(vel);

        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.vels().linear_vel[0], mav_vel[0]); 
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.vels().linear_vel[1], mav_vel[1]); 
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.vels().linear_vel[2], mav_vel[2]);  

        // check acc = 0
        // Eigen::Vector3d acc = Eigen::Vector3d::Random();
        //ptr_UAVCable->mav_.GetAcc(acc);
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.accs().linear_acc[0], 0); 
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.accs().linear_acc[1], 0); 
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.accs().linear_acc[2], 0);  

        // check bodyrate
        // check attitude
        //Eigen::Quaterniond att = Eigen::Quaterniond::UnitRandom();
        //ptr_UAVCable->mav_.GetAttitude(att);
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.pose().att.x(), 0);    
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.pose().att.y(), 0);  
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.pose().att.z(), 0);  
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.pose().att.w(), 1);  


        //Eigen::Vector3d bodyrate = Eigen::Vector3d::Random();
        //ptr_UAVCable->mav_.GetBodyrate(bodyrate);
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.vels().bodyrate[0], 0); 
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.vels().bodyrate[1], 0); 
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.vels().bodyrate[2], 0);  

        // check bodyrate_acc
        //Eigen::Vector3d bodyrate_acc = Eigen::Vector3d::Random();
        //ptr_UAVCable->mav_.GetBodyRateAcc(bodyrate_acc);
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.accs().angular_acc[0], 0); 
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.accs().angular_acc[1], 0); 
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.accs().angular_acc[2], 0);  
}






TEST_F(rotorTMUAVCableTest, checkVerticalConstAcc){
        // 1. set posts of mav and payload to remain cable taut
        const double mav_vel_z = 0;
        const Eigen::Vector3d mav_init_vel{0,0,mav_vel_z};
        ptr_UAVCable->mav_.SetVel(mav_init_vel);

        
        const Eigen::Vector3d mav_init_post{0,0,ptr_UAVCable->cable_.length()+0.1};
        ptr_UAVCable->SetMAVInitPost(mav_init_post);
        
        // 2. set post and vel of payload in vertical direction
        const double payload_vel_z = 0;
        const Eigen::Vector3d attachpoint_vel{0,0,payload_vel_z};
        const Eigen::Vector3d attachpoint_post{0,0,0};

        ptr_joint->SetInitPost(attachpoint_post);
        ptr_joint->SetLinearVel(attachpoint_vel); 


        // 4. set control input for quadrotor
        // given that m_q = 1kg and assume m_p = 1kg
        // apply a thrust force 3*9.8N = 29.4N
        // acc = (29.4 - 2*9.8)/2 = 4.9
        // take payload as an object
        // -t - 1 * 9.8 = 4.9 ===> -t = 14.7N
        const double mav_thrust = 9.8*3;        
        ptr_UAVCable->InputControllerInput(mav_thrust,Eigen::Vector3d::Zero());

        // payload's "linear acc" is 9.8+4.9
        Eigen::Vector3d attach_point_acc{0,0,9.8+4.9};
        ptr_joint->SetLinearAcc(attach_point_acc);


        // update cable status
        ptr_UAVCable->UpdateCable();

        ASSERT_TRUE(ptr_UAVCable->cable_.tautStatus());

        // 5. compute wrenches at attach point when whole system is at static equilibrium
        Eigen::Vector3d zeroVector = Eigen::Vector3d::Zero();
        Eigen::Quaterniond identityQuat = Eigen::Quaterniond::Identity();

        ptr_UAVCable->ComputeInteractionWrenches(identityQuat, zeroVector);

        // 6. 1 step dynamic simulation
        ptr_UAVCable->DoOneStepInt();;

        
        // 7. obtain quadrotor acc, vel, post
        // check position = 0.5* a*t^2

        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.pose().post[0], mav_init_post[0]); 
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.pose().post[1], mav_init_post[1]); 
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.pose().post[2], mav_init_post[2] + 0.5 * 4.9 * pow(0.01, 2));   

        // check vel = at
        // Eigen::Vector3d vel = Eigen::Vector3d::Random();
        //ptr_UAVCable->mav_.GetVel(vel);

        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.vels().linear_vel[0], mav_init_vel[0]); 
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.vels().linear_vel[1], mav_init_vel[1]); 
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.vels().linear_vel[2], mav_init_vel[2] + 4.9 * 0.01);  

        // check acc = a
        // Eigen::Vector3d acc = Eigen::Vector3d::Random();
        //ptr_UAVCable->mav_.GetAcc(acc);
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.accs().linear_acc[0], 0); 
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.accs().linear_acc[1], 0); 
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.accs().linear_acc[2], 4.9);  

        // check bodyrate
        //Eigen::Vector3d bodyrate = Eigen::Vector3d::Random();
        //ptr_UAVCable->mav_.GetBodyrate(bodyrate);
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.vels().bodyrate[0], 0); 
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.vels().bodyrate[1], 0); 
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.vels().bodyrate[2], 0);  

        // check attitude
        //Eigen::Quaterniond att = Eigen::Quaterniond::UnitRandom();
        //ptr_UAVCable->mav_.GetAttitude(att);
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.pose().att.x(), 0);    
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.pose().att.y(), 0);  
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.pose().att.z(), 0);  
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.pose().att.w(), 1);  

        // check bodyrate_acc
        //Eigen::Vector3d bodyrate_acc = Eigen::Vector3d::Random();
        //ptr_UAVCable->mav_.GetBodyRateAcc(bodyrate_acc);
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.accs().angular_acc[0], 0); 
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.accs().angular_acc[1], 0); 
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.accs().angular_acc[2], 0);  
}





TEST_F(rotorTMUAVCableTest, checkVerticalConstAccHundSteps){
        // 1. set posts of mav and payload to make collision
        const double mav_vel_z = 0;
        const Eigen::Vector3d mav_init_vel{0,0,mav_vel_z};
        ptr_UAVCable->mav_.SetVel(mav_init_vel);

        
        // ptr_UAVCable->cable_.GetCableLength(cable_length);
        const Eigen::Vector3d mav_init_post{0,0,ptr_UAVCable->cable_.length()+0.1};
        ptr_UAVCable->SetMAVInitPost(mav_init_post);
        
        // 2. set post and vel of payload in vertical direction
        const double payload_vel_z = 0;
        const Eigen::Vector3d attachpoint_init_vel{0,0,payload_vel_z};
        const Eigen::Vector3d attachpoint_init_post{0,0,0};

        ptr_joint->SetInitPost(attachpoint_init_post);
        ptr_joint->SetLinearVel(attachpoint_init_vel); 

        // 4. set control input for quadrotor
        // given that m_q = 1kg and assume m_p = 1kg
        // apply a thrust force 3*9.8N = 29.4N
        // acc = (29.4 - 2*9.8)/2 = 4.9
        // take payload a an object
        // -t - 1 * 9.8 = 4.9 ===> -t = 14.7N
        const double mav_thrust = 9.8*3;
        // std::cout<<"[----------] mav_thrust is " << mav_thrust<<std::endl;
        
        ptr_UAVCable->InputControllerInput(mav_thrust,Eigen::Vector3d::Zero());

        Eigen::Vector3d attach_point_acc{0,0,9.8+4.9};
        ptr_joint->SetLinearAcc(attach_point_acc);


        // update cable status
        ptr_UAVCable->UpdateCable();

        ASSERT_TRUE(ptr_UAVCable->cable_.tautStatus());        


        // 6. dynamic simulation
        const double dt = 0.01;
        for(double t=dt ; t - 1e-10<=100*dt ; t+= dt)
        {
                // 5. compute wrenches at attach point 
                Eigen::Vector3d zeroVector = Eigen::Vector3d::Zero();
                Eigen::Quaterniond identityQuat = Eigen::Quaterniond::Identity();            
                ptr_UAVCable->ComputeInteractionWrenches(identityQuat, zeroVector);

                ptr_UAVCable->DoOneStepInt();;
                // printf("current step is %.3f \n", t);
        }


        // 7. obtain quadrotor acc, vel, post
        // check position = 0.5* a*t^2
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.pose().post[0], mav_init_post[0]); 
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.pose().post[1], mav_init_post[1]); 
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.pose().post[2], mav_init_post[2] + 0.5 * 4.9 * pow(0.01*100, 2));   

        // std::cout<<"[----------] mav z translatio is " << mav_init_post[2] + 0.5 * 4.9 * pow(0.01*1000, 2) <<std::endl;

        // check vel = at
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.vels().linear_vel[0], mav_init_vel[0]); 
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.vels().linear_vel[1], mav_init_vel[1]); 
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.vels().linear_vel[2], mav_init_vel[2] + 4.9 * 0.01*100);  

        // check acc = a
        // Eigen::Vector3d acc = Eigen::Vector3d::Random();
        //ptr_UAVCable->mav_.GetAcc(acc);
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.accs().linear_acc[0], 0); 
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.accs().linear_acc[1], 0); 
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.accs().linear_acc[2], 4.9);  

        // check bodyrate
        //Eigen::Vector3d bodyrate = Eigen::Vector3d::Random();
        //ptr_UAVCable->mav_.GetBodyrate(bodyrate);
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.vels().bodyrate[0], 0); 
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.vels().bodyrate[1], 0); 
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.vels().bodyrate[2], 0);  

        // check attitude
        //Eigen::Quaterniond att = Eigen::Quaterniond::UnitRandom();
        //ptr_UAVCable->mav_.GetAttitude(att);
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.pose().att.x(), 0);    
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.pose().att.y(), 0);  
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.pose().att.z(), 0);  
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.pose().att.w(), 1);  

        // check bodyrate_acc
        //Eigen::Vector3d bodyrate_acc = Eigen::Vector3d::Random();
        //ptr_UAVCable->mav_.GetBodyRateAcc(bodyrate_acc);
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.accs().angular_acc[0], 0); 
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.accs().angular_acc[1], 0); 
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.accs().angular_acc[2], 0);  
}

TEST_F(rotorTMUAVCableTest, checkVerticalConstAccThousandSteps){
             // 1. set posts of mav and payload to make collision
        const double mav_vel_z = 0;
        const Eigen::Vector3d mav_init_vel{0,0,mav_vel_z};
        ptr_UAVCable->mav_.SetVel(mav_init_vel);

        
        // ptr_UAVCable->cable_.GetCableLength(cable_length);
        const Eigen::Vector3d mav_init_post{0,0,ptr_UAVCable->cable_.length()+0.1};
        ptr_UAVCable->SetMAVInitPost(mav_init_post);
        
        // 2. set post and vel of payload in vertical direction
        const double payload_vel_z = 0;
        const Eigen::Vector3d attachpoint_init_vel{0,0,payload_vel_z};
        const Eigen::Vector3d attachpoint_init_post{0,0,0};

        ptr_joint->SetInitPost(attachpoint_init_post);
        ptr_joint->SetLinearVel(attachpoint_init_vel); 

        // 4. set control input for quadrotor
        // given that m_q = 1kg and assume m_p = 1kg
        // apply a thrust force 3*9.8N = 29.4N
        // acc = (29.4 - 2*9.8)/2 = 4.9
        // take payload a an object
        // -t - 1 * 9.8 = 4.9 ===> -t = 14.7N
        const double mav_thrust = 9.8*3;
        // std::cout<<"[----------] mav_thrust is " << mav_thrust<<std::endl;
        
        ptr_UAVCable->InputControllerInput(mav_thrust,Eigen::Vector3d::Zero());

        Eigen::Vector3d attach_point_acc{0,0,9.8+4.9};
        ptr_joint->SetLinearAcc(attach_point_acc);


        // update cable status
        ptr_UAVCable->UpdateCable();

        ASSERT_TRUE(ptr_UAVCable->cable_.tautStatus());        


        // 6. dynamic simulation
        const double dt = 0.01;
        for(double t=dt ; t - 1e-10<=1000*dt ; t+= dt)
        {
                // 5. compute wrenches at attach point 
                Eigen::Vector3d zeroVector = Eigen::Vector3d::Zero();
                Eigen::Quaterniond identityQuat = Eigen::Quaterniond::Identity();            
                ptr_UAVCable->ComputeInteractionWrenches(identityQuat, zeroVector);

                ptr_UAVCable->DoOneStepInt();;
                // printf("current step is %.3f \n", t);
        }
        // 7. obtain quadrotor acc, vel, post
        // check position = 0.5* a*t^2
        //Eigen::Vector3d ptr_UAVCable->mav_.pose().post = Eigen::Vector3d::Random();

        // ptr_UAVCable->mav_.GetPosition(ptr_UAVCable->mav_.pose().post);

        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.pose().post[0], mav_init_post[0]); 
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.pose().post[1], mav_init_post[1]); 
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.pose().post[2], mav_init_post[2] + 0.5 * 4.9 * pow(0.01*1000, 2));   

        // std::cout<<"[----------] mav z translatio is " << mav_init_post[2] + 0.5 * 4.9 * pow(0.01*1000, 2) <<std::endl;

        // check vel = at
        // Eigen::Vector3d vel = Eigen::Vector3d::Random();
        //ptr_UAVCable->mav_.GetVel(vel);

        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.vels().linear_vel[0], mav_init_vel[0]); 
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.vels().linear_vel[1], mav_init_vel[1]); 
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.vels().linear_vel[2], mav_init_vel[2] + 4.9 * 0.01*1000);  

        // check acc = a
        // Eigen::Vector3d acc = Eigen::Vector3d::Random();
        //ptr_UAVCable->mav_.GetAcc(acc);
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.accs().linear_acc[0], 0); 
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.accs().linear_acc[1], 0); 
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.accs().linear_acc[2], 4.9);  

        // check bodyrate
        //Eigen::Vector3d bodyrate = Eigen::Vector3d::Random();
        //ptr_UAVCable->mav_.GetBodyrate(bodyrate);
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.vels().bodyrate[0], 0); 
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.vels().bodyrate[1], 0); 
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.vels().bodyrate[2], 0);  

        // check attitude
        //Eigen::Quaterniond att = Eigen::Quaterniond::UnitRandom();
        //ptr_UAVCable->mav_.GetAttitude(att);
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.pose().att.x(), 0);    
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.pose().att.y(), 0);  
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.pose().att.z(), 0);  
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.pose().att.w(), 1);  

        // check bodyrate_acc
        //Eigen::Vector3d bodyrate_acc = Eigen::Vector3d::Random();
        //ptr_UAVCable->mav_.GetBodyRateAcc(bodyrate_acc);
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.accs().angular_acc[0], 0); 
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.accs().angular_acc[1], 0); 
        EXPECT_FLOAT_EQ(ptr_UAVCable->mav_.accs().angular_acc[2], 0);  
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