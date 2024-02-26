#include <iostream> 
#include <gtest/gtest.h>
#include <memory>
#include <cmath>
#include <cstdlib>
#include <random>
#include "rotor_tm_sim/lib_cooperative.hpp"

double RandomGenerate(const double &minValue, const double &maxValue);

Eigen::Vector3d RandomUnitVector3d();


class rotorTMCooperative : public ::testing::Test
{
public:

rotorTMCooperative(){
    double mav_mass =1;
    double payload_mass = 1.5;
    Eigen::Matrix3d mav_inertia = Eigen::Matrix3d::Identity(3,3);
    Eigen::Matrix3d payload_inertia = Eigen::Matrix3d::Identity(3,3);
    double cable_length =1;
    double step_size = 0.01;
    std::vector<Eigen::Vector3d> v_attach_point_post{{1,0,0},{0,1,0}, {0,0,0}};


    ptr_Cooperative = std::make_shared<Cooperative>(v_attach_point_post, mav_mass, mav_inertia, cable_length, payload_mass, payload_inertia, step_size);
}

~rotorTMCooperative(){
}

protected:
    std::shared_ptr<Cooperative> ptr_Cooperative;
};

// test if gTest is well integrated
TEST_F(rotorTMCooperative, checkGTest){
    ASSERT_TRUE(true);
}

// test if instance is created
TEST_F(rotorTMCooperative, checkInstanceClass){
    ASSERT_TRUE(ptr_Cooperative!=nullptr);
}

// test if initial posts are set for mavs
// test if instance is created
TEST_F(rotorTMCooperative, checkMAVInitialPosts){
    ptr_Cooperative->SetPayloadInitPost();

    Eigen::Vector3d mav0_init_post{0,0,0};
    Eigen::Vector3d mav1_init_post{0,0,0};
    Eigen::Vector3d mav2_init_post{0,0,0};

    // std::cout<< "fuck point cooperative test 1"<<std::endl;
    std::vector<UAVCable> v_drone_cable = ptr_Cooperative->v_drone_cable_;

    // UAVCable mavcable1 = v_drone_cable[0];
    // mavcable1.mav_.GetPosition(mav0_init_post); // check at(0).mav_ if it gets correct values passed in
    
    v_drone_cable.at(0).mav_.GetPosition(mav0_init_post);
    v_drone_cable.at(1).mav_.GetPosition(mav1_init_post);
    v_drone_cable.at(2).mav_.GetPosition(mav2_init_post);        

    double cable_length;
    ptr_Cooperative->v_drone_cable_.at(0).cable_.GetCableLength(cable_length);

    // std::cout<<"[----------] test: mav0_init_post  is " << mav0_init_post.transpose()<<std::endl;
    EXPECT_FLOAT_EQ(mav0_init_post[0], 1); 
    EXPECT_FLOAT_EQ(mav0_init_post[1], 0); 
    EXPECT_FLOAT_EQ(mav0_init_post[2], cable_length);  

    // std::cout<<"[----------] test: mav1_init_post  is " << mav1_init_post.transpose()<<std::endl;
    EXPECT_FLOAT_EQ(mav1_init_post[0], 0); 
    EXPECT_FLOAT_EQ(mav1_init_post[1], 1); 
    EXPECT_FLOAT_EQ(mav1_init_post[2], cable_length);  

    // std::cout<<"[----------] test: mav2_init_post  is " << mav2_init_post.transpose()<<std::endl;
    EXPECT_FLOAT_EQ(mav2_init_post[0], 0); 
    EXPECT_FLOAT_EQ(mav2_init_post[1], 0); 
    EXPECT_FLOAT_EQ(mav2_init_post[2], cable_length);          
}




// test if initial posts are set for mavs
// test if instance is created
TEST_F(rotorTMCooperative, checkVerticalStaticEquilibrium){

    // set initial posts for mavs and payload
    ptr_Cooperative->SetPayloadInitPost();



    // input mav controllers' inputs to hover
    // 3 mavs + 1 payload =  3 + 1.5 = 4.5
    // mav thrust = (4.5 * 9.8)/3 = 14.7N

    Eigen::VectorXd v_mavs_thrusts = Eigen::MatrixXd::Constant(3,1,14.7);
    std::vector<Eigen::Vector3d> v_mavs_torques(3, Eigen::Vector3d::Zero());

    std::cout<< "fuck point cooperative test 1"<<std::endl;
    ptr_Cooperative->InputControllerInput4MAVs(v_mavs_thrusts, v_mavs_torques);

    // compute interation wrenches and vars for MAVs and payload
    std::cout<< "fuck point cooperative test 2"<<std::endl;
    ptr_Cooperative->ComputeInteractWrenches();

    // call one step dynamic simulation for MAVs and payload
    std::cout<< "fuck point cooperative test 3"<<std::endl;
    ptr_Cooperative->DoOneStepInt4Robots();

    // 
    Eigen::Vector3d mav0_init_post{0,0,0};
    Eigen::Vector3d mav1_init_post{0,0,0};
    Eigen::Vector3d mav2_init_post{0,0,0};

    std::cout<< "fuck point cooperative test 4"<<std::endl;
    std::vector<UAVCable> v_drone_cable = ptr_Cooperative->v_drone_cable_;

    v_drone_cable.at(0).mav_.GetPosition(mav0_init_post);
    v_drone_cable.at(1).mav_.GetPosition(mav1_init_post);
    v_drone_cable.at(2).mav_.GetPosition(mav2_init_post);        
    double cable_length;
    ptr_Cooperative->v_drone_cable_.at(0).cable_.GetCableLength(cable_length);

    std::cout<<"[----------] test: mav1_init_post  is " << mav1_init_post.transpose()<<std::endl;
    EXPECT_FLOAT_EQ(mav1_init_post[0], 0); 
    EXPECT_FLOAT_EQ(mav1_init_post[1], 1); 
    EXPECT_FLOAT_EQ(mav1_init_post[2], cable_length);  

    // vel of mav1
    Eigen::Vector3d mav1_vel;
    v_drone_cable.at(1).mav_.GetVel(mav1_vel);    
    EXPECT_FLOAT_EQ(mav1_vel[0], 0); 
    EXPECT_FLOAT_EQ(mav1_vel[1], 0); 
    EXPECT_FLOAT_EQ(mav1_vel[2], 0);  

    // acc of mav1
    Eigen::Vector3d mav1_acc = Eigen::Vector3d::Random();
    v_drone_cable.at(1).mav_.GetAcc(mav1_acc);
    EXPECT_FLOAT_EQ(mav1_acc[0], 0); 
    EXPECT_FLOAT_EQ(mav1_acc[1], 0); 
    EXPECT_FLOAT_EQ(mav1_acc[2], 0);  

    // check bodyrate of mav1
    Eigen::Vector3d mav1_bodyrate = Eigen::Vector3d::Random();
    v_drone_cable.at(1).mav_.GetBodyrate(mav1_bodyrate);
    EXPECT_FLOAT_EQ(mav1_bodyrate[0], 0); 
    EXPECT_FLOAT_EQ(mav1_bodyrate[1], 0); 
    EXPECT_FLOAT_EQ(mav1_bodyrate[2], 0);  

    // check attitude of mav1
    Eigen::Quaterniond mav1_att = Eigen::Quaterniond::UnitRandom();
    v_drone_cable.at(1).mav_.GetAttitude(mav1_att);
    EXPECT_FLOAT_EQ(mav1_att.x(), 0);     
    EXPECT_FLOAT_EQ(mav1_att.y(), 0);      
    EXPECT_FLOAT_EQ(mav1_att.z(), 0);     
    EXPECT_FLOAT_EQ(mav1_att.w(), 1);      

    // check bodyrate_acc of mav1
    Eigen::Vector3d mav1_bodyrate_acc = Eigen::Vector3d::Random();
    v_drone_cable.at(1).mav_.GetBodyRateAcc(mav1_bodyrate_acc);
    EXPECT_FLOAT_EQ(mav1_bodyrate_acc[0], 0); 
    EXPECT_FLOAT_EQ(mav1_bodyrate_acc[1], 0); 
    EXPECT_FLOAT_EQ(mav1_bodyrate_acc[2], 0);  


    std::cout<<"[----------] test: mav2_init_post  is " << mav2_init_post.transpose()<<std::endl;
    EXPECT_FLOAT_EQ(mav2_init_post[0], 0); 
    EXPECT_FLOAT_EQ(mav2_init_post[1], 0); 
    EXPECT_FLOAT_EQ(mav2_init_post[2], cable_length);        


    // vel of mav2
    Eigen::Vector3d mav2_vel;
    v_drone_cable.at(2).mav_.GetVel(mav2_vel);    
    EXPECT_FLOAT_EQ(mav2_vel[0], 0); 
    EXPECT_FLOAT_EQ(mav2_vel[1], 0); 
    EXPECT_FLOAT_EQ(mav2_vel[2], 0); 


    // acc of mav1
    Eigen::Vector3d mav2_acc = Eigen::Vector3d::Random();
    v_drone_cable.at(2).mav_.GetAcc(mav2_acc);
    EXPECT_FLOAT_EQ(mav2_acc[0], 0); 
    EXPECT_FLOAT_EQ(mav2_acc[1], 0); 
    EXPECT_FLOAT_EQ(mav2_acc[2], 0);  

    // check bodyrate of mav1
    Eigen::Vector3d mav2_bodyrate = Eigen::Vector3d::Random();
    v_drone_cable.at(2).mav_.GetBodyrate(mav2_bodyrate);
    EXPECT_FLOAT_EQ(mav2_bodyrate[0], 0); 
    EXPECT_FLOAT_EQ(mav2_bodyrate[1], 0); 
    EXPECT_FLOAT_EQ(mav2_bodyrate[2], 0);  

    // check attitude of mav1
    Eigen::Quaterniond mav2_att = Eigen::Quaterniond::UnitRandom();
    v_drone_cable.at(2).mav_.GetAttitude(mav2_att);
    EXPECT_FLOAT_EQ(mav2_att.x(), 0);     
    EXPECT_FLOAT_EQ(mav2_att.y(), 0);      
    EXPECT_FLOAT_EQ(mav2_att.z(), 0);     
    EXPECT_FLOAT_EQ(mav2_att.w(), 1);      

    // check bodyrate_acc of mav1
    Eigen::Vector3d mav2_bodyrate_acc = Eigen::Vector3d::Random();
    v_drone_cable.at(2).mav_.GetBodyRateAcc(mav2_bodyrate_acc);
    EXPECT_FLOAT_EQ(mav2_bodyrate_acc[0], 0); 
    EXPECT_FLOAT_EQ(mav2_bodyrate_acc[1], 0); 
    EXPECT_FLOAT_EQ(mav2_bodyrate_acc[2], 0);          
}


TEST_F(rotorTMCooperative, checkVerticalStaticEquilibrium100Steps){

    // set initial posts for mavs and payload
    ptr_Cooperative->SetPayloadInitPost();

    // input mav controllers' inputs to hover
    // 3 mavs + 1 payload =  3 + 1.5 = 4.5
    // mav thrust = (4.5 * 9.8)/3 = 14.7N

    Eigen::VectorXd v_mavs_thrusts = Eigen::MatrixXd::Constant(3,1,14.7);
    std::vector<Eigen::Vector3d> v_mavs_torques(3, Eigen::Vector3d::Zero());

    std::cout<< "fuck point cooperative test 1"<<std::endl;
    ptr_Cooperative->InputControllerInput4MAVs(v_mavs_thrusts, v_mavs_torques);

    // compute interation wrenches and vars for MAVs and payload
    std::cout<< "fuck point cooperative test 2"<<std::endl;
    ptr_Cooperative->ComputeInteractWrenches();

    // call one step dynamic simulation for MAVs and payload
    std::cout<< "fuck point cooperative test 3"<<std::endl;
    // ptr_Cooperative->DoOneStepInt4Robots();
    // do 10 steps integration
    const double dt = 0.01;
    for(double t=dt ; t<=100*dt ; t+= dt)
    {
            ptr_Cooperative->DoOneStepInt4Robots();
            // printf("current step is %.3f \n", t);
    }

    // 
    Eigen::Vector3d mav0_init_post{0,0,0};
    Eigen::Vector3d mav1_init_post{0,0,0};
    Eigen::Vector3d mav2_init_post{0,0,0};

    std::cout<< "fuck point cooperative test 4"<<std::endl;
    std::vector<UAVCable> v_drone_cable = ptr_Cooperative->v_drone_cable_;

    v_drone_cable.at(0).mav_.GetPosition(mav0_init_post);
    v_drone_cable.at(1).mav_.GetPosition(mav1_init_post);
    v_drone_cable.at(2).mav_.GetPosition(mav2_init_post);        
    double cable_length;
    ptr_Cooperative->v_drone_cable_.at(0).cable_.GetCableLength(cable_length);

    std::cout<<"[----------] test: mav0_init_post  is " << mav0_init_post.transpose()<<std::endl;
    EXPECT_FLOAT_EQ(mav0_init_post[0], 1); 
    EXPECT_FLOAT_EQ(mav0_init_post[1], 0); 
    EXPECT_FLOAT_EQ(mav0_init_post[2], cable_length);  

    // vel of mav0
    Eigen::Vector3d mav0_vel;
    v_drone_cable.at(0).mav_.GetVel(mav0_vel);    
    EXPECT_FLOAT_EQ(mav0_vel[0], 0); 
    EXPECT_FLOAT_EQ(mav0_vel[1], 0); 
    EXPECT_FLOAT_EQ(mav0_vel[2], 0);  

    // acc of mav0
    Eigen::Vector3d mav0_acc = Eigen::Vector3d::Random();
    v_drone_cable.at(0).mav_.GetAcc(mav0_acc);
    EXPECT_FLOAT_EQ(mav0_acc[0], 0); 
    EXPECT_FLOAT_EQ(mav0_acc[1], 0); 
    EXPECT_FLOAT_EQ(mav0_acc[2], 0);  

    // check bodyrate of mav0
    Eigen::Vector3d mav0_bodyrate = Eigen::Vector3d::Random();
    v_drone_cable.at(0).mav_.GetBodyrate(mav0_bodyrate);
    EXPECT_FLOAT_EQ(mav0_bodyrate[0], 0); 
    EXPECT_FLOAT_EQ(mav0_bodyrate[1], 0); 
    EXPECT_FLOAT_EQ(mav0_bodyrate[2], 0);  

    // check attitude of mav0
    Eigen::Quaterniond mav0_att = Eigen::Quaterniond::UnitRandom();
    v_drone_cable.at(0).mav_.GetAttitude(mav0_att);
    EXPECT_FLOAT_EQ(mav0_att.x(), 0);     
    EXPECT_FLOAT_EQ(mav0_att.y(), 0);      
    EXPECT_FLOAT_EQ(mav0_att.z(), 0);     
    EXPECT_FLOAT_EQ(mav0_att.w(), 1);      

    // check bodyrate_acc of mav0
    Eigen::Vector3d mav0_bodyrate_acc = Eigen::Vector3d::Random();
    v_drone_cable.at(0).mav_.GetBodyRateAcc(mav0_bodyrate_acc);
    EXPECT_FLOAT_EQ(mav0_bodyrate_acc[0], 0); 
    EXPECT_FLOAT_EQ(mav0_bodyrate_acc[1], 0); 
    EXPECT_FLOAT_EQ(mav0_bodyrate_acc[2], 0);  



    std::cout<<"[----------] test: mav1_init_post  is " << mav1_init_post.transpose()<<std::endl;
    EXPECT_FLOAT_EQ(mav1_init_post[0], 0); 
    EXPECT_FLOAT_EQ(mav1_init_post[1], 1); 
    EXPECT_FLOAT_EQ(mav1_init_post[2], cable_length);  

    // vel of mav1
    Eigen::Vector3d mav1_vel;
    v_drone_cable.at(1).mav_.GetVel(mav1_vel);    
    EXPECT_FLOAT_EQ(mav1_vel[0], 0); 
    EXPECT_FLOAT_EQ(mav1_vel[1], 0); 
    EXPECT_FLOAT_EQ(mav1_vel[2], 0);  

    // acc of mav1
    Eigen::Vector3d mav1_acc = Eigen::Vector3d::Random();
    v_drone_cable.at(1).mav_.GetAcc(mav1_acc);
    EXPECT_FLOAT_EQ(mav1_acc[0], 0); 
    EXPECT_FLOAT_EQ(mav1_acc[1], 0); 
    EXPECT_FLOAT_EQ(mav1_acc[2], 0);  

    // check bodyrate of mav1
    Eigen::Vector3d mav1_bodyrate = Eigen::Vector3d::Random();
    v_drone_cable.at(1).mav_.GetBodyrate(mav1_bodyrate);
    EXPECT_FLOAT_EQ(mav1_bodyrate[0], 0); 
    EXPECT_FLOAT_EQ(mav1_bodyrate[1], 0); 
    EXPECT_FLOAT_EQ(mav1_bodyrate[2], 0);  

    // check attitude of mav1
    Eigen::Quaterniond mav1_att = Eigen::Quaterniond::UnitRandom();
    v_drone_cable.at(1).mav_.GetAttitude(mav1_att);
    EXPECT_FLOAT_EQ(mav1_att.x(), 0);     
    EXPECT_FLOAT_EQ(mav1_att.y(), 0);      
    EXPECT_FLOAT_EQ(mav1_att.z(), 0);     
    EXPECT_FLOAT_EQ(mav1_att.w(), 1);      

    // check bodyrate_acc of mav1
    Eigen::Vector3d mav1_bodyrate_acc = Eigen::Vector3d::Random();
    v_drone_cable.at(1).mav_.GetBodyRateAcc(mav1_bodyrate_acc);
    EXPECT_FLOAT_EQ(mav1_bodyrate_acc[0], 0); 
    EXPECT_FLOAT_EQ(mav1_bodyrate_acc[1], 0); 
    EXPECT_FLOAT_EQ(mav1_bodyrate_acc[2], 0);  


    std::cout<<"[----------] test: mav2_init_post  is " << mav2_init_post.transpose()<<std::endl;
    EXPECT_FLOAT_EQ(mav2_init_post[0], 0); 
    EXPECT_FLOAT_EQ(mav2_init_post[1], 0); 
    EXPECT_FLOAT_EQ(mav2_init_post[2], cable_length);        


    // vel of mav2
    Eigen::Vector3d mav2_vel;
    v_drone_cable.at(2).mav_.GetVel(mav2_vel);    
    EXPECT_FLOAT_EQ(mav2_vel[0], 0); 
    EXPECT_FLOAT_EQ(mav2_vel[1], 0); 
    EXPECT_FLOAT_EQ(mav2_vel[2], 0); 


    // acc of mav1
    Eigen::Vector3d mav2_acc = Eigen::Vector3d::Random();
    v_drone_cable.at(2).mav_.GetAcc(mav2_acc);
    EXPECT_FLOAT_EQ(mav2_acc[0], 0); 
    EXPECT_FLOAT_EQ(mav2_acc[1], 0); 
    EXPECT_FLOAT_EQ(mav2_acc[2], 0);  

    // check bodyrate of mav1
    Eigen::Vector3d mav2_bodyrate = Eigen::Vector3d::Random();
    v_drone_cable.at(2).mav_.GetBodyrate(mav2_bodyrate);
    EXPECT_FLOAT_EQ(mav2_bodyrate[0], 0); 
    EXPECT_FLOAT_EQ(mav2_bodyrate[1], 0); 
    EXPECT_FLOAT_EQ(mav2_bodyrate[2], 0);  

    // check attitude of mav1
    Eigen::Quaterniond mav2_att = Eigen::Quaterniond::UnitRandom();
    v_drone_cable.at(2).mav_.GetAttitude(mav2_att);
    EXPECT_FLOAT_EQ(mav2_att.x(), 0);     
    EXPECT_FLOAT_EQ(mav2_att.y(), 0);      
    EXPECT_FLOAT_EQ(mav2_att.z(), 0);     
    EXPECT_FLOAT_EQ(mav2_att.w(), 1);      

    // check bodyrate_acc of mav1
    Eigen::Vector3d mav2_bodyrate_acc = Eigen::Vector3d::Random();
    v_drone_cable.at(2).mav_.GetBodyRateAcc(mav2_bodyrate_acc);
    EXPECT_FLOAT_EQ(mav2_bodyrate_acc[0], 0); 
    EXPECT_FLOAT_EQ(mav2_bodyrate_acc[1], 0); 
    EXPECT_FLOAT_EQ(mav2_bodyrate_acc[2], 0);  
}


/*Dynmaic cases*/

TEST_F(rotorTMCooperative, checkVerticalConstAcc){

    // set initial posts for mavs and payload
    ptr_Cooperative->SetPayloadInitPost();

    // input mav controllers' inputs to get 1m/s^2
    // 3 mavs + 1 payload =  3 + 1.5 = 4.5
    // mav thrust = (4.5 * (1+9.8) )/3 = 16.2N

    Eigen::VectorXd v_mavs_thrusts = Eigen::MatrixXd::Constant(3,1,16.2);
    std::vector<Eigen::Vector3d> v_mavs_torques(3, Eigen::Vector3d::Zero());

    std::cout<< "--------------------fuck point cooperative test 1--------------------"<<std::endl;
    ptr_Cooperative->InputControllerInput4MAVs(v_mavs_thrusts, v_mavs_torques);

    // compute interation wrenches and vars for MAVs and payload
    std::cout<< "--------------------fuck point cooperative test 2--------------------"<<std::endl;
    ptr_Cooperative->ComputeInteractWrenches();

    // call one step dynamic simulation for MAVs and payload
    std::cout<< "--------------------fuck point cooperative test 3--------------------"<<std::endl;
    // ptr_Cooperative->DoOneStepInt4Robots();
    // do 1 step integration
    ptr_Cooperative->DoOneStepInt4Robots();
            // printf("current step is %.3f \n", t);

    // 
    Eigen::Vector3d mav0_post{0,0,0};
    Eigen::Vector3d mav1_post{0,0,0};
    Eigen::Vector3d mav2_post{0,0,0};

    std::cout<< "fuck point cooperative test 4"<<std::endl;
    std::vector<UAVCable> v_drone_cable = ptr_Cooperative->v_drone_cable_;

    v_drone_cable.at(0).mav_.GetPosition(mav0_post);
    v_drone_cable.at(1).mav_.GetPosition(mav1_post);
    v_drone_cable.at(2).mav_.GetPosition(mav2_post);        
    double cable_length;
    ptr_Cooperative->v_drone_cable_.at(0).cable_.GetCableLength(cable_length);

    std::cout<<"[--------------------] test: mav0_init_post  is " << mav0_post.transpose()<<std::endl;
    // post of mav0 = 0.5 * a * t
    EXPECT_FLOAT_EQ(mav0_post[0], 1); 
    EXPECT_FLOAT_EQ(mav0_post[1], 0); 
    EXPECT_FLOAT_EQ(mav0_post[2], cable_length+0.5*1*pow(0.01,2));  

    // vel of mav0 =  a*t
    Eigen::Vector3d mav0_vel;
    v_drone_cable.at(0).mav_.GetVel(mav0_vel);    
    EXPECT_FLOAT_EQ(mav0_vel[0], 0); 
    EXPECT_FLOAT_EQ(mav0_vel[1], 0); 
    EXPECT_FLOAT_EQ(mav0_vel[2], 0.01);  

    // acc of mav0 =a
    Eigen::Vector3d mav0_acc = Eigen::Vector3d::Random();
    v_drone_cable.at(0).mav_.GetAcc(mav0_acc);
    EXPECT_FLOAT_EQ(mav0_acc[0], 0); 
    EXPECT_FLOAT_EQ(mav0_acc[1], 0); 
    EXPECT_FLOAT_EQ(mav0_acc[2], 1);  

    // check bodyrate of mav0
    Eigen::Vector3d mav0_bodyrate = Eigen::Vector3d::Random();
    v_drone_cable.at(0).mav_.GetBodyrate(mav0_bodyrate);
    EXPECT_FLOAT_EQ(mav0_bodyrate[0], 0); 
    EXPECT_FLOAT_EQ(mav0_bodyrate[1], 0); 
    EXPECT_FLOAT_EQ(mav0_bodyrate[2], 0);  

    // check attitude of mav0
    Eigen::Quaterniond mav0_att = Eigen::Quaterniond::UnitRandom();
    v_drone_cable.at(0).mav_.GetAttitude(mav0_att);
    EXPECT_FLOAT_EQ(mav0_att.x(), 0);     
    EXPECT_FLOAT_EQ(mav0_att.y(), 0);      
    EXPECT_FLOAT_EQ(mav0_att.z(), 0);     
    EXPECT_FLOAT_EQ(mav0_att.w(), 1);      

    // check bodyrate_acc of mav0
    Eigen::Vector3d mav0_bodyrate_acc = Eigen::Vector3d::Random();
    v_drone_cable.at(0).mav_.GetBodyRateAcc(mav0_bodyrate_acc);
    EXPECT_FLOAT_EQ(mav0_bodyrate_acc[0], 0); 
    EXPECT_FLOAT_EQ(mav0_bodyrate_acc[1], 0); 
    EXPECT_FLOAT_EQ(mav0_bodyrate_acc[2], 0);  



    std::cout<<"[--------------------] test: mav1_init_post  is " << mav1_post.transpose()<<std::endl;
    EXPECT_FLOAT_EQ(mav1_post[0], 0); 
    EXPECT_FLOAT_EQ(mav1_post[1], 1); 
    EXPECT_FLOAT_EQ(mav1_post[2], cable_length + 0.5*1*pow(0.01,2));  

    // vel of mav1
    Eigen::Vector3d mav1_vel;
    v_drone_cable.at(1).mav_.GetVel(mav1_vel);    
    EXPECT_FLOAT_EQ(mav1_vel[0], 0); 
    EXPECT_FLOAT_EQ(mav1_vel[1], 0); 
    EXPECT_FLOAT_EQ(mav1_vel[2], 0.01);  

    // acc of mav1
    Eigen::Vector3d mav1_acc = Eigen::Vector3d::Random();
    v_drone_cable.at(1).mav_.GetAcc(mav1_acc);
    EXPECT_FLOAT_EQ(mav1_acc[0], 0); 
    EXPECT_FLOAT_EQ(mav1_acc[1], 0); 
    EXPECT_FLOAT_EQ(mav1_acc[2], 1);  

    // check bodyrate of mav1
    Eigen::Vector3d mav1_bodyrate = Eigen::Vector3d::Random();
    v_drone_cable.at(1).mav_.GetBodyrate(mav1_bodyrate);
    EXPECT_FLOAT_EQ(mav1_bodyrate[0], 0); 
    EXPECT_FLOAT_EQ(mav1_bodyrate[1], 0); 
    EXPECT_FLOAT_EQ(mav1_bodyrate[2], 0);  

    // check attitude of mav1
    Eigen::Quaterniond mav1_att = Eigen::Quaterniond::UnitRandom();
    v_drone_cable.at(1).mav_.GetAttitude(mav1_att);
    EXPECT_FLOAT_EQ(mav1_att.x(), 0);     
    EXPECT_FLOAT_EQ(mav1_att.y(), 0);      
    EXPECT_FLOAT_EQ(mav1_att.z(), 0);     
    EXPECT_FLOAT_EQ(mav1_att.w(), 1);      

    // check bodyrate_acc of mav1
    Eigen::Vector3d mav1_bodyrate_acc = Eigen::Vector3d::Random();
    v_drone_cable.at(1).mav_.GetBodyRateAcc(mav1_bodyrate_acc);
    EXPECT_FLOAT_EQ(mav1_bodyrate_acc[0], 0); 
    EXPECT_FLOAT_EQ(mav1_bodyrate_acc[1], 0); 
    EXPECT_FLOAT_EQ(mav1_bodyrate_acc[2], 0);  


    std::cout<<"[--------------------] test: mav2_init_post  is " << mav2_post.transpose()<<std::endl;
    EXPECT_FLOAT_EQ(mav2_post[0], 0); 
    EXPECT_FLOAT_EQ(mav2_post[1], 0); 
    EXPECT_FLOAT_EQ(mav2_post[2], cable_length+0.5*1*pow(0.01,2));        


    // vel of mav2
    Eigen::Vector3d mav2_vel;
    v_drone_cable.at(2).mav_.GetVel(mav2_vel);    
    EXPECT_FLOAT_EQ(mav2_vel[0], 0); 
    EXPECT_FLOAT_EQ(mav2_vel[1], 0); 
    EXPECT_FLOAT_EQ(mav2_vel[2], 0.01); 


    // acc of mav1
    Eigen::Vector3d mav2_acc = Eigen::Vector3d::Random();
    v_drone_cable.at(2).mav_.GetAcc(mav2_acc);
    EXPECT_FLOAT_EQ(mav2_acc[0], 0); 
    EXPECT_FLOAT_EQ(mav2_acc[1], 0); 
    EXPECT_FLOAT_EQ(mav2_acc[2], 1);  

    // check bodyrate of mav1
    Eigen::Vector3d mav2_bodyrate = Eigen::Vector3d::Random();
    v_drone_cable.at(2).mav_.GetBodyrate(mav2_bodyrate);
    EXPECT_FLOAT_EQ(mav2_bodyrate[0], 0); 
    EXPECT_FLOAT_EQ(mav2_bodyrate[1], 0); 
    EXPECT_FLOAT_EQ(mav2_bodyrate[2], 0);  

    // check attitude of mav1
    Eigen::Quaterniond mav2_att = Eigen::Quaterniond::UnitRandom();
    v_drone_cable.at(2).mav_.GetAttitude(mav2_att);
    EXPECT_FLOAT_EQ(mav2_att.x(), 0);     
    EXPECT_FLOAT_EQ(mav2_att.y(), 0);      
    EXPECT_FLOAT_EQ(mav2_att.z(), 0);     
    EXPECT_FLOAT_EQ(mav2_att.w(), 1);      

    // check bodyrate_acc of mav1
    Eigen::Vector3d mav2_bodyrate_acc = Eigen::Vector3d::Random();
    v_drone_cable.at(2).mav_.GetBodyRateAcc(mav2_bodyrate_acc);
    EXPECT_FLOAT_EQ(mav2_bodyrate_acc[0], 0); 
    EXPECT_FLOAT_EQ(mav2_bodyrate_acc[1], 0); 
    EXPECT_FLOAT_EQ(mav2_bodyrate_acc[2], 0);  
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