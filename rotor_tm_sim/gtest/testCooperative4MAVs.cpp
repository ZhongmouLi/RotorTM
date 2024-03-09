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
    std::vector<Eigen::Vector3d> v_attach_point_post{{1,1,0},{-1,1,0}, {-1,-1,0}, {1,-1,0}};

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

// // test if initial posts are set for mavs
// // test if instance is created
// TEST_F(rotorTMCooperative, checkInitialPostsNoInput){
//     ptr_Cooperative->SetPayloadInitPost();

//     Eigen::Vector3d payload_init_post = Eigen::Vector3d::Random();
//     Eigen::Vector3d mav0_init_post = Eigen::Vector3d::Random();
//     Eigen::Vector3d mav1_init_post = Eigen::Vector3d::Random();
//     Eigen::Vector3d mav2_init_post = Eigen::Vector3d::Random();
//     Eigen::Vector3d mav3_init_post = Eigen::Vector3d::Random();


//     ptr_Cooperative->payload_.GetPosition(payload_init_post);
//     ASSERT_FLOAT_EQ(payload_init_post[0], 0); 
//     ASSERT_FLOAT_EQ(payload_init_post[1], 0); 
//     ASSERT_FLOAT_EQ(payload_init_post[2], 0); 


//     // std::cout<< "fuck point cooperative test 1"<<std::endl;
//     std::vector<UAVCable> &v_drone_cable = ptr_Cooperative->v_drone_cable_;

//     // UAVCable mavcable1 = v_drone_cable[0];
//     // mavcable1.mav_.GetPosition(mav0_init_post); // check at(0).mav_ if it gets correct values passed in
    
//     v_drone_cable.at(0).mav_.GetPosition(mav0_init_post);
//     v_drone_cable.at(1).mav_.GetPosition(mav1_init_post);
//     v_drone_cable.at(2).mav_.GetPosition(mav2_init_post);        
//     v_drone_cable.at(3).mav_.GetPosition(mav3_init_post);  
    
//     double cable_length;
//     ptr_Cooperative->v_drone_cable_.at(0).cable_.GetCableLength(cable_length);

//     // std::cout<<"[----------] test: mav0_init_post  is " << mav0_init_post.transpose()<<std::endl;
//     ASSERT_FLOAT_EQ(mav0_init_post[0], 1); 
//     ASSERT_FLOAT_EQ(mav0_init_post[1], 1); 
//     ASSERT_FLOAT_EQ(mav0_init_post[2], cable_length);  

//     // std::cout<<"[----------] test: mav1_init_post  is " << mav1_init_post.transpose()<<std::endl;
//     ASSERT_FLOAT_EQ(mav1_init_post[0], -1); 
//     ASSERT_FLOAT_EQ(mav1_init_post[1], 1); 
//     ASSERT_FLOAT_EQ(mav1_init_post[2], cable_length);  

//     // std::cout<<"[----------] test: mav2_init_post  is " << mav2_init_post.transpose()<<std::endl;
//     ASSERT_FLOAT_EQ(mav2_init_post[0], -1); 
//     ASSERT_FLOAT_EQ(mav2_init_post[1], -1); 
//     ASSERT_FLOAT_EQ(mav2_init_post[2], cable_length);          


//     ASSERT_FLOAT_EQ(mav3_init_post[0], 1); 
//     ASSERT_FLOAT_EQ(mav3_init_post[1], -1); 
//     ASSERT_FLOAT_EQ(mav3_init_post[2], cable_length);       

// }




// TEST_F(rotorTMCooperative, checkInitialPostsInput){

//     Eigen::Vector3d payload_init_post{1,2,3};
//     ptr_Cooperative->SetPayloadInitPost(payload_init_post);

//     Eigen::Vector3d payload_post = Eigen::Vector3d::Random();
//     Eigen::Vector3d mav0_post = Eigen::Vector3d::Random();
//     Eigen::Vector3d mav1_post = Eigen::Vector3d::Random();
//     Eigen::Vector3d mav2_post = Eigen::Vector3d::Random();
//     Eigen::Vector3d mav3_post = Eigen::Vector3d::Random();


//     ptr_Cooperative->payload_.GetPosition(payload_post);
//     ASSERT_FLOAT_EQ(payload_post[0], payload_init_post[0]); 
//     ASSERT_FLOAT_EQ(payload_post[1], payload_init_post[1]); 
//     ASSERT_FLOAT_EQ(payload_post[2], payload_init_post[2]); 


//     // std::cout<< "fuck point cooperative test 1"<<std::endl;
//     std::vector<UAVCable> &v_drone_cable = ptr_Cooperative->v_drone_cable_;

//     // UAVCable mavcable1 = v_drone_cable[0];
//     // mavcable1.mav_.GetPosition(mav0_init_post); // check at(0).mav_ if it gets correct values passed in
    
//     v_drone_cable.at(0).mav_.GetPosition(mav0_post);
//     v_drone_cable.at(1).mav_.GetPosition(mav1_post);
//     v_drone_cable.at(2).mav_.GetPosition(mav2_post);        
//     v_drone_cable.at(3).mav_.GetPosition(mav3_post);  
    
//     double cable_length;
//     ptr_Cooperative->v_drone_cable_.at(0).cable_.GetCableLength(cable_length);

//     // std::cout<<"[----------] test: mav0_init_post  is " << mav0_init_post.transpose()<<std::endl;
//     ASSERT_FLOAT_EQ(mav0_post[0], payload_init_post[0]+1); 
//     ASSERT_FLOAT_EQ(mav0_post[1], payload_init_post[1]+1); 
//     ASSERT_FLOAT_EQ(mav0_post[2], payload_init_post[2]+cable_length);  

//     // std::cout<<"[----------] test: mav1_init_post  is " << mav1_init_post.transpose()<<std::endl;
//     ASSERT_FLOAT_EQ(mav1_post[0], payload_init_post[0]-1); 
//     ASSERT_FLOAT_EQ(mav1_post[1], payload_init_post[1]+1); 
//     ASSERT_FLOAT_EQ(mav1_post[2], payload_init_post[2]+cable_length);  

//     // std::cout<<"[----------] test: mav2_init_post  is " << mav2_init_post.transpose()<<std::endl;
//     ASSERT_FLOAT_EQ(mav2_post[0], payload_init_post[0]-1); 
//     ASSERT_FLOAT_EQ(mav2_post[1], payload_init_post[1]-1); 
//     ASSERT_FLOAT_EQ(mav2_post[2], payload_init_post[2]+cable_length);          


//     ASSERT_FLOAT_EQ(mav3_post[0], payload_init_post[0]+1); 
//     ASSERT_FLOAT_EQ(mav3_post[1], payload_init_post[1]-1); 
//     ASSERT_FLOAT_EQ(mav3_post[2], payload_init_post[2]+cable_length);       

// }







// TEST_F(rotorTMCooperative, checkInitialPostsRandInput){

//     Eigen::Vector3d payload_init_post = Eigen::Vector3d::Random();
//     ptr_Cooperative->SetPayloadInitPost(payload_init_post);

//     Eigen::Vector3d payload_post = Eigen::Vector3d::Random();
//     Eigen::Vector3d mav0_post = Eigen::Vector3d::Random();
//     Eigen::Vector3d mav1_post = Eigen::Vector3d::Random();
//     Eigen::Vector3d mav2_post = Eigen::Vector3d::Random();
//     Eigen::Vector3d mav3_post = Eigen::Vector3d::Random();


//     ptr_Cooperative->payload_.GetPosition(payload_post);
//     ASSERT_FLOAT_EQ(payload_post[0], payload_init_post[0]); 
//     ASSERT_FLOAT_EQ(payload_post[1], payload_init_post[1]); 
//     ASSERT_FLOAT_EQ(payload_post[2], payload_init_post[2]); 


//     // std::cout<< "fuck point cooperative test 1"<<std::endl;
//     std::vector<UAVCable> &v_drone_cable = ptr_Cooperative->v_drone_cable_;

//     // UAVCable mavcable1 = v_drone_cable[0];
//     // mavcable1.mav_.GetPosition(mav0_init_post); // check at(0).mav_ if it gets correct values passed in
    
//     v_drone_cable.at(0).mav_.GetPosition(mav0_post);
//     v_drone_cable.at(1).mav_.GetPosition(mav1_post);
//     v_drone_cable.at(2).mav_.GetPosition(mav2_post);        
//     v_drone_cable.at(3).mav_.GetPosition(mav3_post);  
    
//     double cable_length;
//     ptr_Cooperative->v_drone_cable_.at(0).cable_.GetCableLength(cable_length);

//     // std::cout<<"[----------] test: mav0_init_post  is " << mav0_init_post.transpose()<<std::endl;
//     ASSERT_FLOAT_EQ(mav0_post[0], payload_init_post[0]+1); 
//     ASSERT_FLOAT_EQ(mav0_post[1], payload_init_post[1]+1); 
//     ASSERT_FLOAT_EQ(mav0_post[2], payload_init_post[2]+cable_length);  

//     // std::cout<<"[----------] test: mav1_init_post  is " << mav1_init_post.transpose()<<std::endl;
//     ASSERT_FLOAT_EQ(mav1_post[0], payload_init_post[0]-1); 
//     ASSERT_FLOAT_EQ(mav1_post[1], payload_init_post[1]+1); 
//     ASSERT_FLOAT_EQ(mav1_post[2], payload_init_post[2]+cable_length);  

//     // std::cout<<"[----------] test: mav2_init_post  is " << mav2_init_post.transpose()<<std::endl;
//     ASSERT_FLOAT_EQ(mav2_post[0], payload_init_post[0]-1); 
//     ASSERT_FLOAT_EQ(mav2_post[1], payload_init_post[1]-1); 
//     ASSERT_FLOAT_EQ(mav2_post[2], payload_init_post[2]+cable_length);          


//     ASSERT_FLOAT_EQ(mav3_post[0], payload_init_post[0]+1); 
//     ASSERT_FLOAT_EQ(mav3_post[1], payload_init_post[1]-1); 
//     ASSERT_FLOAT_EQ(mav3_post[2], payload_init_post[2]+cable_length);       

// }












// // test if initial posts are set for mavs
// // test if instance is created
// TEST_F(rotorTMCooperative, checkVerticalStaticEquilibrium){

//     // set initial posts for mavs and payload
//     ptr_Cooperative->SetPayloadInitPost();

//     // input mav controllers' inputs to hover
//     // 4 mavs + 1 payload =  4 + 1.5 = 5.5
//     // mav thrust = (5.5 * 9.8)/4 = 13.475N

//     Eigen::VectorXd v_mavs_thrusts = Eigen::MatrixXd::Constant(4,1,13.475);
//     std::vector<Eigen::Vector3d> v_mavs_torques(4, Eigen::Vector3d::Zero());

//     // std::cout<< "fuck point cooperative test 1"<<std::endl;
//     ptr_Cooperative->InputControllerInput4MAVs(v_mavs_thrusts, v_mavs_torques);

//     // compute interation wrenches and vars for MAVs and payload
//     // std::cout<< "fuck point cooperative test 2"<<std::endl;
//     ptr_Cooperative->ComputeInteractWrenches();

//     // call one step dynamic simulation for MAVs and payload
//     // std::cout<< "fuck point cooperative test 3"<<std::endl;
//     ptr_Cooperative->DoOneStepInt4Robots();


//     // payload
//     Eigen::Vector3d payload_post = Eigen::Vector3d::Random();
//     Eigen::Vector3d payload_vel = Eigen::Vector3d::Random();
//     Eigen::Vector3d payload_acc = Eigen::Vector3d::Random();
//     Eigen::Quaterniond payload_attitude = Eigen::Quaterniond::UnitRandom();
//     Eigen::Vector3d payload_bodyrate = Eigen::Vector3d::Random();
//     Eigen::Vector3d payload_bodyrate_acc = Eigen::Vector3d::Random();

//     // post of payload
//     ptr_Cooperative->payload_.GetPosition(payload_post);
//     ASSERT_FLOAT_EQ(payload_post[0], 0); 
//     ASSERT_FLOAT_EQ(payload_post[1], 0); 
//     ASSERT_NEAR(payload_post[2], 0, 1e-10); 

//     // vel of payload
//     ptr_Cooperative->payload_.GetVel(payload_vel);
//     ASSERT_FLOAT_EQ(payload_vel[0], 0); 
//     ASSERT_FLOAT_EQ(payload_vel[1], 0); 
//     ASSERT_NEAR(payload_vel[2], 0, 1e-10);     

//     // acc of payload
//     ptr_Cooperative->payload_.GetAcc(payload_acc);
//     ASSERT_FLOAT_EQ(payload_acc[0], 0); 
//     ASSERT_FLOAT_EQ(payload_acc[1], 0); 
//     ASSERT_NEAR(payload_acc[2], 0, 1e-10);   

//     //  bodyrate of payload
//     ptr_Cooperative->payload_.GetBodyrate(payload_bodyrate);
//     ASSERT_FLOAT_EQ(payload_bodyrate[0], 0); 
//     ASSERT_FLOAT_EQ(payload_bodyrate[1], 0); 
//     ASSERT_FLOAT_EQ(payload_bodyrate[2], 0);  

//     // attitude of payload
//     ptr_Cooperative->payload_.GetAttitude(payload_attitude);
//     ASSERT_FLOAT_EQ(payload_attitude.x(), 0);     
//     ASSERT_FLOAT_EQ(payload_attitude.y(), 0);      
//     ASSERT_FLOAT_EQ(payload_attitude.z(), 0);     
//     ASSERT_FLOAT_EQ(payload_attitude.w(), 1);      

//     // bodyrate_acc of payload
//     ptr_Cooperative->payload_.GetBodyRateAcc(payload_bodyrate_acc);
//     ASSERT_FLOAT_EQ(payload_bodyrate_acc[0], 0); 
//     ASSERT_FLOAT_EQ(payload_bodyrate_acc[1], 0); 
//     ASSERT_FLOAT_EQ(payload_bodyrate_acc[2], 0);  


//     // mavs
//     Eigen::Vector3d mav0_post{0,0,0};
//     Eigen::Vector3d mav1_post{0,0,0};
//     Eigen::Vector3d mav2_post{0,0,0};
//     Eigen::Vector3d mav3_post{0,0,0};

//     // std::cout<< "fuck point cooperative test 4"<<std::endl;
//     std::vector<UAVCable> v_drone_cable = ptr_Cooperative->v_drone_cable_;

//     v_drone_cable.at(0).mav_.GetPosition(mav0_post);
//     v_drone_cable.at(1).mav_.GetPosition(mav1_post);
//     v_drone_cable.at(2).mav_.GetPosition(mav2_post);    
//     v_drone_cable.at(3).mav_.GetPosition(mav3_post);    

//     double cable_length;
//     ptr_Cooperative->v_drone_cable_.at(0).cable_.GetCableLength(cable_length);

//     // std::cout<<"[----------] test: mav1_init_post  is " << mav1_init_post.transpose()<<std::endl;
//     ASSERT_FLOAT_EQ(mav0_post[0], 1); 
//     ASSERT_FLOAT_EQ(mav0_post[1], 1); 
//     ASSERT_FLOAT_EQ(mav0_post[2], cable_length);  

//     // vel of mav0
//     Eigen::Vector3d mav0_vel;
//     v_drone_cable.at(0).mav_.GetVel(mav0_vel);    
//     ASSERT_FLOAT_EQ(mav0_vel[0], 0); 
//     ASSERT_FLOAT_EQ(mav0_vel[1], 0); 
//     ASSERT_NEAR(mav0_vel[2], 0.0, 1e-10); 

//     // acc of mav0
//     Eigen::Vector3d mav0_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(0).mav_.GetAcc(mav0_acc);
//     ASSERT_FLOAT_EQ(mav0_acc[0], 0); 
//     ASSERT_FLOAT_EQ(mav0_acc[1], 0); 
//     ASSERT_NEAR(mav0_acc[2], 0, 1e-10); 

//     // check bodyrate of mav0
//     Eigen::Vector3d mav0_bodyrate = Eigen::Vector3d::Random();
//     v_drone_cable.at(0).mav_.GetBodyrate(mav0_bodyrate);
//     ASSERT_FLOAT_EQ(mav0_bodyrate[0], 0); 
//     ASSERT_FLOAT_EQ(mav0_bodyrate[1], 0); 
//     ASSERT_FLOAT_EQ(mav0_bodyrate[2], 0);  

//     // check attitude of mav0
//     Eigen::Quaterniond mav0_att = Eigen::Quaterniond::UnitRandom();
//     v_drone_cable.at(0).mav_.GetAttitude(mav0_att);
//     ASSERT_FLOAT_EQ(mav0_att.x(), 0);     
//     ASSERT_FLOAT_EQ(mav0_att.y(), 0);      
//     ASSERT_FLOAT_EQ(mav0_att.z(), 0);     
//     ASSERT_FLOAT_EQ(mav0_att.w(), 1);      

//     // check bodyrate_acc of mav0
//     Eigen::Vector3d mav0_bodyrate_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(0).mav_.GetBodyRateAcc(mav0_bodyrate_acc);
//     ASSERT_FLOAT_EQ(mav0_bodyrate_acc[0], 0); 
//     ASSERT_FLOAT_EQ(mav0_bodyrate_acc[1], 0); 
//     ASSERT_FLOAT_EQ(mav0_bodyrate_acc[2], 0);  



//     //mav1
//     ASSERT_FLOAT_EQ(mav1_post[0], -1); 
//     ASSERT_FLOAT_EQ(mav1_post[1], 1); 
//     ASSERT_FLOAT_EQ(mav1_post[2], cable_length);  

//     // vel of mav1
//     Eigen::Vector3d mav1_vel;
//     v_drone_cable.at(1).mav_.GetVel(mav1_vel);    
//     ASSERT_FLOAT_EQ(mav1_vel[0], 0); 
//     ASSERT_FLOAT_EQ(mav1_vel[1], 0); 
//     ASSERT_NEAR(mav1_vel[2], 0.0, 1e-10);   

//     // acc of mav1
//     Eigen::Vector3d mav1_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(1).mav_.GetAcc(mav1_acc);
//     ASSERT_FLOAT_EQ(mav1_acc[0], 0); 
//     ASSERT_FLOAT_EQ(mav1_acc[1], 0); 
//     ASSERT_NEAR(mav1_acc[2], 0, 1e-10); 

//     // check bodyrate of mav1
//     Eigen::Vector3d mav1_bodyrate = Eigen::Vector3d::Random();
//     v_drone_cable.at(1).mav_.GetBodyrate(mav1_bodyrate);
//     ASSERT_FLOAT_EQ(mav1_bodyrate[0], 0); 
//     ASSERT_FLOAT_EQ(mav1_bodyrate[1], 0); 
//     ASSERT_FLOAT_EQ(mav1_bodyrate[2], 0);  

//     // check attitude of mav1
//     Eigen::Quaterniond mav1_att = Eigen::Quaterniond::UnitRandom();
//     v_drone_cable.at(1).mav_.GetAttitude(mav1_att);
//     ASSERT_FLOAT_EQ(mav1_att.x(), 0);     
//     ASSERT_FLOAT_EQ(mav1_att.y(), 0);      
//     ASSERT_FLOAT_EQ(mav1_att.z(), 0);     
//     ASSERT_FLOAT_EQ(mav1_att.w(), 1);      

//     // check bodyrate_acc of mav1
//     Eigen::Vector3d mav1_bodyrate_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(1).mav_.GetBodyRateAcc(mav1_bodyrate_acc);
//     ASSERT_FLOAT_EQ(mav1_bodyrate_acc[0], 0); 
//     ASSERT_FLOAT_EQ(mav1_bodyrate_acc[1], 0); 
//     ASSERT_FLOAT_EQ(mav1_bodyrate_acc[2], 0);      


//     //mav2
//     ASSERT_FLOAT_EQ(mav2_post[0], -1); 
//     ASSERT_FLOAT_EQ(mav2_post[1], -1); 
//     ASSERT_FLOAT_EQ(mav2_post[2], cable_length);  

//     // vel of mav2
//     Eigen::Vector3d mav2_vel;
//     v_drone_cable.at(2).mav_.GetVel(mav2_vel);    
//     ASSERT_FLOAT_EQ(mav2_vel[0], 0); 
//     ASSERT_FLOAT_EQ(mav2_vel[1], 0); 
//     ASSERT_NEAR(mav2_vel[2], 0.0, 1e-10);  

//     // acc of mav2
//     Eigen::Vector3d mav2_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(2).mav_.GetAcc(mav2_acc);
//     ASSERT_FLOAT_EQ(mav2_acc[0], 0); 
//     ASSERT_FLOAT_EQ(mav2_acc[1], 0); 
//     ASSERT_NEAR(mav2_acc[2], 0, 1e-10);   

//     // check bodyrate of mav2
//     Eigen::Vector3d mav2_bodyrate = Eigen::Vector3d::Random();
//     v_drone_cable.at(2).mav_.GetBodyrate(mav2_bodyrate);
//     ASSERT_FLOAT_EQ(mav2_bodyrate[0], 0); 
//     ASSERT_FLOAT_EQ(mav2_bodyrate[1], 0); 
//     ASSERT_FLOAT_EQ(mav2_bodyrate[2], 0);  

//     // check attitude of mav2
//     Eigen::Quaterniond mav2_att = Eigen::Quaterniond::UnitRandom();
//     v_drone_cable.at(2).mav_.GetAttitude(mav2_att);
//     ASSERT_FLOAT_EQ(mav2_att.x(), 0);     
//     ASSERT_FLOAT_EQ(mav2_att.y(), 0);      
//     ASSERT_FLOAT_EQ(mav2_att.z(), 0);     
//     ASSERT_FLOAT_EQ(mav2_att.w(), 1);      

//     // check bodyrate_acc of mav2
//     Eigen::Vector3d mav2_bodyrate_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(2).mav_.GetBodyRateAcc(mav2_bodyrate_acc);
//     ASSERT_FLOAT_EQ(mav2_bodyrate_acc[0], 0); 
//     ASSERT_FLOAT_EQ(mav2_bodyrate_acc[1], 0); 
//     ASSERT_FLOAT_EQ(mav2_bodyrate_acc[2], 0); 

//     //mav3
//     ASSERT_FLOAT_EQ(mav3_post[0], 1); 
//     ASSERT_FLOAT_EQ(mav3_post[1], -1); 
//     ASSERT_FLOAT_EQ(mav3_post[2], cable_length);  

//     // vel of mav3
//     Eigen::Vector3d mav3_vel;
//     v_drone_cable.at(3).mav_.GetVel(mav3_vel);    
//     ASSERT_FLOAT_EQ(mav3_vel[0], 0); 
//     ASSERT_FLOAT_EQ(mav3_vel[1], 0); 
//     ASSERT_NEAR(mav3_vel[2], 0.0, 1e-10); 

//     // acc of mav3
//     Eigen::Vector3d mav3_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(3).mav_.GetAcc(mav3_acc);
//     ASSERT_FLOAT_EQ(mav3_acc[0], 0); 
//     ASSERT_FLOAT_EQ(mav3_acc[1], 0); 
//     ASSERT_NEAR(mav3_acc[2], 0, 1e-10); 

//     // check bodyrate of mav3
//     Eigen::Vector3d mav3_bodyrate = Eigen::Vector3d::Random();
//     v_drone_cable.at(3).mav_.GetBodyrate(mav3_bodyrate);
//     ASSERT_FLOAT_EQ(mav3_bodyrate[0], 0); 
//     ASSERT_FLOAT_EQ(mav3_bodyrate[1], 0); 
//     ASSERT_FLOAT_EQ(mav3_bodyrate[2], 0);  

//     // check attitude of mav3
//     Eigen::Quaterniond mav3_att = Eigen::Quaterniond::UnitRandom();
//     v_drone_cable.at(3).mav_.GetAttitude(mav3_att);
//     ASSERT_FLOAT_EQ(mav3_att.x(), 0);     
//     ASSERT_FLOAT_EQ(mav3_att.y(), 0);      
//     ASSERT_FLOAT_EQ(mav3_att.z(), 0);     
//     ASSERT_FLOAT_EQ(mav3_att.w(), 1);      

//     // check bodyrate_acc of mav3
//     Eigen::Vector3d mav3_bodyrate_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(3).mav_.GetBodyRateAcc(mav3_bodyrate_acc);
//     ASSERT_FLOAT_EQ(mav3_bodyrate_acc[0], 0); 
//     ASSERT_FLOAT_EQ(mav3_bodyrate_acc[1], 0); 
//     ASSERT_FLOAT_EQ(mav3_bodyrate_acc[2], 0);     
// }




// test if initial posts are set for mavs
// test if instance is created
TEST_F(rotorTMCooperative, checkVerticalInitialPostStaticEquilibrium){

    // set initial posts for mavs and payload
    Eigen::Vector3d payload_init_post{1,2,3};
    ptr_Cooperative->SetPayloadInitPost(payload_init_post);

    // input mav controllers' inputs to hover
    // 4 mavs + 1 payload =  4 + 1.5 = 5.5
    // mav thrust = (5.5 * 9.8)/4 = 13.475N

    Eigen::VectorXd v_mavs_thrusts = Eigen::MatrixXd::Constant(4,1,13.475);
    std::vector<Eigen::Vector3d> v_mavs_torques(4, Eigen::Vector3d::Zero());

    // std::cout<< "fuck point cooperative test 1"<<std::endl;
    ptr_Cooperative->InputControllerInput4MAVs(v_mavs_thrusts, v_mavs_torques);

    // compute interation wrenches and vars for MAVs and payload
    // std::cout<< "fuck point cooperative test 2"<<std::endl;
    ptr_Cooperative->ComputeInteractWrenches();

    // call one step dynamic simulation for MAVs and payload
    // std::cout<< "fuck point cooperative test 3"<<std::endl;
    ptr_Cooperative->DoOneStepInt4Robots();


    // payload
    Eigen::Vector3d payload_post = Eigen::Vector3d::Random();
    Eigen::Vector3d payload_vel = Eigen::Vector3d::Random();
    Eigen::Vector3d payload_acc = Eigen::Vector3d::Random();
    Eigen::Quaterniond payload_attitude = Eigen::Quaterniond::UnitRandom();
    Eigen::Vector3d payload_bodyrate = Eigen::Vector3d::Random();
    Eigen::Vector3d payload_bodyrate_acc = Eigen::Vector3d::Random();

    // post of payload
    ptr_Cooperative->payload_.GetPosition(payload_post);
    ASSERT_FLOAT_EQ(payload_post[0], payload_init_post[0]); 
    ASSERT_FLOAT_EQ(payload_post[1], payload_init_post[1]); 
    ASSERT_NEAR(payload_post[2], payload_init_post[2], 1e-6); 

    // vel of payload
    ptr_Cooperative->payload_.GetVel(payload_vel);
    ASSERT_FLOAT_EQ(payload_vel[0], 0); 
    ASSERT_FLOAT_EQ(payload_vel[1], 0); 
    ASSERT_NEAR(payload_vel[2], 0, 1e-10);     

    // acc of payload
    ptr_Cooperative->payload_.GetAcc(payload_acc);

    std::cout<<"FUCK " <<payload_acc.transpose()<<std::endl;
    ASSERT_FLOAT_EQ(payload_acc[0], 0); 
    ASSERT_FLOAT_EQ(payload_acc[1], 0); 
    ASSERT_NEAR(payload_acc[2], 0, 1e-10);   

    //  bodyrate of payload
    ptr_Cooperative->payload_.GetBodyrate(payload_bodyrate);
    ASSERT_FLOAT_EQ(payload_bodyrate[0], 0); 
    ASSERT_FLOAT_EQ(payload_bodyrate[1], 0); 
    ASSERT_FLOAT_EQ(payload_bodyrate[2], 0);  

    // attitude of payload
    ptr_Cooperative->payload_.GetAttitude(payload_attitude);
    ASSERT_FLOAT_EQ(payload_attitude.x(), 0);     
    ASSERT_FLOAT_EQ(payload_attitude.y(), 0);      
    ASSERT_FLOAT_EQ(payload_attitude.z(), 0);     
    ASSERT_FLOAT_EQ(payload_attitude.w(), 1);      

    // bodyrate_acc of payload
    ptr_Cooperative->payload_.GetBodyRateAcc(payload_bodyrate_acc);
    ASSERT_FLOAT_EQ(payload_bodyrate_acc[0], 0); 
    ASSERT_FLOAT_EQ(payload_bodyrate_acc[1], 0); 
    ASSERT_FLOAT_EQ(payload_bodyrate_acc[2], 0);  


    // mavs
    Eigen::Vector3d mav0_post{0,0,0};
    Eigen::Vector3d mav1_post{0,0,0};
    Eigen::Vector3d mav2_post{0,0,0};
    Eigen::Vector3d mav3_post{0,0,0};

    // std::cout<< "fuck point cooperative test 4"<<std::endl;
    std::vector<UAVCable> v_drone_cable = ptr_Cooperative->v_drone_cable_;

    v_drone_cable.at(0).mav_.GetPosition(mav0_post);
    v_drone_cable.at(1).mav_.GetPosition(mav1_post);
    v_drone_cable.at(2).mav_.GetPosition(mav2_post);    
    v_drone_cable.at(3).mav_.GetPosition(mav3_post);    

    double cable_length;
    ptr_Cooperative->v_drone_cable_.at(0).cable_.GetCableLength(cable_length);

    // std::cout<<"[----------] test: mav1_init_post  is " << mav1_init_post.transpose()<<std::endl;
    ASSERT_FLOAT_EQ(mav0_post[0], 1); 
    ASSERT_FLOAT_EQ(mav0_post[1], 1); 
    ASSERT_FLOAT_EQ(mav0_post[2], cable_length);  

    // vel of mav0
    Eigen::Vector3d mav0_vel;
    v_drone_cable.at(0).mav_.GetVel(mav0_vel);    
    ASSERT_FLOAT_EQ(mav0_vel[0], 0); 
    ASSERT_FLOAT_EQ(mav0_vel[1], 0); 
    ASSERT_NEAR(mav0_vel[2], 0.0, 1e-10); 

    // acc of mav0
    Eigen::Vector3d mav0_acc = Eigen::Vector3d::Random();
    v_drone_cable.at(0).mav_.GetAcc(mav0_acc);
    ASSERT_FLOAT_EQ(mav0_acc[0], 0); 
    ASSERT_FLOAT_EQ(mav0_acc[1], 0); 
    ASSERT_NEAR(mav0_acc[2], 0, 1e-10); 

    // check bodyrate of mav0
    Eigen::Vector3d mav0_bodyrate = Eigen::Vector3d::Random();
    v_drone_cable.at(0).mav_.GetBodyrate(mav0_bodyrate);
    ASSERT_FLOAT_EQ(mav0_bodyrate[0], 0); 
    ASSERT_FLOAT_EQ(mav0_bodyrate[1], 0); 
    ASSERT_FLOAT_EQ(mav0_bodyrate[2], 0);  

    // check attitude of mav0
    Eigen::Quaterniond mav0_att = Eigen::Quaterniond::UnitRandom();
    v_drone_cable.at(0).mav_.GetAttitude(mav0_att);
    ASSERT_FLOAT_EQ(mav0_att.x(), 0);     
    ASSERT_FLOAT_EQ(mav0_att.y(), 0);      
    ASSERT_FLOAT_EQ(mav0_att.z(), 0);     
    ASSERT_FLOAT_EQ(mav0_att.w(), 1);      

    // check bodyrate_acc of mav0
    Eigen::Vector3d mav0_bodyrate_acc = Eigen::Vector3d::Random();
    v_drone_cable.at(0).mav_.GetBodyRateAcc(mav0_bodyrate_acc);
    ASSERT_FLOAT_EQ(mav0_bodyrate_acc[0], 0); 
    ASSERT_FLOAT_EQ(mav0_bodyrate_acc[1], 0); 
    ASSERT_FLOAT_EQ(mav0_bodyrate_acc[2], 0);  



    //mav1
    ASSERT_FLOAT_EQ(mav1_post[0], -1); 
    ASSERT_FLOAT_EQ(mav1_post[1], 1); 
    ASSERT_FLOAT_EQ(mav1_post[2], cable_length);  

    // vel of mav1
    Eigen::Vector3d mav1_vel;
    v_drone_cable.at(1).mav_.GetVel(mav1_vel);    
    ASSERT_FLOAT_EQ(mav1_vel[0], 0); 
    ASSERT_FLOAT_EQ(mav1_vel[1], 0); 
    ASSERT_NEAR(mav1_vel[2], 0.0, 1e-10);   

    // acc of mav1
    Eigen::Vector3d mav1_acc = Eigen::Vector3d::Random();
    v_drone_cable.at(1).mav_.GetAcc(mav1_acc);
    ASSERT_FLOAT_EQ(mav1_acc[0], 0); 
    ASSERT_FLOAT_EQ(mav1_acc[1], 0); 
    ASSERT_NEAR(mav1_acc[2], 0, 1e-10); 

    // check bodyrate of mav1
    Eigen::Vector3d mav1_bodyrate = Eigen::Vector3d::Random();
    v_drone_cable.at(1).mav_.GetBodyrate(mav1_bodyrate);
    ASSERT_FLOAT_EQ(mav1_bodyrate[0], 0); 
    ASSERT_FLOAT_EQ(mav1_bodyrate[1], 0); 
    ASSERT_FLOAT_EQ(mav1_bodyrate[2], 0);  

    // check attitude of mav1
    Eigen::Quaterniond mav1_att = Eigen::Quaterniond::UnitRandom();
    v_drone_cable.at(1).mav_.GetAttitude(mav1_att);
    ASSERT_FLOAT_EQ(mav1_att.x(), 0);     
    ASSERT_FLOAT_EQ(mav1_att.y(), 0);      
    ASSERT_FLOAT_EQ(mav1_att.z(), 0);     
    ASSERT_FLOAT_EQ(mav1_att.w(), 1);      

    // check bodyrate_acc of mav1
    Eigen::Vector3d mav1_bodyrate_acc = Eigen::Vector3d::Random();
    v_drone_cable.at(1).mav_.GetBodyRateAcc(mav1_bodyrate_acc);
    ASSERT_FLOAT_EQ(mav1_bodyrate_acc[0], 0); 
    ASSERT_FLOAT_EQ(mav1_bodyrate_acc[1], 0); 
    ASSERT_FLOAT_EQ(mav1_bodyrate_acc[2], 0);      


    //mav2
    ASSERT_FLOAT_EQ(mav2_post[0], -1); 
    ASSERT_FLOAT_EQ(mav2_post[1], -1); 
    ASSERT_FLOAT_EQ(mav2_post[2], cable_length);  

    // vel of mav2
    Eigen::Vector3d mav2_vel;
    v_drone_cable.at(2).mav_.GetVel(mav2_vel);    
    ASSERT_FLOAT_EQ(mav2_vel[0], 0); 
    ASSERT_FLOAT_EQ(mav2_vel[1], 0); 
    ASSERT_NEAR(mav2_vel[2], 0.0, 1e-10);  

    // acc of mav2
    Eigen::Vector3d mav2_acc = Eigen::Vector3d::Random();
    v_drone_cable.at(2).mav_.GetAcc(mav2_acc);
    ASSERT_FLOAT_EQ(mav2_acc[0], 0); 
    ASSERT_FLOAT_EQ(mav2_acc[1], 0); 
    ASSERT_NEAR(mav2_acc[2], 0, 1e-10);   

    // check bodyrate of mav2
    Eigen::Vector3d mav2_bodyrate = Eigen::Vector3d::Random();
    v_drone_cable.at(2).mav_.GetBodyrate(mav2_bodyrate);
    ASSERT_FLOAT_EQ(mav2_bodyrate[0], 0); 
    ASSERT_FLOAT_EQ(mav2_bodyrate[1], 0); 
    ASSERT_FLOAT_EQ(mav2_bodyrate[2], 0);  

    // check attitude of mav2
    Eigen::Quaterniond mav2_att = Eigen::Quaterniond::UnitRandom();
    v_drone_cable.at(2).mav_.GetAttitude(mav2_att);
    ASSERT_FLOAT_EQ(mav2_att.x(), 0);     
    ASSERT_FLOAT_EQ(mav2_att.y(), 0);      
    ASSERT_FLOAT_EQ(mav2_att.z(), 0);     
    ASSERT_FLOAT_EQ(mav2_att.w(), 1);      

    // check bodyrate_acc of mav2
    Eigen::Vector3d mav2_bodyrate_acc = Eigen::Vector3d::Random();
    v_drone_cable.at(2).mav_.GetBodyRateAcc(mav2_bodyrate_acc);
    ASSERT_FLOAT_EQ(mav2_bodyrate_acc[0], 0); 
    ASSERT_FLOAT_EQ(mav2_bodyrate_acc[1], 0); 
    ASSERT_FLOAT_EQ(mav2_bodyrate_acc[2], 0); 

    //mav3
    ASSERT_FLOAT_EQ(mav3_post[0], 1); 
    ASSERT_FLOAT_EQ(mav3_post[1], -1); 
    ASSERT_FLOAT_EQ(mav3_post[2], cable_length);  

    // vel of mav3
    Eigen::Vector3d mav3_vel;
    v_drone_cable.at(3).mav_.GetVel(mav3_vel);    
    ASSERT_FLOAT_EQ(mav3_vel[0], 0); 
    ASSERT_FLOAT_EQ(mav3_vel[1], 0); 
    ASSERT_NEAR(mav3_vel[2], 0.0, 1e-10); 

    // acc of mav3
    Eigen::Vector3d mav3_acc = Eigen::Vector3d::Random();
    v_drone_cable.at(3).mav_.GetAcc(mav3_acc);
    ASSERT_FLOAT_EQ(mav3_acc[0], 0); 
    ASSERT_FLOAT_EQ(mav3_acc[1], 0); 
    ASSERT_NEAR(mav3_acc[2], 0, 1e-10); 

    // check bodyrate of mav3
    Eigen::Vector3d mav3_bodyrate = Eigen::Vector3d::Random();
    v_drone_cable.at(3).mav_.GetBodyrate(mav3_bodyrate);
    ASSERT_FLOAT_EQ(mav3_bodyrate[0], 0); 
    ASSERT_FLOAT_EQ(mav3_bodyrate[1], 0); 
    ASSERT_FLOAT_EQ(mav3_bodyrate[2], 0);  

    // check attitude of mav3
    Eigen::Quaterniond mav3_att = Eigen::Quaterniond::UnitRandom();
    v_drone_cable.at(3).mav_.GetAttitude(mav3_att);
    ASSERT_FLOAT_EQ(mav3_att.x(), 0);     
    ASSERT_FLOAT_EQ(mav3_att.y(), 0);      
    ASSERT_FLOAT_EQ(mav3_att.z(), 0);     
    ASSERT_FLOAT_EQ(mav3_att.w(), 1);      

    // check bodyrate_acc of mav3
    Eigen::Vector3d mav3_bodyrate_acc = Eigen::Vector3d::Random();
    v_drone_cable.at(3).mav_.GetBodyRateAcc(mav3_bodyrate_acc);
    ASSERT_FLOAT_EQ(mav3_bodyrate_acc[0], 0); 
    ASSERT_FLOAT_EQ(mav3_bodyrate_acc[1], 0); 
    ASSERT_FLOAT_EQ(mav3_bodyrate_acc[2], 0);     
}








// // test if hovering in 100 steps
// TEST_F(rotorTMCooperative, checkVerticalStaticEquilibrium100Steps){

//     // set initial posts for mavs and payload
//     ptr_Cooperative->SetPayloadInitPost();

//     // input mav controllers' inputs to hover
//     // 4 mavs + 1 payload =  4 + 1.5 = 5.5
//     // mav thrust = (5.5 * 9.8)/4 = 13.475N

//     Eigen::VectorXd v_mavs_thrusts = Eigen::MatrixXd::Constant(4,1,13.475);
//     std::vector<Eigen::Vector3d> v_mavs_torques(4, Eigen::Vector3d::Zero());


//     // call one step dynamic simulation for MAVs and payload
//     // std::cout<< "fuck point cooperative test 3"<<std::endl;
//     const double dt = 0.01;
//     for(double t=dt ; t<=100*dt ; t+= dt)
//     {
//             // std::cout<< "fuck point cooperative test 1"<<std::endl;
//             ptr_Cooperative->InputControllerInput4MAVs(v_mavs_thrusts, v_mavs_torques);

//             // compute interation wrenches and vars for MAVs and payload
//             // std::cout<< "fuck point cooperative test 2"<<std::endl;
//             ptr_Cooperative->ComputeInteractWrenches();

//             ptr_Cooperative->DoOneStepInt4Robots();
//             // printf("current step is %.3f \n", t);
//     }


//     // payload
//     Eigen::Vector3d payload_post = Eigen::Vector3d::Random();
//     Eigen::Vector3d payload_vel = Eigen::Vector3d::Random();
//     Eigen::Vector3d payload_acc = Eigen::Vector3d::Random();
//     Eigen::Quaterniond payload_attitude = Eigen::Quaterniond::UnitRandom();
//     Eigen::Vector3d payload_bodyrate = Eigen::Vector3d::Random();
//     Eigen::Vector3d payload_bodyrate_acc = Eigen::Vector3d::Random();

//     // post of payload
//     ptr_Cooperative->payload_.GetPosition(payload_post);
//     ASSERT_FLOAT_EQ(payload_post[0], 0); 
//     ASSERT_FLOAT_EQ(payload_post[1], 0); 
//     ASSERT_NEAR(payload_post[2], 0, 1e-10); 

//     // vel of payload
//     ptr_Cooperative->payload_.GetVel(payload_vel);
//     ASSERT_FLOAT_EQ(payload_vel[0], 0); 
//     ASSERT_FLOAT_EQ(payload_vel[1], 0); 
//     ASSERT_NEAR(payload_vel[2], 0, 1e-10);     

//     // acc of payload
//     ptr_Cooperative->payload_.GetAcc(payload_acc);
//     ASSERT_FLOAT_EQ(payload_acc[0], 0); 
//     ASSERT_FLOAT_EQ(payload_acc[1], 0); 
//     ASSERT_NEAR(payload_acc[2], 0, 1e-10);   

//     //  bodyrate of payload
//     ptr_Cooperative->payload_.GetBodyrate(payload_bodyrate);
//     ASSERT_FLOAT_EQ(payload_bodyrate[0], 0); 
//     ASSERT_FLOAT_EQ(payload_bodyrate[1], 0); 
//     ASSERT_FLOAT_EQ(payload_bodyrate[2], 0);  

//     // attitude of payload
//     ptr_Cooperative->payload_.GetAttitude(payload_attitude);
//     ASSERT_FLOAT_EQ(payload_attitude.x(), 0);     
//     ASSERT_FLOAT_EQ(payload_attitude.y(), 0);      
//     ASSERT_FLOAT_EQ(payload_attitude.z(), 0);     
//     ASSERT_FLOAT_EQ(payload_attitude.w(), 1);      

//     // bodyrate_acc of payload
//     ptr_Cooperative->payload_.GetBodyRateAcc(payload_bodyrate_acc);
//     ASSERT_FLOAT_EQ(payload_bodyrate_acc[0], 0); 
//     ASSERT_FLOAT_EQ(payload_bodyrate_acc[1], 0); 
//     ASSERT_FLOAT_EQ(payload_bodyrate_acc[2], 0);  

//     // 
//     Eigen::Vector3d mav0_post{0,0,0};
//     Eigen::Vector3d mav1_post{0,0,0};
//     Eigen::Vector3d mav2_post{0,0,0};
//     Eigen::Vector3d mav3_post{0,0,0};

//     // std::cout<< "fuck point cooperative test 4"<<std::endl;
//     std::vector<UAVCable> v_drone_cable = ptr_Cooperative->v_drone_cable_;

//     v_drone_cable.at(0).mav_.GetPosition(mav0_post);
//     v_drone_cable.at(1).mav_.GetPosition(mav1_post);
//     v_drone_cable.at(2).mav_.GetPosition(mav2_post);    
//     v_drone_cable.at(3).mav_.GetPosition(mav3_post);    

//     double cable_length;
//     ptr_Cooperative->v_drone_cable_.at(0).cable_.GetCableLength(cable_length);

//     // std::cout<<"[----------] test: mav1_init_post  is " << mav1_init_post.transpose()<<std::endl;
//     ASSERT_FLOAT_EQ(mav0_post[0], 1); 
//     ASSERT_FLOAT_EQ(mav0_post[1], 1); 
//     ASSERT_FLOAT_EQ(mav0_post[2], cable_length);  

//     // vel of mav0
//     Eigen::Vector3d mav0_vel;
//     v_drone_cable.at(0).mav_.GetVel(mav0_vel);    
//     ASSERT_NEAR(mav0_vel[0], 0.0, 1e-10); 
//     ASSERT_NEAR(mav0_vel[1], 0.0, 1e-10);
//     ASSERT_NEAR(mav0_vel[2], 0.0, 1e-10);  

//     // acc of mav0
//     Eigen::Vector3d mav0_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(0).mav_.GetAcc(mav0_acc);
//     ASSERT_NEAR(mav0_acc[0], 0, 1e-10);  
//     ASSERT_NEAR(mav0_acc[1], 0, 1e-10);  
//     ASSERT_NEAR(mav0_acc[2], 0, 1e-10);   

//     // check bodyrate of mav0
//     Eigen::Vector3d mav0_bodyrate = Eigen::Vector3d::Random();
//     v_drone_cable.at(0).mav_.GetBodyrate(mav0_bodyrate);
//     ASSERT_FLOAT_EQ(mav0_bodyrate[0], 0); 
//     ASSERT_FLOAT_EQ(mav0_bodyrate[1], 0); 
//     ASSERT_FLOAT_EQ(mav0_bodyrate[2], 0);  

//     // check attitude of mav0
//     Eigen::Quaterniond mav0_att = Eigen::Quaterniond::UnitRandom();
//     v_drone_cable.at(0).mav_.GetAttitude(mav0_att);
//     ASSERT_FLOAT_EQ(mav0_att.x(), 0);     
//     ASSERT_FLOAT_EQ(mav0_att.y(), 0);      
//     ASSERT_FLOAT_EQ(mav0_att.z(), 0);     
//     ASSERT_FLOAT_EQ(mav0_att.w(), 1);      

//     // check bodyrate_acc of mav0
//     Eigen::Vector3d mav0_bodyrate_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(0).mav_.GetBodyRateAcc(mav0_bodyrate_acc);
//     ASSERT_FLOAT_EQ(mav0_bodyrate_acc[0], 0); 
//     ASSERT_FLOAT_EQ(mav0_bodyrate_acc[1], 0); 
//     ASSERT_FLOAT_EQ(mav0_bodyrate_acc[2], 0);  



//     //mav1
//     ASSERT_FLOAT_EQ(mav1_post[0], -1); 
//     ASSERT_FLOAT_EQ(mav1_post[1], 1); 
//     ASSERT_FLOAT_EQ(mav1_post[2], cable_length);  

//     // vel of mav1
//     Eigen::Vector3d mav1_vel;
//     v_drone_cable.at(1).mav_.GetVel(mav1_vel);    
//     ASSERT_NEAR(mav1_vel[0], 0.0, 1e-10); 
//     ASSERT_NEAR(mav1_vel[1], 0.0, 1e-10);
//     ASSERT_NEAR(mav1_vel[2], 0.0, 1e-10);  

//     // acc of mav1
//     Eigen::Vector3d mav1_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(1).mav_.GetAcc(mav1_acc);
//     ASSERT_NEAR(mav1_acc[0], 0, 1e-10);  
//     ASSERT_NEAR(mav1_acc[1], 0, 1e-10);  
//     ASSERT_NEAR(mav1_acc[2], 0, 1e-10);   

//     // check bodyrate of mav1
//     Eigen::Vector3d mav1_bodyrate = Eigen::Vector3d::Random();
//     v_drone_cable.at(1).mav_.GetBodyrate(mav1_bodyrate);
//     ASSERT_FLOAT_EQ(mav1_bodyrate[0], 0); 
//     ASSERT_FLOAT_EQ(mav1_bodyrate[1], 0); 
//     ASSERT_FLOAT_EQ(mav1_bodyrate[2], 0);  

//     // check attitude of mav1
//     Eigen::Quaterniond mav1_att = Eigen::Quaterniond::UnitRandom();
//     v_drone_cable.at(1).mav_.GetAttitude(mav1_att);
//     ASSERT_FLOAT_EQ(mav1_att.x(), 0);     
//     ASSERT_FLOAT_EQ(mav1_att.y(), 0);      
//     ASSERT_FLOAT_EQ(mav1_att.z(), 0);     
//     ASSERT_FLOAT_EQ(mav1_att.w(), 1);      

//     // check bodyrate_acc of mav1
//     Eigen::Vector3d mav1_bodyrate_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(1).mav_.GetBodyRateAcc(mav1_bodyrate_acc);
//     ASSERT_FLOAT_EQ(mav1_bodyrate_acc[0], 0); 
//     ASSERT_FLOAT_EQ(mav1_bodyrate_acc[1], 0); 
//     ASSERT_FLOAT_EQ(mav1_bodyrate_acc[2], 0);      


//     //mav2
//     ASSERT_FLOAT_EQ(mav2_post[0], -1); 
//     ASSERT_FLOAT_EQ(mav2_post[1], -1); 
//     ASSERT_FLOAT_EQ(mav2_post[2], cable_length);  

//     // vel of mav2
//     Eigen::Vector3d mav2_vel;
//     v_drone_cable.at(2).mav_.GetVel(mav2_vel);    
//     ASSERT_NEAR(mav2_vel[0], 0.0, 1e-10); 
//     ASSERT_NEAR(mav2_vel[1], 0.0, 1e-10);
//     ASSERT_NEAR(mav2_vel[2], 0.0, 1e-10);  


//     // acc of mav2
//     Eigen::Vector3d mav2_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(2).mav_.GetAcc(mav2_acc);
//     ASSERT_NEAR(mav2_acc[0], 0, 1e-10);  
//     ASSERT_NEAR(mav2_acc[1], 0, 1e-10);  
//     ASSERT_NEAR(mav2_acc[2], 0, 1e-10);   

//     // check bodyrate of mav2
//     Eigen::Vector3d mav2_bodyrate = Eigen::Vector3d::Random();
//     v_drone_cable.at(2).mav_.GetBodyrate(mav2_bodyrate);
//     ASSERT_FLOAT_EQ(mav2_bodyrate[0], 0); 
//     ASSERT_FLOAT_EQ(mav2_bodyrate[1], 0); 
//     ASSERT_FLOAT_EQ(mav2_bodyrate[2], 0);  

//     // check attitude of mav2
//     Eigen::Quaterniond mav2_att = Eigen::Quaterniond::UnitRandom();
//     v_drone_cable.at(2).mav_.GetAttitude(mav2_att);
//     ASSERT_FLOAT_EQ(mav2_att.x(), 0);     
//     ASSERT_FLOAT_EQ(mav2_att.y(), 0);      
//     ASSERT_FLOAT_EQ(mav2_att.z(), 0);     
//     ASSERT_FLOAT_EQ(mav2_att.w(), 1);      

//     // check bodyrate_acc of mav2
//     Eigen::Vector3d mav2_bodyrate_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(2).mav_.GetBodyRateAcc(mav2_bodyrate_acc);
//     ASSERT_FLOAT_EQ(mav2_bodyrate_acc[0], 0); 
//     ASSERT_FLOAT_EQ(mav2_bodyrate_acc[1], 0); 
//     ASSERT_FLOAT_EQ(mav2_bodyrate_acc[2], 0); 

//     //mav3
//     ASSERT_FLOAT_EQ(mav3_post[0], 1); 
//     ASSERT_FLOAT_EQ(mav3_post[1], -1); 
//     ASSERT_FLOAT_EQ(mav3_post[2], cable_length);  

//     // vel of mav3
//     Eigen::Vector3d mav3_vel;
//     v_drone_cable.at(3).mav_.GetVel(mav3_vel);    
//     ASSERT_NEAR(mav3_vel[0], 0.0, 1e-10); 
//     ASSERT_NEAR(mav3_vel[1], 0.0, 1e-10);
//     ASSERT_NEAR(mav3_vel[2], 0.0, 1e-10);  


//     // acc of mav3
//     Eigen::Vector3d mav3_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(3).mav_.GetAcc(mav3_acc);
//     ASSERT_NEAR(mav3_acc[0], 0, 1e-10);  
//     ASSERT_NEAR(mav3_acc[1], 0, 1e-10);  
//     ASSERT_NEAR(mav3_acc[2], 0, 1e-10);   

//     // check bodyrate of mav3
//     Eigen::Vector3d mav3_bodyrate = Eigen::Vector3d::Random();
//     v_drone_cable.at(3).mav_.GetBodyrate(mav3_bodyrate);
//     ASSERT_FLOAT_EQ(mav3_bodyrate[0], 0); 
//     ASSERT_FLOAT_EQ(mav3_bodyrate[1], 0); 
//     ASSERT_FLOAT_EQ(mav3_bodyrate[2], 0);  

//     // check attitude of mav3
//     Eigen::Quaterniond mav3_att = Eigen::Quaterniond::UnitRandom();
//     v_drone_cable.at(3).mav_.GetAttitude(mav3_att);
//     ASSERT_FLOAT_EQ(mav3_att.x(), 0);     
//     ASSERT_FLOAT_EQ(mav3_att.y(), 0);      
//     ASSERT_FLOAT_EQ(mav3_att.z(), 0);     
//     ASSERT_FLOAT_EQ(mav3_att.w(), 1);      

//     // check bodyrate_acc of mav3
//     Eigen::Vector3d mav3_bodyrate_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(3).mav_.GetBodyRateAcc(mav3_bodyrate_acc);
//     ASSERT_FLOAT_EQ(mav3_bodyrate_acc[0], 0); 
//     ASSERT_FLOAT_EQ(mav3_bodyrate_acc[1], 0); 
//     ASSERT_FLOAT_EQ(mav3_bodyrate_acc[2], 0);     
// }










// // test if hovering in 1000 steps
// TEST_F(rotorTMCooperative, checkVerticalStaticEquilibrium1000Steps){

//     // set initial posts for mavs and payload
//     ptr_Cooperative->SetPayloadInitPost();

//     // input mav controllers' inputs to hover
//     // 4 mavs + 1 payload =  4 + 1.5 = 5.5
//     // mav thrust = (5.5 * 9.8)/4 = 13.475N
//     // mavs' net thrust force = 53.9

//     Eigen::VectorXd v_mavs_thrusts = Eigen::MatrixXd::Constant(4,1,13.475);
//     std::vector<Eigen::Vector3d> v_mavs_torques(4, Eigen::Vector3d::Zero());

//     // std::cout<< "fuck point cooperative test 1"<<std::endl;
//     ptr_Cooperative->InputControllerInput4MAVs(v_mavs_thrusts, v_mavs_torques);

//     // compute interation wrenches and vars for MAVs and payload
//     // std::cout<< "fuck point cooperative test 2"<<std::endl;
//     ptr_Cooperative->ComputeInteractWrenches();

//     // call one step dynamic simulation for MAVs and payload
//     // std::cout<< "fuck point cooperative test 3"<<std::endl;
//     const double dt = 0.01;
//     for(double t=dt ; t<=1000*dt ; t+= dt)
//     {
//             ptr_Cooperative->DoOneStepInt4Robots();
//             // printf("current step is %.3f \n", t);
//     }



//     // payload
//     Eigen::Vector3d payload_post = Eigen::Vector3d::Random();
//     Eigen::Vector3d payload_vel = Eigen::Vector3d::Random();
//     Eigen::Vector3d payload_acc = Eigen::Vector3d::Random();
//     Eigen::Quaterniond payload_attitude = Eigen::Quaterniond::UnitRandom();
//     Eigen::Vector3d payload_bodyrate = Eigen::Vector3d::Random();
//     Eigen::Vector3d payload_bodyrate_acc = Eigen::Vector3d::Random();

//     // post of payload
//     ptr_Cooperative->payload_.GetPosition(payload_post);
//     ASSERT_FLOAT_EQ(payload_post[0], 0); 
//     ASSERT_FLOAT_EQ(payload_post[1], 0); 
//     ASSERT_NEAR(payload_post[2], 0, 1e-10); 

//     // vel of payload
//     ptr_Cooperative->payload_.GetVel(payload_vel);
//     ASSERT_FLOAT_EQ(payload_vel[0], 0); 
//     ASSERT_FLOAT_EQ(payload_vel[1], 0); 
//     ASSERT_NEAR(payload_vel[2], 0, 1e-10);     

//     // acc of payload
//     ptr_Cooperative->payload_.GetAcc(payload_acc);
//     ASSERT_FLOAT_EQ(payload_acc[0], 0); 
//     ASSERT_FLOAT_EQ(payload_acc[1], 0); 
//     ASSERT_NEAR(payload_acc[2], 0, 1e-10);   

//     //  bodyrate of payload
//     ptr_Cooperative->payload_.GetBodyrate(payload_bodyrate);
//     ASSERT_FLOAT_EQ(payload_bodyrate[0], 0); 
//     ASSERT_FLOAT_EQ(payload_bodyrate[1], 0); 
//     ASSERT_FLOAT_EQ(payload_bodyrate[2], 0);  

//     // attitude of payload
//     ptr_Cooperative->payload_.GetAttitude(payload_attitude);
//     ASSERT_FLOAT_EQ(payload_attitude.x(), 0);     
//     ASSERT_FLOAT_EQ(payload_attitude.y(), 0);      
//     ASSERT_FLOAT_EQ(payload_attitude.z(), 0);     
//     ASSERT_FLOAT_EQ(payload_attitude.w(), 1);      

//     // bodyrate_acc of payload
//     ptr_Cooperative->payload_.GetBodyRateAcc(payload_bodyrate_acc);
//     ASSERT_FLOAT_EQ(payload_bodyrate_acc[0], 0); 
//     ASSERT_FLOAT_EQ(payload_bodyrate_acc[1], 0); 
//     ASSERT_FLOAT_EQ(payload_bodyrate_acc[2], 0);  

//     // mavs
//     Eigen::Vector3d mav0_post{0,0,0};
//     Eigen::Vector3d mav1_post{0,0,0};
//     Eigen::Vector3d mav2_post{0,0,0};
//     Eigen::Vector3d mav3_post{0,0,0};

//     // std::cout<< "fuck point cooperative test 4"<<std::endl;
//     std::vector<UAVCable> v_drone_cable = ptr_Cooperative->v_drone_cable_;

//     v_drone_cable.at(0).mav_.GetPosition(mav0_post);
//     v_drone_cable.at(1).mav_.GetPosition(mav1_post);
//     v_drone_cable.at(2).mav_.GetPosition(mav2_post);    
//     v_drone_cable.at(3).mav_.GetPosition(mav3_post);    

//     double cable_length;
//     ptr_Cooperative->v_drone_cable_.at(0).cable_.GetCableLength(cable_length);

//     // std::cout<<"[----------] test: mav1_init_post  is " << mav1_init_post.transpose()<<std::endl;
//     ASSERT_FLOAT_EQ(mav0_post[0], 1); 
//     ASSERT_FLOAT_EQ(mav0_post[1], 1); 
//     ASSERT_FLOAT_EQ(mav0_post[2], cable_length);  

//     // vel of mav0
//     Eigen::Vector3d mav0_vel;
//     v_drone_cable.at(0).mav_.GetVel(mav0_vel);    
//     ASSERT_NEAR(mav0_vel[0], 0.0, 1e-10); 
//     ASSERT_NEAR(mav0_vel[1], 0.0, 1e-10);
//     ASSERT_NEAR(mav0_vel[2], 0.0, 1e-10);  

//     // acc of mav0
//     Eigen::Vector3d mav0_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(0).mav_.GetAcc(mav0_acc);
//     ASSERT_NEAR(mav0_acc[0], 0, 1e-10);  
//     ASSERT_NEAR(mav0_acc[1], 0, 1e-10);  
//     ASSERT_NEAR(mav0_acc[2], 0, 1e-10);   

//     // check bodyrate of mav0
//     Eigen::Vector3d mav0_bodyrate = Eigen::Vector3d::Random();
//     v_drone_cable.at(0).mav_.GetBodyrate(mav0_bodyrate);
//     ASSERT_FLOAT_EQ(mav0_bodyrate[0], 0); 
//     ASSERT_FLOAT_EQ(mav0_bodyrate[1], 0); 
//     ASSERT_FLOAT_EQ(mav0_bodyrate[2], 0);  

//     // check attitude of mav0
//     Eigen::Quaterniond mav0_att = Eigen::Quaterniond::UnitRandom();
//     v_drone_cable.at(0).mav_.GetAttitude(mav0_att);
//     ASSERT_FLOAT_EQ(mav0_att.x(), 0);     
//     ASSERT_FLOAT_EQ(mav0_att.y(), 0);      
//     ASSERT_FLOAT_EQ(mav0_att.z(), 0);     
//     ASSERT_FLOAT_EQ(mav0_att.w(), 1);      

//     // check bodyrate_acc of mav0
//     Eigen::Vector3d mav0_bodyrate_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(0).mav_.GetBodyRateAcc(mav0_bodyrate_acc);
//     ASSERT_FLOAT_EQ(mav0_bodyrate_acc[0], 0); 
//     ASSERT_FLOAT_EQ(mav0_bodyrate_acc[1], 0); 
//     ASSERT_FLOAT_EQ(mav0_bodyrate_acc[2], 0);  



//     //mav1
//     ASSERT_FLOAT_EQ(mav1_post[0], -1); 
//     ASSERT_FLOAT_EQ(mav1_post[1], 1); 
//     ASSERT_FLOAT_EQ(mav1_post[2], cable_length);  

//     // vel of mav1
//     Eigen::Vector3d mav1_vel;
//     v_drone_cable.at(1).mav_.GetVel(mav1_vel);    
//     ASSERT_NEAR(mav1_vel[0], 0.0, 1e-10); 
//     ASSERT_NEAR(mav1_vel[1], 0.0, 1e-10);
//     ASSERT_NEAR(mav1_vel[2], 0.0, 1e-10);  

//     // acc of mav1
//     Eigen::Vector3d mav1_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(1).mav_.GetAcc(mav1_acc);
//     ASSERT_NEAR(mav1_acc[0], 0, 1e-10);  
//     ASSERT_NEAR(mav1_acc[1], 0, 1e-10);  
//     ASSERT_NEAR(mav1_acc[2], 0, 1e-10);   

//     // check bodyrate of mav1
//     Eigen::Vector3d mav1_bodyrate = Eigen::Vector3d::Random();
//     v_drone_cable.at(1).mav_.GetBodyrate(mav1_bodyrate);
//     ASSERT_FLOAT_EQ(mav1_bodyrate[0], 0); 
//     ASSERT_FLOAT_EQ(mav1_bodyrate[1], 0); 
//     ASSERT_FLOAT_EQ(mav1_bodyrate[2], 0);  

//     // check attitude of mav1
//     Eigen::Quaterniond mav1_att = Eigen::Quaterniond::UnitRandom();
//     v_drone_cable.at(1).mav_.GetAttitude(mav1_att);
//     ASSERT_FLOAT_EQ(mav1_att.x(), 0);     
//     ASSERT_FLOAT_EQ(mav1_att.y(), 0);      
//     ASSERT_FLOAT_EQ(mav1_att.z(), 0);     
//     ASSERT_FLOAT_EQ(mav1_att.w(), 1);      

//     // check bodyrate_acc of mav1
//     Eigen::Vector3d mav1_bodyrate_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(1).mav_.GetBodyRateAcc(mav1_bodyrate_acc);
//     ASSERT_FLOAT_EQ(mav1_bodyrate_acc[0], 0); 
//     ASSERT_FLOAT_EQ(mav1_bodyrate_acc[1], 0); 
//     ASSERT_FLOAT_EQ(mav1_bodyrate_acc[2], 0);      


//     //mav2
//     ASSERT_FLOAT_EQ(mav2_post[0], -1); 
//     ASSERT_FLOAT_EQ(mav2_post[1], -1); 
//     ASSERT_FLOAT_EQ(mav2_post[2], cable_length);  

//     // vel of mav2
//     Eigen::Vector3d mav2_vel;
//     v_drone_cable.at(2).mav_.GetVel(mav2_vel);    
//     ASSERT_NEAR(mav2_vel[0], 0.0, 1e-10); 
//     ASSERT_NEAR(mav2_vel[1], 0.0, 1e-10);
//     ASSERT_NEAR(mav2_vel[2], 0.0, 1e-10);  


//     // acc of mav2
//     Eigen::Vector3d mav2_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(2).mav_.GetAcc(mav2_acc);
//     ASSERT_NEAR(mav2_acc[0], 0, 1e-10);  
//     ASSERT_NEAR(mav2_acc[1], 0, 1e-10);  
//     ASSERT_NEAR(mav2_acc[2], 0, 1e-10);   

//     // check bodyrate of mav2
//     Eigen::Vector3d mav2_bodyrate = Eigen::Vector3d::Random();
//     v_drone_cable.at(2).mav_.GetBodyrate(mav2_bodyrate);
//     ASSERT_FLOAT_EQ(mav2_bodyrate[0], 0); 
//     ASSERT_FLOAT_EQ(mav2_bodyrate[1], 0); 
//     ASSERT_FLOAT_EQ(mav2_bodyrate[2], 0);  

//     // check attitude of mav2
//     Eigen::Quaterniond mav2_att = Eigen::Quaterniond::UnitRandom();
//     v_drone_cable.at(2).mav_.GetAttitude(mav2_att);
//     ASSERT_FLOAT_EQ(mav2_att.x(), 0);     
//     ASSERT_FLOAT_EQ(mav2_att.y(), 0);      
//     ASSERT_FLOAT_EQ(mav2_att.z(), 0);     
//     ASSERT_FLOAT_EQ(mav2_att.w(), 1);      

//     // check bodyrate_acc of mav2
//     Eigen::Vector3d mav2_bodyrate_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(2).mav_.GetBodyRateAcc(mav2_bodyrate_acc);
//     ASSERT_FLOAT_EQ(mav2_bodyrate_acc[0], 0); 
//     ASSERT_FLOAT_EQ(mav2_bodyrate_acc[1], 0); 
//     ASSERT_FLOAT_EQ(mav2_bodyrate_acc[2], 0); 

//     //mav3
//     ASSERT_FLOAT_EQ(mav3_post[0], 1); 
//     ASSERT_FLOAT_EQ(mav3_post[1], -1); 
//     ASSERT_FLOAT_EQ(mav3_post[2], cable_length);  

//     // vel of mav3
//     Eigen::Vector3d mav3_vel;
//     v_drone_cable.at(3).mav_.GetVel(mav3_vel);    
//     ASSERT_NEAR(mav3_vel[0], 0.0, 1e-10); 
//     ASSERT_NEAR(mav3_vel[1], 0.0, 1e-10);
//     ASSERT_NEAR(mav3_vel[2], 0.0, 1e-10);  


//     // acc of mav3
//     Eigen::Vector3d mav3_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(3).mav_.GetAcc(mav3_acc);
//     ASSERT_NEAR(mav3_acc[0], 0, 1e-10);  
//     ASSERT_NEAR(mav3_acc[1], 0, 1e-10);  
//     ASSERT_NEAR(mav3_acc[2], 0, 1e-10);   

//     // check bodyrate of mav3
//     Eigen::Vector3d mav3_bodyrate = Eigen::Vector3d::Random();
//     v_drone_cable.at(3).mav_.GetBodyrate(mav3_bodyrate);
//     ASSERT_FLOAT_EQ(mav3_bodyrate[0], 0); 
//     ASSERT_FLOAT_EQ(mav3_bodyrate[1], 0); 
//     ASSERT_FLOAT_EQ(mav3_bodyrate[2], 0);  

//     // check attitude of mav3
//     Eigen::Quaterniond mav3_att = Eigen::Quaterniond::UnitRandom();
//     v_drone_cable.at(3).mav_.GetAttitude(mav3_att);
//     ASSERT_FLOAT_EQ(mav3_att.x(), 0);     
//     ASSERT_FLOAT_EQ(mav3_att.y(), 0);      
//     ASSERT_FLOAT_EQ(mav3_att.z(), 0);     
//     ASSERT_FLOAT_EQ(mav3_att.w(), 1);      

//     // check bodyrate_acc of mav3
//     Eigen::Vector3d mav3_bodyrate_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(3).mav_.GetBodyRateAcc(mav3_bodyrate_acc);
//     ASSERT_FLOAT_EQ(mav3_bodyrate_acc[0], 0); 
//     ASSERT_FLOAT_EQ(mav3_bodyrate_acc[1], 0); 
//     ASSERT_FLOAT_EQ(mav3_bodyrate_acc[2], 0);     
// }








// // test if hovering in 1000 steps
// TEST_F(rotorTMCooperative, checkVerticalInitPostStaticEquilibrium1000Steps){

//     Eigen::Vector3d payload_init_post{1,0,0};
//     // set initial posts for mavs and payload
//     ptr_Cooperative->SetPayloadInitPost(payload_init_post);

//     // input mav controllers' inputs to hover
//     // 4 mavs + 1 payload =  4 + 1.5 = 5.5
//     // mav thrust = (5.5 * 9.8)/4 = 13.475N
//     // mavs' net thrust force = 53.9

//     Eigen::VectorXd v_mavs_thrusts = Eigen::MatrixXd::Constant(4,1,13.475);
//     std::vector<Eigen::Vector3d> v_mavs_torques(4, Eigen::Vector3d::Zero());

//     // std::cout<< "fuck point cooperative test 1"<<std::endl;
//     ptr_Cooperative->InputControllerInput4MAVs(v_mavs_thrusts, v_mavs_torques);

//     // compute interation wrenches and vars for MAVs and payload
//     // std::cout<< "fuck point cooperative test 2"<<std::endl;
//     ptr_Cooperative->ComputeInteractWrenches();

//     // call one step dynamic simulation for MAVs and payload
//     // std::cout<< "fuck point cooperative test 3"<<std::endl;
//     const double dt = 0.01;
//     for(double t=dt ; t<=1000*dt ; t+= dt)
//     {
//             ptr_Cooperative->DoOneStepInt4Robots();
//             // printf("current step is %.3f \n", t);
//     }



//     // payload
//     Eigen::Vector3d payload_post = Eigen::Vector3d::Random();
//     Eigen::Vector3d payload_vel = Eigen::Vector3d::Random();
//     Eigen::Vector3d payload_acc = Eigen::Vector3d::Random();
//     Eigen::Quaterniond payload_attitude = Eigen::Quaterniond::UnitRandom();
//     Eigen::Vector3d payload_bodyrate = Eigen::Vector3d::Random();
//     Eigen::Vector3d payload_bodyrate_acc = Eigen::Vector3d::Random();

//     // post of payload
//     ptr_Cooperative->payload_.GetPosition(payload_post);

//     ASSERT_FLOAT_EQ(payload_post[0], payload_init_post[0]); 
//     ASSERT_FLOAT_EQ(payload_post[1], payload_init_post[1]); 
//     ASSERT_NEAR(payload_post[2], payload_init_post[2], 1e-10); 

//     // vel of payload
//     ptr_Cooperative->payload_.GetVel(payload_vel);
//     ASSERT_FLOAT_EQ(payload_vel[0], 0); 
//     ASSERT_FLOAT_EQ(payload_vel[1], 0); 
//     ASSERT_NEAR(payload_vel[2], 0, 1e-10);     

//     // acc of payload
//     ptr_Cooperative->payload_.GetAcc(payload_acc);
//     ASSERT_FLOAT_EQ(payload_acc[0], 0); 
//     ASSERT_FLOAT_EQ(payload_acc[1], 0); 
//     ASSERT_NEAR(payload_acc[2], 0, 1e-10);   

//     //  bodyrate of payload
//     ptr_Cooperative->payload_.GetBodyrate(payload_bodyrate);
//     ASSERT_FLOAT_EQ(payload_bodyrate[0], 0); 
//     ASSERT_FLOAT_EQ(payload_bodyrate[1], 0); 
//     ASSERT_FLOAT_EQ(payload_bodyrate[2], 0);  

//     // attitude of payload
//     ptr_Cooperative->payload_.GetAttitude(payload_attitude);
//     ASSERT_FLOAT_EQ(payload_attitude.x(), 0);     
//     ASSERT_FLOAT_EQ(payload_attitude.y(), 0);      
//     ASSERT_FLOAT_EQ(payload_attitude.z(), 0);     
//     ASSERT_FLOAT_EQ(payload_attitude.w(), 1);      

//     // bodyrate_acc of payload
//     ptr_Cooperative->payload_.GetBodyRateAcc(payload_bodyrate_acc);
//     ASSERT_FLOAT_EQ(payload_bodyrate_acc[0], 0); 
//     ASSERT_FLOAT_EQ(payload_bodyrate_acc[1], 0); 
//     ASSERT_FLOAT_EQ(payload_bodyrate_acc[2], 0);  
    
//     // mavs
//     Eigen::Vector3d mav0_post{0,0,0};
//     Eigen::Vector3d mav1_post{0,0,0};
//     Eigen::Vector3d mav2_post{0,0,0};
//     Eigen::Vector3d mav3_post{0,0,0};

//     // std::cout<< "fuck point cooperative test 4"<<std::endl;
//     std::vector<UAVCable> v_drone_cable = ptr_Cooperative->v_drone_cable_;

//     v_drone_cable.at(0).mav_.GetPosition(mav0_post);
//     v_drone_cable.at(1).mav_.GetPosition(mav1_post);
//     v_drone_cable.at(2).mav_.GetPosition(mav2_post);    
//     v_drone_cable.at(3).mav_.GetPosition(mav3_post);    

//     double cable_length;
//     ptr_Cooperative->v_drone_cable_.at(0).cable_.GetCableLength(cable_length);

//     // std::cout<<"[----------] test: mav1_init_post  is " << mav1_init_post.transpose()<<std::endl;
//     ASSERT_FLOAT_EQ(mav0_post[0], 1); 
//     ASSERT_FLOAT_EQ(mav0_post[1], 1); 
//     ASSERT_FLOAT_EQ(mav0_post[2], cable_length);  

//     // vel of mav0
//     Eigen::Vector3d mav0_vel;
//     v_drone_cable.at(0).mav_.GetVel(mav0_vel);    
//     ASSERT_NEAR(mav0_vel[0], 0.0, 1e-10); 
//     ASSERT_NEAR(mav0_vel[1], 0.0, 1e-10);
//     ASSERT_NEAR(mav0_vel[2], 0.0, 1e-10);  

//     // acc of mav0
//     Eigen::Vector3d mav0_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(0).mav_.GetAcc(mav0_acc);
//     ASSERT_NEAR(mav0_acc[0], 0, 1e-10);  
//     ASSERT_NEAR(mav0_acc[1], 0, 1e-10);  
//     ASSERT_NEAR(mav0_acc[2], 0, 1e-10);   

//     // check bodyrate of mav0
//     Eigen::Vector3d mav0_bodyrate = Eigen::Vector3d::Random();
//     v_drone_cable.at(0).mav_.GetBodyrate(mav0_bodyrate);
//     ASSERT_FLOAT_EQ(mav0_bodyrate[0], 0); 
//     ASSERT_FLOAT_EQ(mav0_bodyrate[1], 0); 
//     ASSERT_FLOAT_EQ(mav0_bodyrate[2], 0);  

//     // check attitude of mav0
//     Eigen::Quaterniond mav0_att = Eigen::Quaterniond::UnitRandom();
//     v_drone_cable.at(0).mav_.GetAttitude(mav0_att);
//     ASSERT_FLOAT_EQ(mav0_att.x(), 0);     
//     ASSERT_FLOAT_EQ(mav0_att.y(), 0);      
//     ASSERT_FLOAT_EQ(mav0_att.z(), 0);     
//     ASSERT_FLOAT_EQ(mav0_att.w(), 1);      

//     // check bodyrate_acc of mav0
//     Eigen::Vector3d mav0_bodyrate_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(0).mav_.GetBodyRateAcc(mav0_bodyrate_acc);
//     ASSERT_FLOAT_EQ(mav0_bodyrate_acc[0], 0); 
//     ASSERT_FLOAT_EQ(mav0_bodyrate_acc[1], 0); 
//     ASSERT_FLOAT_EQ(mav0_bodyrate_acc[2], 0);  



//     //mav1
//     ASSERT_FLOAT_EQ(mav1_post[0], -1); 
//     ASSERT_FLOAT_EQ(mav1_post[1], 1); 
//     ASSERT_FLOAT_EQ(mav1_post[2], cable_length);  

//     // vel of mav1
//     Eigen::Vector3d mav1_vel;
//     v_drone_cable.at(1).mav_.GetVel(mav1_vel);    
//     ASSERT_NEAR(mav1_vel[0], 0.0, 1e-10); 
//     ASSERT_NEAR(mav1_vel[1], 0.0, 1e-10);
//     ASSERT_NEAR(mav1_vel[2], 0.0, 1e-10);  

//     // acc of mav1
//     Eigen::Vector3d mav1_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(1).mav_.GetAcc(mav1_acc);
//     ASSERT_NEAR(mav1_acc[0], 0, 1e-10);  
//     ASSERT_NEAR(mav1_acc[1], 0, 1e-10);  
//     ASSERT_NEAR(mav1_acc[2], 0, 1e-10);   

//     // check bodyrate of mav1
//     Eigen::Vector3d mav1_bodyrate = Eigen::Vector3d::Random();
//     v_drone_cable.at(1).mav_.GetBodyrate(mav1_bodyrate);
//     ASSERT_FLOAT_EQ(mav1_bodyrate[0], 0); 
//     ASSERT_FLOAT_EQ(mav1_bodyrate[1], 0); 
//     ASSERT_FLOAT_EQ(mav1_bodyrate[2], 0);  

//     // check attitude of mav1
//     Eigen::Quaterniond mav1_att = Eigen::Quaterniond::UnitRandom();
//     v_drone_cable.at(1).mav_.GetAttitude(mav1_att);
//     ASSERT_FLOAT_EQ(mav1_att.x(), 0);     
//     ASSERT_FLOAT_EQ(mav1_att.y(), 0);      
//     ASSERT_FLOAT_EQ(mav1_att.z(), 0);     
//     ASSERT_FLOAT_EQ(mav1_att.w(), 1);      

//     // check bodyrate_acc of mav1
//     Eigen::Vector3d mav1_bodyrate_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(1).mav_.GetBodyRateAcc(mav1_bodyrate_acc);
//     ASSERT_FLOAT_EQ(mav1_bodyrate_acc[0], 0); 
//     ASSERT_FLOAT_EQ(mav1_bodyrate_acc[1], 0); 
//     ASSERT_FLOAT_EQ(mav1_bodyrate_acc[2], 0);      


//     //mav2
//     ASSERT_FLOAT_EQ(mav2_post[0], -1); 
//     ASSERT_FLOAT_EQ(mav2_post[1], -1); 
//     ASSERT_FLOAT_EQ(mav2_post[2], cable_length);  

//     // vel of mav2
//     Eigen::Vector3d mav2_vel;
//     v_drone_cable.at(2).mav_.GetVel(mav2_vel);    
//     ASSERT_NEAR(mav2_vel[0], 0.0, 1e-10); 
//     ASSERT_NEAR(mav2_vel[1], 0.0, 1e-10);
//     ASSERT_NEAR(mav2_vel[2], 0.0, 1e-10);  


//     // acc of mav2
//     Eigen::Vector3d mav2_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(2).mav_.GetAcc(mav2_acc);
//     ASSERT_NEAR(mav2_acc[0], 0, 1e-10);  
//     ASSERT_NEAR(mav2_acc[1], 0, 1e-10);  
//     ASSERT_NEAR(mav2_acc[2], 0, 1e-10);   

//     // check bodyrate of mav2
//     Eigen::Vector3d mav2_bodyrate = Eigen::Vector3d::Random();
//     v_drone_cable.at(2).mav_.GetBodyrate(mav2_bodyrate);
//     ASSERT_FLOAT_EQ(mav2_bodyrate[0], 0); 
//     ASSERT_FLOAT_EQ(mav2_bodyrate[1], 0); 
//     ASSERT_FLOAT_EQ(mav2_bodyrate[2], 0);  

//     // check attitude of mav2
//     Eigen::Quaterniond mav2_att = Eigen::Quaterniond::UnitRandom();
//     v_drone_cable.at(2).mav_.GetAttitude(mav2_att);
//     ASSERT_FLOAT_EQ(mav2_att.x(), 0);     
//     ASSERT_FLOAT_EQ(mav2_att.y(), 0);      
//     ASSERT_FLOAT_EQ(mav2_att.z(), 0);     
//     ASSERT_FLOAT_EQ(mav2_att.w(), 1);      

//     // check bodyrate_acc of mav2
//     Eigen::Vector3d mav2_bodyrate_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(2).mav_.GetBodyRateAcc(mav2_bodyrate_acc);
//     ASSERT_FLOAT_EQ(mav2_bodyrate_acc[0], 0); 
//     ASSERT_FLOAT_EQ(mav2_bodyrate_acc[1], 0); 
//     ASSERT_FLOAT_EQ(mav2_bodyrate_acc[2], 0); 

//     //mav3
//     ASSERT_FLOAT_EQ(mav3_post[0], 1); 
//     ASSERT_FLOAT_EQ(mav3_post[1], -1); 
//     ASSERT_FLOAT_EQ(mav3_post[2], cable_length);  

//     // vel of mav3
//     Eigen::Vector3d mav3_vel;
//     v_drone_cable.at(3).mav_.GetVel(mav3_vel);    
//     ASSERT_NEAR(mav3_vel[0], 0.0, 1e-10); 
//     ASSERT_NEAR(mav3_vel[1], 0.0, 1e-10);
//     ASSERT_NEAR(mav3_vel[2], 0.0, 1e-10);  


//     // acc of mav3
//     Eigen::Vector3d mav3_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(3).mav_.GetAcc(mav3_acc);
//     ASSERT_NEAR(mav3_acc[0], 0, 1e-10);  
//     ASSERT_NEAR(mav3_acc[1], 0, 1e-10);  
//     ASSERT_NEAR(mav3_acc[2], 0, 1e-10);   

//     // check bodyrate of mav3
//     Eigen::Vector3d mav3_bodyrate = Eigen::Vector3d::Random();
//     v_drone_cable.at(3).mav_.GetBodyrate(mav3_bodyrate);
//     ASSERT_FLOAT_EQ(mav3_bodyrate[0], 0); 
//     ASSERT_FLOAT_EQ(mav3_bodyrate[1], 0); 
//     ASSERT_FLOAT_EQ(mav3_bodyrate[2], 0);  

//     // check attitude of mav3
//     Eigen::Quaterniond mav3_att = Eigen::Quaterniond::UnitRandom();
//     v_drone_cable.at(3).mav_.GetAttitude(mav3_att);
//     ASSERT_FLOAT_EQ(mav3_att.x(), 0);     
//     ASSERT_FLOAT_EQ(mav3_att.y(), 0);      
//     ASSERT_FLOAT_EQ(mav3_att.z(), 0);     
//     ASSERT_FLOAT_EQ(mav3_att.w(), 1);      

//     // check bodyrate_acc of mav3
//     Eigen::Vector3d mav3_bodyrate_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(3).mav_.GetBodyRateAcc(mav3_bodyrate_acc);
//     ASSERT_FLOAT_EQ(mav3_bodyrate_acc[0], 0); 
//     ASSERT_FLOAT_EQ(mav3_bodyrate_acc[1], 0); 
//     ASSERT_FLOAT_EQ(mav3_bodyrate_acc[2], 0);     
// }






















// // Dynamic simulation

// // const acc for 1 step
// TEST_F(rotorTMCooperative, checkVerticalConstAcc1Step){

//     // set initial posts for mavs and payload
//     ptr_Cooperative->SetPayloadInitPost();

//     // input mav controllers' inputs to produce 1m/s^2 along vertical direction
//     // 4 mavs + 1 payload =  4 + 1.5 = 5.5
//     // mav thrust = (5.5 * (1+9.8) )/4 = 14.85N
//     // mavs' net thrust force = 59.4

//     Eigen::VectorXd v_mavs_thrusts = Eigen::MatrixXd::Constant(4,1,14.85);
//     std::vector<Eigen::Vector3d> v_mavs_torques(4, Eigen::Vector3d::Zero());

//     // std::cout<< "fuck point cooperative test 1"<<std::endl;
//     ptr_Cooperative->InputControllerInput4MAVs(v_mavs_thrusts, v_mavs_torques);

//     // compute interation wrenches and vars for MAVs and payload
//     // std::cout<< "fuck point cooperative test 2"<<std::endl;
//     ptr_Cooperative->ComputeInteractWrenches();

//     // call one step dynamic simulation for MAVs and payload
//     // std::cout<< "fuck point cooperative test 3"<<std::endl;
//     ptr_Cooperative->DoOneStepInt4Robots();
//             // printf("current step is %.3f \n", t);
    

//     // 
//     Eigen::Vector3d mav0_post{0,0,0};
//     Eigen::Vector3d mav1_post{0,0,0};
//     Eigen::Vector3d mav2_post{0,0,0};
//     Eigen::Vector3d mav3_post{0,0,0};

//     std::cout<< "fuck point cooperative test 4"<<std::endl;
//     std::vector<UAVCable> v_drone_cable = ptr_Cooperative->v_drone_cable_;

//     v_drone_cable.at(0).mav_.GetPosition(mav0_post);
//     v_drone_cable.at(1).mav_.GetPosition(mav1_post);
//     v_drone_cable.at(2).mav_.GetPosition(mav2_post);    
//     v_drone_cable.at(3).mav_.GetPosition(mav3_post);    

//     double cable_length;
//     ptr_Cooperative->v_drone_cable_.at(0).cable_.GetCableLength(cable_length);

//     // std::cout<<"[----------] test: mav1_init_post  is " << mav1_init_post.transpose()<<std::endl;
//     ASSERT_FLOAT_EQ(mav0_post[0], 1); 
//     ASSERT_FLOAT_EQ(mav0_post[1], 1); 
//     ASSERT_FLOAT_EQ(mav0_post[2], cable_length+0.5*1*pow(0.01,2));  

//     // vel of mav0
//     Eigen::Vector3d mav0_vel;
//     v_drone_cable.at(0).mav_.GetVel(mav0_vel);    
//     ASSERT_NEAR(mav0_vel[0], 0.0, 1e-10); 
//     ASSERT_NEAR(mav0_vel[1], 0.0, 1e-10);
//     ASSERT_NEAR(mav0_vel[2], 0.01, 1e-10);  

//     // acc of mav0
//     Eigen::Vector3d mav0_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(0).mav_.GetAcc(mav0_acc);
//     ASSERT_NEAR(mav0_acc[0], 0, 1e-10);  
//     ASSERT_NEAR(mav0_acc[1], 0, 1e-10);  
//     ASSERT_NEAR(mav0_acc[2], 0, 1);   

//     // check bodyrate of mav0
//     Eigen::Vector3d mav0_bodyrate = Eigen::Vector3d::Random();
//     v_drone_cable.at(0).mav_.GetBodyrate(mav0_bodyrate);
//     ASSERT_FLOAT_EQ(mav0_bodyrate[0], 0); 
//     ASSERT_FLOAT_EQ(mav0_bodyrate[1], 0); 
//     ASSERT_FLOAT_EQ(mav0_bodyrate[2], 0);  

//     // check attitude of mav0
//     Eigen::Quaterniond mav0_att = Eigen::Quaterniond::UnitRandom();
//     v_drone_cable.at(0).mav_.GetAttitude(mav0_att);
//     ASSERT_FLOAT_EQ(mav0_att.x(), 0);     
//     ASSERT_FLOAT_EQ(mav0_att.y(), 0);      
//     ASSERT_FLOAT_EQ(mav0_att.z(), 0);     
//     ASSERT_FLOAT_EQ(mav0_att.w(), 1);      

//     // check bodyrate_acc of mav0
//     Eigen::Vector3d mav0_bodyrate_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(0).mav_.GetBodyRateAcc(mav0_bodyrate_acc);
//     ASSERT_FLOAT_EQ(mav0_bodyrate_acc[0], 0); 
//     ASSERT_FLOAT_EQ(mav0_bodyrate_acc[1], 0); 
//     ASSERT_FLOAT_EQ(mav0_bodyrate_acc[2], 0);  



//     //mav1
//     ASSERT_FLOAT_EQ(mav1_post[0], -1); 
//     ASSERT_FLOAT_EQ(mav1_post[1], 1); 
//     ASSERT_FLOAT_EQ(mav1_post[2], cable_length+0.5*1*pow(0.01,2));  

//     // vel of mav1
//     Eigen::Vector3d mav1_vel;
//     v_drone_cable.at(1).mav_.GetVel(mav1_vel);    
//     ASSERT_NEAR(mav1_vel[0], 0.0, 1e-10); 
//     ASSERT_NEAR(mav1_vel[1], 0.0, 1e-10);
//     ASSERT_NEAR(mav1_vel[2], 0.01, 1e-10);  

//     // acc of mav1
//     Eigen::Vector3d mav1_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(1).mav_.GetAcc(mav1_acc);
//     ASSERT_NEAR(mav1_acc[0], 0, 1e-10);  
//     ASSERT_NEAR(mav1_acc[1], 0, 1e-10);  
//     ASSERT_NEAR(mav1_acc[2], 0, 1);   

//     // check bodyrate of mav1
//     Eigen::Vector3d mav1_bodyrate = Eigen::Vector3d::Random();
//     v_drone_cable.at(1).mav_.GetBodyrate(mav1_bodyrate);
//     ASSERT_FLOAT_EQ(mav1_bodyrate[0], 0); 
//     ASSERT_FLOAT_EQ(mav1_bodyrate[1], 0); 
//     ASSERT_FLOAT_EQ(mav1_bodyrate[2], 0);  

//     // check attitude of mav1
//     Eigen::Quaterniond mav1_att = Eigen::Quaterniond::UnitRandom();
//     v_drone_cable.at(1).mav_.GetAttitude(mav1_att);
//     ASSERT_FLOAT_EQ(mav1_att.x(), 0);     
//     ASSERT_FLOAT_EQ(mav1_att.y(), 0);      
//     ASSERT_FLOAT_EQ(mav1_att.z(), 0);     
//     ASSERT_FLOAT_EQ(mav1_att.w(), 1);      

//     // check bodyrate_acc of mav1
//     Eigen::Vector3d mav1_bodyrate_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(1).mav_.GetBodyRateAcc(mav1_bodyrate_acc);
//     ASSERT_FLOAT_EQ(mav1_bodyrate_acc[0], 0); 
//     ASSERT_FLOAT_EQ(mav1_bodyrate_acc[1], 0); 
//     ASSERT_FLOAT_EQ(mav1_bodyrate_acc[2], 0);      


//     //mav2
//     ASSERT_FLOAT_EQ(mav2_post[0], -1); 
//     ASSERT_FLOAT_EQ(mav2_post[1], -1); 
//     ASSERT_FLOAT_EQ(mav2_post[2], cable_length+0.5*1*pow(0.01,2));  

//     // vel of mav2
//     Eigen::Vector3d mav2_vel;
//     v_drone_cable.at(2).mav_.GetVel(mav2_vel);    
//     ASSERT_NEAR(mav2_vel[0], 0.0, 1e-10); 
//     ASSERT_NEAR(mav2_vel[1], 0.0, 1e-10);
//     ASSERT_NEAR(mav2_vel[2], 0.01, 1e-10);  


//     // acc of mav2
//     Eigen::Vector3d mav2_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(2).mav_.GetAcc(mav2_acc);
//     ASSERT_NEAR(mav2_acc[0], 0, 1e-10);  
//     ASSERT_NEAR(mav2_acc[1], 0, 1e-10);  
//     ASSERT_NEAR(mav2_acc[2], 1, 1e-10);   

//     // check bodyrate of mav2
//     Eigen::Vector3d mav2_bodyrate = Eigen::Vector3d::Random();
//     v_drone_cable.at(2).mav_.GetBodyrate(mav2_bodyrate);
//     ASSERT_FLOAT_EQ(mav2_bodyrate[0], 0); 
//     ASSERT_FLOAT_EQ(mav2_bodyrate[1], 0); 
//     ASSERT_FLOAT_EQ(mav2_bodyrate[2], 0);  

//     // check attitude of mav2
//     Eigen::Quaterniond mav2_att = Eigen::Quaterniond::UnitRandom();
//     v_drone_cable.at(2).mav_.GetAttitude(mav2_att);
//     ASSERT_FLOAT_EQ(mav2_att.x(), 0);     
//     ASSERT_FLOAT_EQ(mav2_att.y(), 0);      
//     ASSERT_FLOAT_EQ(mav2_att.z(), 0);     
//     ASSERT_FLOAT_EQ(mav2_att.w(), 1);      

//     // check bodyrate_acc of mav2
//     Eigen::Vector3d mav2_bodyrate_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(2).mav_.GetBodyRateAcc(mav2_bodyrate_acc);
//     ASSERT_FLOAT_EQ(mav2_bodyrate_acc[0], 0); 
//     ASSERT_FLOAT_EQ(mav2_bodyrate_acc[1], 0); 
//     ASSERT_FLOAT_EQ(mav2_bodyrate_acc[2], 0); 

//     //mav3
//     ASSERT_FLOAT_EQ(mav3_post[0], 1); 
//     ASSERT_FLOAT_EQ(mav3_post[1], -1); 
//     ASSERT_FLOAT_EQ(mav3_post[2], cable_length+0.5*1*pow(0.01,2));  

//     // vel of mav3
//     Eigen::Vector3d mav3_vel;
//     v_drone_cable.at(3).mav_.GetVel(mav3_vel);    
//     ASSERT_NEAR(mav3_vel[0], 0.0, 1e-10); 
//     ASSERT_NEAR(mav3_vel[1], 0.0, 1e-10);
//     ASSERT_NEAR(mav3_vel[2], 0.01, 1e-10);  


//     // acc of mav3
//     Eigen::Vector3d mav3_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(3).mav_.GetAcc(mav3_acc);
//     ASSERT_NEAR(mav3_acc[0], 0, 1e-10);  
//     ASSERT_NEAR(mav3_acc[1], 0, 1e-10);  
//     ASSERT_NEAR(mav3_acc[2], 1, 1e-10);   

//     // check bodyrate of mav3
//     Eigen::Vector3d mav3_bodyrate = Eigen::Vector3d::Random();
//     v_drone_cable.at(3).mav_.GetBodyrate(mav3_bodyrate);
//     ASSERT_FLOAT_EQ(mav3_bodyrate[0], 0); 
//     ASSERT_FLOAT_EQ(mav3_bodyrate[1], 0); 
//     ASSERT_FLOAT_EQ(mav3_bodyrate[2], 0);  

//     // check attitude of mav3
//     Eigen::Quaterniond mav3_att = Eigen::Quaterniond::UnitRandom();
//     v_drone_cable.at(3).mav_.GetAttitude(mav3_att);
//     ASSERT_FLOAT_EQ(mav3_att.x(), 0);     
//     ASSERT_FLOAT_EQ(mav3_att.y(), 0);      
//     ASSERT_FLOAT_EQ(mav3_att.z(), 0);     
//     ASSERT_FLOAT_EQ(mav3_att.w(), 1);      

//     // check bodyrate_acc of mav3
//     Eigen::Vector3d mav3_bodyrate_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(3).mav_.GetBodyRateAcc(mav3_bodyrate_acc);
//     ASSERT_FLOAT_EQ(mav3_bodyrate_acc[0], 0); 
//     ASSERT_FLOAT_EQ(mav3_bodyrate_acc[1], 0); 
//     ASSERT_FLOAT_EQ(mav3_bodyrate_acc[2], 0);     
// }






// // 1000 stesps with 1m/s^2

// // const acc for 1 step
// TEST_F(rotorTMCooperative, checkVerticalConstAcc1000Step){

//     // set initial posts for mavs and payload
//     ptr_Cooperative->SetPayloadInitPost();

//     // input mav controllers' inputs to produce 1m/s^2 along vertical direction
//     // 4 mavs + 1 payload =  4 + 1.5 = 5.5
//     // mav thrust = (5.5 * (1+9.8) )/4 = 14.85N
//     // mavs' net thrust force = 59.4

//     Eigen::VectorXd v_mavs_thrusts = Eigen::MatrixXd::Constant(4,1,14.85);
//     std::vector<Eigen::Vector3d> v_mavs_torques(4, Eigen::Vector3d::Zero());

//     // std::cout<< "fuck point cooperative test 1"<<std::endl;
//     ptr_Cooperative->InputControllerInput4MAVs(v_mavs_thrusts, v_mavs_torques);

//     // compute interation wrenches and vars for MAVs and payload
//     // std::cout<< "fuck point cooperative test 2"<<std::endl;
//     ptr_Cooperative->ComputeInteractWrenches();

//     // call one step dynamic simulation for MAVs and payload
//     // std::cout<< "fuck point cooperative test 3"<<std::endl;
//     // ptr_Cooperative->DoOneStepInt4Robots();
//     const double dt = 0.01;
//     for(double t=dt ; t-1e-5<=1000*dt ; t+= dt)
//     {
//             ptr_Cooperative->DoOneStepInt4Robots();
//             // printf("current step is %.3f \n", t);
//     }
//             // printf("current step is %.3f \n", t);
    

//     // 
//     Eigen::Vector3d mav0_post{0,0,0};
//     Eigen::Vector3d mav1_post{0,0,0};
//     Eigen::Vector3d mav2_post{0,0,0};
//     Eigen::Vector3d mav3_post{0,0,0};

//     // std::cout<< "fuck point cooperative test 4"<<std::endl;
//     std::vector<UAVCable> v_drone_cable = ptr_Cooperative->v_drone_cable_;

//     v_drone_cable.at(0).mav_.GetPosition(mav0_post);
//     v_drone_cable.at(1).mav_.GetPosition(mav1_post);
//     v_drone_cable.at(2).mav_.GetPosition(mav2_post);    
//     v_drone_cable.at(3).mav_.GetPosition(mav3_post);    

//     double cable_length;
//     ptr_Cooperative->v_drone_cable_.at(0).cable_.GetCableLength(cable_length);

//     // std::cout<<"[----------] test: mav1_init_post  is " << mav1_init_post.transpose()<<std::endl;
//     ASSERT_FLOAT_EQ(mav0_post[0], 1); 
//     ASSERT_FLOAT_EQ(mav0_post[1], 1); 
//     std::cout<< "cable length is " <<cable_length<<std::endl;
//     std::cout<< "cable_length+0.5*1*pow(0.01*1000,2) is " <<cable_length+0.5*1*pow(0.01*1000,2)<<std::endl;
//     std::cout<< "mav0_post[2] is " <<mav0_post[2]<<std::endl;
//     ASSERT_FLOAT_EQ(mav0_post[2], cable_length+0.5*1*pow(0.01*1000,2));  

//     // vel of mav0
//     Eigen::Vector3d mav0_vel;
//     v_drone_cable.at(0).mav_.GetVel(mav0_vel);    
//     ASSERT_NEAR(mav0_vel[0], 0.0, 1e-10); 
//     ASSERT_NEAR(mav0_vel[1], 0.0, 1e-10);
//     ASSERT_NEAR(mav0_vel[2], 0.01*1000, 1e-10);  

//     // acc of mav0
//     Eigen::Vector3d mav0_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(0).mav_.GetAcc(mav0_acc);
//     ASSERT_NEAR(mav0_acc[0], 0, 1e-10);  
//     ASSERT_NEAR(mav0_acc[1], 0, 1e-10);  
//     ASSERT_NEAR(mav0_acc[2], 0, 1);   

//     // check bodyrate of mav0
//     Eigen::Vector3d mav0_bodyrate = Eigen::Vector3d::Random();
//     v_drone_cable.at(0).mav_.GetBodyrate(mav0_bodyrate);
//     ASSERT_FLOAT_EQ(mav0_bodyrate[0], 0); 
//     ASSERT_FLOAT_EQ(mav0_bodyrate[1], 0); 
//     ASSERT_FLOAT_EQ(mav0_bodyrate[2], 0);  

//     // check attitude of mav0
//     Eigen::Quaterniond mav0_att = Eigen::Quaterniond::UnitRandom();
//     v_drone_cable.at(0).mav_.GetAttitude(mav0_att);
//     ASSERT_FLOAT_EQ(mav0_att.x(), 0);     
//     ASSERT_FLOAT_EQ(mav0_att.y(), 0);      
//     ASSERT_FLOAT_EQ(mav0_att.z(), 0);     
//     ASSERT_FLOAT_EQ(mav0_att.w(), 1);      

//     // check bodyrate_acc of mav0
//     Eigen::Vector3d mav0_bodyrate_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(0).mav_.GetBodyRateAcc(mav0_bodyrate_acc);
//     ASSERT_FLOAT_EQ(mav0_bodyrate_acc[0], 0); 
//     ASSERT_FLOAT_EQ(mav0_bodyrate_acc[1], 0); 
//     ASSERT_FLOAT_EQ(mav0_bodyrate_acc[2], 0);  



//     //mav1
//     ASSERT_FLOAT_EQ(mav1_post[0], -1); 
//     ASSERT_FLOAT_EQ(mav1_post[1], 1); 
//     ASSERT_FLOAT_EQ(mav1_post[2], cable_length+0.5*1*pow(0.01*1000,2));  

//     // vel of mav1
//     Eigen::Vector3d mav1_vel;
//     v_drone_cable.at(1).mav_.GetVel(mav1_vel);    
//     ASSERT_NEAR(mav1_vel[0], 0.0, 1e-10); 
//     ASSERT_NEAR(mav1_vel[1], 0.0, 1e-10);
//     ASSERT_NEAR(mav1_vel[2], 0.01*1000, 1e-10);  

//     // acc of mav1
//     Eigen::Vector3d mav1_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(1).mav_.GetAcc(mav1_acc);
//     ASSERT_NEAR(mav1_acc[0], 0, 1e-10);  
//     ASSERT_NEAR(mav1_acc[1], 0, 1e-10);  
//     ASSERT_NEAR(mav1_acc[2], 0, 1);   

//     // check bodyrate of mav1
//     Eigen::Vector3d mav1_bodyrate = Eigen::Vector3d::Random();
//     v_drone_cable.at(1).mav_.GetBodyrate(mav1_bodyrate);
//     ASSERT_FLOAT_EQ(mav1_bodyrate[0], 0); 
//     ASSERT_FLOAT_EQ(mav1_bodyrate[1], 0); 
//     ASSERT_FLOAT_EQ(mav1_bodyrate[2], 0);  

//     // check attitude of mav1
//     Eigen::Quaterniond mav1_att = Eigen::Quaterniond::UnitRandom();
//     v_drone_cable.at(1).mav_.GetAttitude(mav1_att);
//     ASSERT_FLOAT_EQ(mav1_att.x(), 0);     
//     ASSERT_FLOAT_EQ(mav1_att.y(), 0);      
//     ASSERT_FLOAT_EQ(mav1_att.z(), 0);     
//     ASSERT_FLOAT_EQ(mav1_att.w(), 1);      

//     // check bodyrate_acc of mav1
//     Eigen::Vector3d mav1_bodyrate_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(1).mav_.GetBodyRateAcc(mav1_bodyrate_acc);
//     ASSERT_FLOAT_EQ(mav1_bodyrate_acc[0], 0); 
//     ASSERT_FLOAT_EQ(mav1_bodyrate_acc[1], 0); 
//     ASSERT_FLOAT_EQ(mav1_bodyrate_acc[2], 0);      


//     //mav2
//     ASSERT_FLOAT_EQ(mav2_post[0], -1); 
//     ASSERT_FLOAT_EQ(mav2_post[1], -1); 
//     ASSERT_FLOAT_EQ(mav2_post[2], cable_length+0.5*1*pow(0.01*1000,2));  

//     // vel of mav2
//     Eigen::Vector3d mav2_vel;
//     v_drone_cable.at(2).mav_.GetVel(mav2_vel);    
//     ASSERT_NEAR(mav2_vel[0], 0.0, 1e-10); 
//     ASSERT_NEAR(mav2_vel[1], 0.0, 1e-10);
//     ASSERT_NEAR(mav2_vel[2], 0.01*1000, 1e-10);  


//     // acc of mav2
//     Eigen::Vector3d mav2_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(2).mav_.GetAcc(mav2_acc);
//     ASSERT_NEAR(mav2_acc[0], 0, 1e-10);  
//     ASSERT_NEAR(mav2_acc[1], 0, 1e-10);  
//     ASSERT_NEAR(mav2_acc[2], 1, 1e-10);   

//     // check bodyrate of mav2
//     Eigen::Vector3d mav2_bodyrate = Eigen::Vector3d::Random();
//     v_drone_cable.at(2).mav_.GetBodyrate(mav2_bodyrate);
//     ASSERT_FLOAT_EQ(mav2_bodyrate[0], 0); 
//     ASSERT_FLOAT_EQ(mav2_bodyrate[1], 0); 
//     ASSERT_FLOAT_EQ(mav2_bodyrate[2], 0);  

//     // check attitude of mav2
//     Eigen::Quaterniond mav2_att = Eigen::Quaterniond::UnitRandom();
//     v_drone_cable.at(2).mav_.GetAttitude(mav2_att);
//     ASSERT_FLOAT_EQ(mav2_att.x(), 0);     
//     ASSERT_FLOAT_EQ(mav2_att.y(), 0);      
//     ASSERT_FLOAT_EQ(mav2_att.z(), 0);     
//     ASSERT_FLOAT_EQ(mav2_att.w(), 1);      

//     // check bodyrate_acc of mav2
//     Eigen::Vector3d mav2_bodyrate_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(2).mav_.GetBodyRateAcc(mav2_bodyrate_acc);
//     ASSERT_FLOAT_EQ(mav2_bodyrate_acc[0], 0); 
//     ASSERT_FLOAT_EQ(mav2_bodyrate_acc[1], 0); 
//     ASSERT_FLOAT_EQ(mav2_bodyrate_acc[2], 0); 

//     //mav3
//     ASSERT_FLOAT_EQ(mav3_post[0], 1); 
//     ASSERT_FLOAT_EQ(mav3_post[1], -1); 
//     ASSERT_FLOAT_EQ(mav3_post[2], cable_length+0.5*1*pow(0.01*1000,2));  

//     // vel of mav3
//     Eigen::Vector3d mav3_vel;
//     v_drone_cable.at(3).mav_.GetVel(mav3_vel);    
//     ASSERT_NEAR(mav3_vel[0], 0.0, 1e-10); 
//     ASSERT_NEAR(mav3_vel[1], 0.0, 1e-10);
//     ASSERT_NEAR(mav3_vel[2], 0.01*1000, 1e-10);  


//     // acc of mav3
//     Eigen::Vector3d mav3_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(3).mav_.GetAcc(mav3_acc);
//     ASSERT_NEAR(mav3_acc[0], 0, 1e-10);  
//     ASSERT_NEAR(mav3_acc[1], 0, 1e-10);  
//     ASSERT_NEAR(mav3_acc[2], 1, 1e-10);   

//     // check bodyrate of mav3
//     Eigen::Vector3d mav3_bodyrate = Eigen::Vector3d::Random();
//     v_drone_cable.at(3).mav_.GetBodyrate(mav3_bodyrate);
//     ASSERT_FLOAT_EQ(mav3_bodyrate[0], 0); 
//     ASSERT_FLOAT_EQ(mav3_bodyrate[1], 0); 
//     ASSERT_FLOAT_EQ(mav3_bodyrate[2], 0);  

//     // check attitude of mav3
//     Eigen::Quaterniond mav3_att = Eigen::Quaterniond::UnitRandom();
//     v_drone_cable.at(3).mav_.GetAttitude(mav3_att);
//     ASSERT_FLOAT_EQ(mav3_att.x(), 0);     
//     ASSERT_FLOAT_EQ(mav3_att.y(), 0);      
//     ASSERT_FLOAT_EQ(mav3_att.z(), 0);     
//     ASSERT_FLOAT_EQ(mav3_att.w(), 1);      

//     // check bodyrate_acc of mav3
//     Eigen::Vector3d mav3_bodyrate_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(3).mav_.GetBodyRateAcc(mav3_bodyrate_acc);
//     ASSERT_FLOAT_EQ(mav3_bodyrate_acc[0], 0); 
//     ASSERT_FLOAT_EQ(mav3_bodyrate_acc[1], 0); 
//     ASSERT_FLOAT_EQ(mav3_bodyrate_acc[2], 0);     
// }










// // 10000 stesps with 1m/s^2

// // const acc for 10000 step
// TEST_F(rotorTMCooperative, checkVerticalConstAcc10000Step){

//     // set initial posts for mavs and payload
//     ptr_Cooperative->SetPayloadInitPost();

//     // input mav controllers' inputs to produce 1m/s^2 along vertical direction
//     // 4 mavs + 1 payload =  4 + 1.5 = 5.5
//     // mav thrust = (5.5 * (1+9.8) )/4 = 14.85N
//     // mavs' net thrust force = 59.4

//     Eigen::VectorXd v_mavs_thrusts = Eigen::MatrixXd::Constant(4,1,14.85);
//     std::vector<Eigen::Vector3d> v_mavs_torques(4, Eigen::Vector3d::Zero());

//     // std::cout<< "fuck point cooperative test 1"<<std::endl;
//     ptr_Cooperative->InputControllerInput4MAVs(v_mavs_thrusts, v_mavs_torques);

//     // compute interation wrenches and vars for MAVs and payload
//     // std::cout<< "fuck point cooperative test 2"<<std::endl;
//     ptr_Cooperative->ComputeInteractWrenches();

//     // call one step dynamic simulation for MAVs and payload
//     // std::cout<< "fuck point cooperative test 3"<<std::endl;
//     // ptr_Cooperative->DoOneStepInt4Robots();
//     const double dt = 0.01;
//     for(double t=dt ; t-1e-5<=10000*dt ; t+= dt)
//     {
//             ptr_Cooperative->DoOneStepInt4Robots();
//             // printf("current step is %.3f \n", t);
//     }
//             // printf("current step is %.3f \n", t);
    

//     // 
//     Eigen::Vector3d mav0_post{0,0,0};
//     Eigen::Vector3d mav1_post{0,0,0};
//     Eigen::Vector3d mav2_post{0,0,0};
//     Eigen::Vector3d mav3_post{0,0,0};

//     // std::cout<< "fuck point cooperative test 4"<<std::endl;
//     std::vector<UAVCable> v_drone_cable = ptr_Cooperative->v_drone_cable_;

//     v_drone_cable.at(0).mav_.GetPosition(mav0_post);
//     v_drone_cable.at(1).mav_.GetPosition(mav1_post);
//     v_drone_cable.at(2).mav_.GetPosition(mav2_post);    
//     v_drone_cable.at(3).mav_.GetPosition(mav3_post);    

//     double cable_length;
//     ptr_Cooperative->v_drone_cable_.at(0).cable_.GetCableLength(cable_length);

//     // std::cout<<"[----------] test: mav1_init_post  is " << mav1_init_post.transpose()<<std::endl;
//     ASSERT_FLOAT_EQ(mav0_post[0], 1); 
//     ASSERT_FLOAT_EQ(mav0_post[1], 1); 
//     std::cout<< "cable length is " <<cable_length<<std::endl;
//     std::cout<< "cable_length+0.5*1*pow(0.01*10000,2) is " <<cable_length+0.5*1*pow(0.01*10000,2)<<std::endl;
//     std::cout<< "mav0_post[2] is " <<mav0_post[2]<<std::endl;
//     ASSERT_FLOAT_EQ(mav0_post[2], cable_length+0.5*1*pow(0.01*10000,2));  

//     // vel of mav0
//     Eigen::Vector3d mav0_vel;
//     v_drone_cable.at(0).mav_.GetVel(mav0_vel);    
//     ASSERT_NEAR(mav0_vel[0], 0.0, 1e-10); 
//     ASSERT_NEAR(mav0_vel[1], 0.0, 1e-10);
//     ASSERT_NEAR(mav0_vel[2], 0.01*10000, 1e-10);  

//     // acc of mav0
//     Eigen::Vector3d mav0_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(0).mav_.GetAcc(mav0_acc);
//     ASSERT_NEAR(mav0_acc[0], 0, 1e-10);  
//     ASSERT_NEAR(mav0_acc[1], 0, 1e-10);  
//     ASSERT_NEAR(mav0_acc[2], 0, 1);   

//     // check bodyrate of mav0
//     Eigen::Vector3d mav0_bodyrate = Eigen::Vector3d::Random();
//     v_drone_cable.at(0).mav_.GetBodyrate(mav0_bodyrate);
//     ASSERT_FLOAT_EQ(mav0_bodyrate[0], 0); 
//     ASSERT_FLOAT_EQ(mav0_bodyrate[1], 0); 
//     ASSERT_FLOAT_EQ(mav0_bodyrate[2], 0);  

//     // check attitude of mav0
//     Eigen::Quaterniond mav0_att = Eigen::Quaterniond::UnitRandom();
//     v_drone_cable.at(0).mav_.GetAttitude(mav0_att);
//     ASSERT_FLOAT_EQ(mav0_att.x(), 0);     
//     ASSERT_FLOAT_EQ(mav0_att.y(), 0);      
//     ASSERT_FLOAT_EQ(mav0_att.z(), 0);     
//     ASSERT_FLOAT_EQ(mav0_att.w(), 1);      

//     // check bodyrate_acc of mav0
//     Eigen::Vector3d mav0_bodyrate_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(0).mav_.GetBodyRateAcc(mav0_bodyrate_acc);
//     ASSERT_FLOAT_EQ(mav0_bodyrate_acc[0], 0); 
//     ASSERT_FLOAT_EQ(mav0_bodyrate_acc[1], 0); 
//     ASSERT_FLOAT_EQ(mav0_bodyrate_acc[2], 0);  



//     //mav1
//     ASSERT_FLOAT_EQ(mav1_post[0], -1); 
//     ASSERT_FLOAT_EQ(mav1_post[1], 1); 
//     ASSERT_FLOAT_EQ(mav1_post[2], cable_length+0.5*1*pow(0.01*10000,2));  

//     // vel of mav1
//     Eigen::Vector3d mav1_vel;
//     v_drone_cable.at(1).mav_.GetVel(mav1_vel);    
//     ASSERT_NEAR(mav1_vel[0], 0.0, 1e-10); 
//     ASSERT_NEAR(mav1_vel[1], 0.0, 1e-10);
//     ASSERT_NEAR(mav1_vel[2], 0.01*10000, 1e-10);  

//     // acc of mav1
//     Eigen::Vector3d mav1_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(1).mav_.GetAcc(mav1_acc);
//     ASSERT_NEAR(mav1_acc[0], 0, 1e-10);  
//     ASSERT_NEAR(mav1_acc[1], 0, 1e-10);  
//     ASSERT_NEAR(mav1_acc[2], 0, 1);   

//     // check bodyrate of mav1
//     Eigen::Vector3d mav1_bodyrate = Eigen::Vector3d::Random();
//     v_drone_cable.at(1).mav_.GetBodyrate(mav1_bodyrate);
//     ASSERT_FLOAT_EQ(mav1_bodyrate[0], 0); 
//     ASSERT_FLOAT_EQ(mav1_bodyrate[1], 0); 
//     ASSERT_FLOAT_EQ(mav1_bodyrate[2], 0);  

//     // check attitude of mav1
//     Eigen::Quaterniond mav1_att = Eigen::Quaterniond::UnitRandom();
//     v_drone_cable.at(1).mav_.GetAttitude(mav1_att);
//     ASSERT_FLOAT_EQ(mav1_att.x(), 0);     
//     ASSERT_FLOAT_EQ(mav1_att.y(), 0);      
//     ASSERT_FLOAT_EQ(mav1_att.z(), 0);     
//     ASSERT_FLOAT_EQ(mav1_att.w(), 1);      

//     // check bodyrate_acc of mav1
//     Eigen::Vector3d mav1_bodyrate_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(1).mav_.GetBodyRateAcc(mav1_bodyrate_acc);
//     ASSERT_FLOAT_EQ(mav1_bodyrate_acc[0], 0); 
//     ASSERT_FLOAT_EQ(mav1_bodyrate_acc[1], 0); 
//     ASSERT_FLOAT_EQ(mav1_bodyrate_acc[2], 0);      


//     //mav2
//     ASSERT_FLOAT_EQ(mav2_post[0], -1); 
//     ASSERT_FLOAT_EQ(mav2_post[1], -1); 
//     ASSERT_FLOAT_EQ(mav2_post[2], cable_length+0.5*1*pow(0.01*10000,2));  

//     // vel of mav2
//     Eigen::Vector3d mav2_vel;
//     v_drone_cable.at(2).mav_.GetVel(mav2_vel);    
//     ASSERT_NEAR(mav2_vel[0], 0.0, 1e-10); 
//     ASSERT_NEAR(mav2_vel[1], 0.0, 1e-10);
//     ASSERT_NEAR(mav2_vel[2], 0.01*10000, 1e-10);  


//     // acc of mav2
//     Eigen::Vector3d mav2_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(2).mav_.GetAcc(mav2_acc);
//     ASSERT_NEAR(mav2_acc[0], 0, 1e-10);  
//     ASSERT_NEAR(mav2_acc[1], 0, 1e-10);  
//     ASSERT_NEAR(mav2_acc[2], 1, 1e-10);   

//     // check bodyrate of mav2
//     Eigen::Vector3d mav2_bodyrate = Eigen::Vector3d::Random();
//     v_drone_cable.at(2).mav_.GetBodyrate(mav2_bodyrate);
//     ASSERT_FLOAT_EQ(mav2_bodyrate[0], 0); 
//     ASSERT_FLOAT_EQ(mav2_bodyrate[1], 0); 
//     ASSERT_FLOAT_EQ(mav2_bodyrate[2], 0);  

//     // check attitude of mav2
//     Eigen::Quaterniond mav2_att = Eigen::Quaterniond::UnitRandom();
//     v_drone_cable.at(2).mav_.GetAttitude(mav2_att);
//     ASSERT_FLOAT_EQ(mav2_att.x(), 0);     
//     ASSERT_FLOAT_EQ(mav2_att.y(), 0);      
//     ASSERT_FLOAT_EQ(mav2_att.z(), 0);     
//     ASSERT_FLOAT_EQ(mav2_att.w(), 1);      

//     // check bodyrate_acc of mav2
//     Eigen::Vector3d mav2_bodyrate_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(2).mav_.GetBodyRateAcc(mav2_bodyrate_acc);
//     ASSERT_FLOAT_EQ(mav2_bodyrate_acc[0], 0); 
//     ASSERT_FLOAT_EQ(mav2_bodyrate_acc[1], 0); 
//     ASSERT_FLOAT_EQ(mav2_bodyrate_acc[2], 0); 

//     //mav3
//     ASSERT_FLOAT_EQ(mav3_post[0], 1); 
//     ASSERT_FLOAT_EQ(mav3_post[1], -1); 
//     ASSERT_FLOAT_EQ(mav3_post[2], cable_length+0.5*1*pow(0.01*10000,2));  

//     // vel of mav3
//     Eigen::Vector3d mav3_vel;
//     v_drone_cable.at(3).mav_.GetVel(mav3_vel);    
//     ASSERT_NEAR(mav3_vel[0], 0.0, 1e-10); 
//     ASSERT_NEAR(mav3_vel[1], 0.0, 1e-10);
//     ASSERT_NEAR(mav3_vel[2], 0.01*10000, 1e-10);  


//     // acc of mav3
//     Eigen::Vector3d mav3_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(3).mav_.GetAcc(mav3_acc);
//     ASSERT_NEAR(mav3_acc[0], 0, 1e-10);  
//     ASSERT_NEAR(mav3_acc[1], 0, 1e-10);  
//     ASSERT_NEAR(mav3_acc[2], 1, 1e-10);   

//     // check bodyrate of mav3
//     Eigen::Vector3d mav3_bodyrate = Eigen::Vector3d::Random();
//     v_drone_cable.at(3).mav_.GetBodyrate(mav3_bodyrate);
//     ASSERT_FLOAT_EQ(mav3_bodyrate[0], 0); 
//     ASSERT_FLOAT_EQ(mav3_bodyrate[1], 0); 
//     ASSERT_FLOAT_EQ(mav3_bodyrate[2], 0);  

//     // check attitude of mav3
//     Eigen::Quaterniond mav3_att = Eigen::Quaterniond::UnitRandom();
//     v_drone_cable.at(3).mav_.GetAttitude(mav3_att);
//     ASSERT_FLOAT_EQ(mav3_att.x(), 0);     
//     ASSERT_FLOAT_EQ(mav3_att.y(), 0);      
//     ASSERT_FLOAT_EQ(mav3_att.z(), 0);     
//     ASSERT_FLOAT_EQ(mav3_att.w(), 1);      

//     // check bodyrate_acc of mav3
//     Eigen::Vector3d mav3_bodyrate_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(3).mav_.GetBodyRateAcc(mav3_bodyrate_acc);
//     ASSERT_FLOAT_EQ(mav3_bodyrate_acc[0], 0); 
//     ASSERT_FLOAT_EQ(mav3_bodyrate_acc[1], 0); 
//     ASSERT_FLOAT_EQ(mav3_bodyrate_acc[2], 0);     
// }



// // variable acc for 100 steps
// TEST_F(rotorTMCooperative, checkVerticalVarAcc100Step){

//     // set initial posts for mavs and payload
//     ptr_Cooperative->SetPayloadInitPost();

//     // (1) input mav controllers' inputs to produce 1m/s^2 along vertical direction for 50 seconds
//     // 4 mavs + 1 payload =  4 + 1.5 = 5.5
//     // mav thrust = (5.5 * (1+9.8) )/4 = 14.85N
//     // mavs' net thrust force = 59.4
//     std::cout<<"[----------] first segment of trajectory"<<std::endl;
    
//     Eigen::VectorXd v_mavs_thrusts = Eigen::MatrixXd::Constant(4,1,14.85);
//     std::vector<Eigen::Vector3d> v_mavs_torques(4, Eigen::Vector3d::Zero());

//     // std::cout<< "fuck point cooperative test 1"<<std::endl;
//     ptr_Cooperative->InputControllerInput4MAVs(v_mavs_thrusts, v_mavs_torques);

//     // compute interation wrenches and vars for MAVs and payload
//     // std::cout<< "fuck point cooperative test 2"<<std::endl;
//     ptr_Cooperative->ComputeInteractWrenches();

//     // call one step dynamic simulation for MAVs and payload
//     // std::cout<< "fuck point cooperative test 3"<<std::endl;
//     // ptr_Cooperative->DoOneStepInt4Robots();

    
//     const double dt = 0.01;
//     for(double t=dt ; t-1e-5<=50*dt ; t+= dt)
//     {
//             ptr_Cooperative->DoOneStepInt4Robots();
//             // printf("current step is %.3f \n", t);
//     }
        
//     // (2) input mav controllers' inputs to produce 0.5m/s^2 along vertical direction for 50 seconds
//     // 4 mavs + 1 payload =  4 + 1.5 = 5.5
//     // mav thrust = (5.5 * (0.5+9.8) )/4 = 14.1625N
//     // mavs' net thrust force = 59.4
//     std::cout<<"[----------] second segment of trajectory"<<std::endl;
//     Eigen::VectorXd v_mavs_thrusts_second_segment = Eigen::MatrixXd::Constant(4,1,14.1625);
//     std::vector<Eigen::Vector3d> v_mavs_torques_second_segment(4, Eigen::Vector3d::Zero());

//     // std::cout<< "fuck point cooperative test 1"<<std::endl;
//     ptr_Cooperative->InputControllerInput4MAVs(v_mavs_thrusts_second_segment, v_mavs_torques_second_segment);

//     // compute interation wrenches and vars for MAVs and payload
//     // std::cout<< "fuck point cooperative test 2"<<std::endl;
//     ptr_Cooperative->ComputeInteractWrenches();

//     // call one step dynamic simulation for MAVs and payload
//     // std::cout<< "fuck point cooperative test 3"<<std::endl;
//     // ptr_Cooperative->DoOneStepInt4Robots();
//     for(double t=dt ; t-1e-5<=50*dt ; t+= dt)
//     {
//             ptr_Cooperative->DoOneStepInt4Robots();
//             // printf("current step is %.3f \n", t);
//     }    

//     // 
//     Eigen::Vector3d mav0_post{0,0,0};
//     Eigen::Vector3d mav1_post{0,0,0};
//     Eigen::Vector3d mav2_post{0,0,0};
//     Eigen::Vector3d mav3_post{0,0,0};

//     // std::cout<< "fuck point cooperative test 4"<<std::endl;
//     std::vector<UAVCable> v_drone_cable = ptr_Cooperative->v_drone_cable_;

//     v_drone_cable.at(0).mav_.GetPosition(mav0_post);
//     v_drone_cable.at(1).mav_.GetPosition(mav1_post);
//     v_drone_cable.at(2).mav_.GetPosition(mav2_post);    
//     v_drone_cable.at(3).mav_.GetPosition(mav3_post);    

//     double cable_length;
//     ptr_Cooperative->v_drone_cable_.at(0).cable_.GetCableLength(cable_length);

//     // std::cout<<"[----------] test: mav1_init_post  is " << mav1_init_post.transpose()<<std::endl;
//     ASSERT_FLOAT_EQ(mav0_post[0], 1); 
//     ASSERT_FLOAT_EQ(mav0_post[1], 1); 
//     std::cout<< "cable length is " <<cable_length<<std::endl;
//     std::cout<< "cable_length+0.5*1*pow(0.01*50,2) + +0.5*0.5*pow(0.01*50,2) is " <<cable_length+0.5*1*pow(0.01*50,2) + +0.5*0.5*pow(0.01*50,2)<<std::endl;
//     std::cout<< "mav0_post[2] is " <<mav0_post[2]<<std::endl;
//     ASSERT_FLOAT_EQ(mav0_post[2], cable_length+0.5*1*pow(0.01*50,2) + +0.5*0.5*pow(0.01*50,2));  

//     // vel of mav0
//     // at
//     Eigen::Vector3d mav0_vel;
//     v_drone_cable.at(0).mav_.GetVel(mav0_vel);    
//     ASSERT_NEAR(mav0_vel[0], 0.0, 1e-10); 
//     ASSERT_NEAR(mav0_vel[1], 0.0, 1e-10);
//     ASSERT_NEAR(mav0_vel[2], 1*0.01*50 + 0.5*0.01*50, 1e-10);  

//     // acc of mav0
//     Eigen::Vector3d mav0_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(0).mav_.GetAcc(mav0_acc);
//     ASSERT_NEAR(mav0_acc[0], 0, 1e-10);  
//     ASSERT_NEAR(mav0_acc[1], 0, 1e-10);  
//     ASSERT_NEAR(mav0_acc[2], 0.5, 1);   

//     // check bodyrate of mav0
//     Eigen::Vector3d mav0_bodyrate = Eigen::Vector3d::Random();
//     v_drone_cable.at(0).mav_.GetBodyrate(mav0_bodyrate);
//     ASSERT_FLOAT_EQ(mav0_bodyrate[0], 0); 
//     ASSERT_FLOAT_EQ(mav0_bodyrate[1], 0); 
//     ASSERT_FLOAT_EQ(mav0_bodyrate[2], 0);  

//     // check attitude of mav0
//     Eigen::Quaterniond mav0_att = Eigen::Quaterniond::UnitRandom();
//     v_drone_cable.at(0).mav_.GetAttitude(mav0_att);
//     ASSERT_FLOAT_EQ(mav0_att.x(), 0);     
//     ASSERT_FLOAT_EQ(mav0_att.y(), 0);      
//     ASSERT_FLOAT_EQ(mav0_att.z(), 0);     
//     ASSERT_FLOAT_EQ(mav0_att.w(), 1);      

//     // check bodyrate_acc of mav0
//     Eigen::Vector3d mav0_bodyrate_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(0).mav_.GetBodyRateAcc(mav0_bodyrate_acc);
//     ASSERT_FLOAT_EQ(mav0_bodyrate_acc[0], 0); 
//     ASSERT_FLOAT_EQ(mav0_bodyrate_acc[1], 0); 
//     ASSERT_FLOAT_EQ(mav0_bodyrate_acc[2], 0);  



//     //mav1
//     ASSERT_FLOAT_EQ(mav1_post[0], -1); 
//     ASSERT_FLOAT_EQ(mav1_post[1], 1); 
//     ASSERT_FLOAT_EQ(mav1_post[2], cable_length+0.5*1*pow(0.01*50,2) + +0.5*0.5*pow(0.01*50,2));  

//     // vel of mav1
//     Eigen::Vector3d mav1_vel;
//     v_drone_cable.at(1).mav_.GetVel(mav1_vel);    
//     ASSERT_NEAR(mav1_vel[0], 0.0, 1e-10); 
//     ASSERT_NEAR(mav1_vel[1], 0.0, 1e-10);
//     ASSERT_NEAR(mav1_vel[2], 1*0.01*50 + 0.5*0.01*50, 1e-10);  

//     // acc of mav1
//     Eigen::Vector3d mav1_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(1).mav_.GetAcc(mav1_acc);
//     ASSERT_NEAR(mav1_acc[0], 0, 1e-10);  
//     ASSERT_NEAR(mav1_acc[1], 0, 1e-10);  
//     ASSERT_NEAR(mav1_acc[2], 0.5, 1);   

//     // check bodyrate of mav1
//     Eigen::Vector3d mav1_bodyrate = Eigen::Vector3d::Random();
//     v_drone_cable.at(1).mav_.GetBodyrate(mav1_bodyrate);
//     ASSERT_FLOAT_EQ(mav1_bodyrate[0], 0); 
//     ASSERT_FLOAT_EQ(mav1_bodyrate[1], 0); 
//     ASSERT_FLOAT_EQ(mav1_bodyrate[2], 0);  

//     // check attitude of mav1
//     Eigen::Quaterniond mav1_att = Eigen::Quaterniond::UnitRandom();
//     v_drone_cable.at(1).mav_.GetAttitude(mav1_att);
//     ASSERT_FLOAT_EQ(mav1_att.x(), 0);     
//     ASSERT_FLOAT_EQ(mav1_att.y(), 0);      
//     ASSERT_FLOAT_EQ(mav1_att.z(), 0);     
//     ASSERT_FLOAT_EQ(mav1_att.w(), 1);      

//     // check bodyrate_acc of mav1
//     Eigen::Vector3d mav1_bodyrate_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(1).mav_.GetBodyRateAcc(mav1_bodyrate_acc);
//     ASSERT_FLOAT_EQ(mav1_bodyrate_acc[0], 0); 
//     ASSERT_FLOAT_EQ(mav1_bodyrate_acc[1], 0); 
//     ASSERT_FLOAT_EQ(mav1_bodyrate_acc[2], 0);      


//     //mav2
//     ASSERT_FLOAT_EQ(mav2_post[0], -1); 
//     ASSERT_FLOAT_EQ(mav2_post[1], -1); 
//     ASSERT_FLOAT_EQ(mav2_post[2], cable_length+0.5*1*pow(0.01*50,2) + +0.5*0.5*pow(0.01*50,2));  

//     // vel of mav2
//     Eigen::Vector3d mav2_vel;
//     v_drone_cable.at(2).mav_.GetVel(mav2_vel);    
//     ASSERT_NEAR(mav2_vel[0], 0.0, 1e-10); 
//     ASSERT_NEAR(mav2_vel[1], 0.0, 1e-10);
//     ASSERT_NEAR(mav2_vel[2],1*0.01*50 + 0.5*0.01*50, 1e-10);  


//     // acc of mav2
//     Eigen::Vector3d mav2_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(2).mav_.GetAcc(mav2_acc);
//     ASSERT_NEAR(mav2_acc[0], 0, 1e-10);  
//     ASSERT_NEAR(mav2_acc[1], 0, 1e-10);  
//     ASSERT_NEAR(mav2_acc[2], 0.5, 1e-10);   

//     // check bodyrate of mav2
//     Eigen::Vector3d mav2_bodyrate = Eigen::Vector3d::Random();
//     v_drone_cable.at(2).mav_.GetBodyrate(mav2_bodyrate);
//     ASSERT_FLOAT_EQ(mav2_bodyrate[0], 0); 
//     ASSERT_FLOAT_EQ(mav2_bodyrate[1], 0); 
//     ASSERT_FLOAT_EQ(mav2_bodyrate[2], 0);  

//     // check attitude of mav2
//     Eigen::Quaterniond mav2_att = Eigen::Quaterniond::UnitRandom();
//     v_drone_cable.at(2).mav_.GetAttitude(mav2_att);
//     ASSERT_FLOAT_EQ(mav2_att.x(), 0);     
//     ASSERT_FLOAT_EQ(mav2_att.y(), 0);      
//     ASSERT_FLOAT_EQ(mav2_att.z(), 0);     
//     ASSERT_FLOAT_EQ(mav2_att.w(), 1);      

//     // check bodyrate_acc of mav2
//     Eigen::Vector3d mav2_bodyrate_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(2).mav_.GetBodyRateAcc(mav2_bodyrate_acc);
//     ASSERT_FLOAT_EQ(mav2_bodyrate_acc[0], 0); 
//     ASSERT_FLOAT_EQ(mav2_bodyrate_acc[1], 0); 
//     ASSERT_FLOAT_EQ(mav2_bodyrate_acc[2], 0); 

//     //mav3
//     ASSERT_FLOAT_EQ(mav3_post[0], 1); 
//     ASSERT_FLOAT_EQ(mav3_post[1], -1); 
//     ASSERT_FLOAT_EQ(mav3_post[2], cable_length+0.5*1*pow(0.01*50,2) + +0.5*0.5*pow(0.01*50,2));  

//     // vel of mav3
//     Eigen::Vector3d mav3_vel;
//     v_drone_cable.at(3).mav_.GetVel(mav3_vel);    
//     ASSERT_NEAR(mav3_vel[0], 0.0, 1e-10); 
//     ASSERT_NEAR(mav3_vel[1], 0.0, 1e-10);
//     ASSERT_NEAR(mav3_vel[2], 1*0.01*50 + 0.5*0.01*50, 1e-10);  


//     // acc of mav3
//     Eigen::Vector3d mav3_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(3).mav_.GetAcc(mav3_acc);
//     ASSERT_NEAR(mav3_acc[0], 0, 1e-10);  
//     ASSERT_NEAR(mav3_acc[1], 0, 1e-10);  
//     ASSERT_NEAR(mav3_acc[2], 0.5, 1e-10);   

//     // check bodyrate of mav3
//     Eigen::Vector3d mav3_bodyrate = Eigen::Vector3d::Random();
//     v_drone_cable.at(3).mav_.GetBodyrate(mav3_bodyrate);
//     ASSERT_FLOAT_EQ(mav3_bodyrate[0], 0); 
//     ASSERT_FLOAT_EQ(mav3_bodyrate[1], 0); 
//     ASSERT_FLOAT_EQ(mav3_bodyrate[2], 0);  

//     // check attitude of mav3
//     Eigen::Quaterniond mav3_att = Eigen::Quaterniond::UnitRandom();
//     v_drone_cable.at(3).mav_.GetAttitude(mav3_att);
//     ASSERT_FLOAT_EQ(mav3_att.x(), 0);     
//     ASSERT_FLOAT_EQ(mav3_att.y(), 0);      
//     ASSERT_FLOAT_EQ(mav3_att.z(), 0);     
//     ASSERT_FLOAT_EQ(mav3_att.w(), 1);      

//     // check bodyrate_acc of mav3
//     Eigen::Vector3d mav3_bodyrate_acc = Eigen::Vector3d::Random();
//     v_drone_cable.at(3).mav_.GetBodyRateAcc(mav3_bodyrate_acc);
//     ASSERT_FLOAT_EQ(mav3_bodyrate_acc[0], 0); 
//     ASSERT_FLOAT_EQ(mav3_bodyrate_acc[1], 0); 
//     ASSERT_FLOAT_EQ(mav3_bodyrate_acc[2], 0);     
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