#include <iostream> 
#include <gtest/gtest.h>
#include <memory>
#include <cmath>
#include <cstdlib>
#include <random>
#include <vector>
#include "rotor_tm_sim/lib_cooperative.hpp"

double RandomGenerate(const double &minValue, const double &maxValue);

Eigen::Vector3d RandomUnitVector3d();


class rotorTMCooperative4MAV : public ::testing::Test
{
public:

rotorTMCooperative4MAV(){
    const int ROS_FREQ = 100;

    // set int step size to be same as ros step
    const double dt = 1.0/ROS_FREQ;

    const double mav_mass = 0.25;
    Eigen::Matrix3d mav_inertia = Eigen::Matrix3d::Zero(3,3);
    mav_inertia(0,0)= 0.000601;
    mav_inertia(1,1)= 0.000589;
    mav_inertia(2,2)= 0.001076; 

    MassProperty mav_mass_property = {mav_mass, mav_inertia};

    // payload
    // load_params/fedex_box_payload.yaml
    // 1. set payload param
    double payload_mass = 0.250;
    Eigen::Matrix3d payload_inertia = Eigen::Matrix3d::Zero(3,3);    
    payload_inertia(0,0)= 0.000601;
    payload_inertia(1,1)= 0.000589;
    payload_inertia(2,2)= 0.01076; 

    MassProperty payload_mass_property = {payload_mass, payload_inertia};
    // 2. set cable length
    const double cable_length = 0.5;

    // const Eigen::Vector3d v_attach_point_post{0,0,0};

    

    v_ptr_joints.reserve(4);
    v_ptr_uavcables.reserve(4);

  for (size_t i = 0; i < 4; i++)
    {
        auto ptr_joint = std::make_shared<Joint>(v_attach_point_post.at(i));

        v_ptr_joints.push_back(ptr_joint);
        // use v_ptr_uavcables[i] or use pus_back

        auto ptr_uav_cable= std::make_shared<UAVCable>(mav_mass_property, cable_length, v_ptr_joints.at(i), dt); 

        v_ptr_uavcables.push_back(ptr_uav_cable);
    }


    for (size_t i = 0; i < 4; i++)
    {
        // link join with uav cable
        v_ptr_joints.at(i)->LinkUAVCable(v_ptr_uavcables.at(i));
    }
    

    ptr_payload = std::make_shared<Payload>(payload_mass_property, v_ptr_joints, dt);

    ptr_Cooperative = std::make_shared<Cooperative>(ptr_payload, v_ptr_joints, v_ptr_uavcables);
}

~rotorTMCooperative4MAV(){
}

protected:
    std::shared_ptr<Cooperative> ptr_Cooperative;

    std::shared_ptr<Payload> ptr_payload;
    std::vector<std::shared_ptr<Joint>> v_ptr_joints;
    std::vector<std::shared_ptr<UAVCable>> v_ptr_uavcables;
    const std::vector<Eigen::Vector3d> v_attach_point_post{{1, 0, 0},{0,  1,   0}, {-1, 0, 0}, {0, -1, 0}};
};

// test if gTest is well integrated
TEST_F(rotorTMCooperative4MAV, checkGTest){
    ASSERT_TRUE(true);
}

// test if instance is created
TEST_F(rotorTMCooperative4MAV, checkInstanceClass){
    ASSERT_TRUE(ptr_Cooperative!=nullptr);
}


TEST_F(rotorTMCooperative4MAV, checkInitialPostsNoInput){
    // set position as default that payload is {0,0,0}
    ptr_Cooperative->SetPayloadInitPost();


    const double cable_length = ptr_Cooperative->v_ptr_uavcables_.at(0)->cable_.length();

    // std::cout << ptr_Cooperative->v_ptr_uavcables_.size()<<std::endl;

    // std::cout << "ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.pose().post" <<ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.pose().post<<std::endl;

    ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.pose().post[0], //
    ptr_Cooperative->ptr_payload_->jointPosttAt(0)[0]); 

    ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.pose().post[1], //
    ptr_Cooperative->ptr_payload_->jointPosttAt(0)[1]); 

    ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.pose().post[2], //
    ptr_Cooperative->ptr_payload_->jointPosttAt(0)[2]+cable_length); 


    ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.pose().post[0], //
    ptr_Cooperative->ptr_payload_->jointPosttAt(1)[0]); 

    ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.pose().post[1], //
    ptr_Cooperative->ptr_payload_->jointPosttAt(1)[1]); 

    ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.pose().post[2], //
    ptr_Cooperative->ptr_payload_->jointPosttAt(1)[2]+cable_length); 

    // compare 3rd mav with 3rd joint
    ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(2)->mav_.pose().post[0], //
    ptr_Cooperative->ptr_payload_->jointPosttAt(2)[0]);

    ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(2)->mav_.pose().post[1], //
    ptr_Cooperative->ptr_payload_->jointPosttAt(2)[1]); 

    ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.pose().post[2], //
    ptr_Cooperative->ptr_payload_->jointPosttAt(2)[2]+cable_length); 

    // with 4th joint
    ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.pose().post[0], //
    ptr_Cooperative->ptr_payload_->jointPosttAt(3)[0]);

    ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.pose().post[1], //
    ptr_Cooperative->ptr_payload_->jointPosttAt(3)[1]);

    ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.pose().post[2], //
    ptr_Cooperative->ptr_payload_->jointPosttAt(3)[2]+cable_length);
}




TEST_F(rotorTMCooperative4MAV, checkInitialPostsInput){

    Eigen::Vector3d payload_init_post = Eigen::Vector3d::Random();
    ptr_Cooperative->SetPayloadInitPost(payload_init_post);

   
    const double cable_length = ptr_Cooperative->v_ptr_uavcables_.at(0)->cable_.length();



    ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.pose().post[0], //
    ptr_Cooperative->ptr_payload_->jointPosttAt(0)[0]); 

    ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.pose().post[1], //
    ptr_Cooperative->ptr_payload_->jointPosttAt(0)[1]); 

    ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.pose().post[2], //
    ptr_Cooperative->ptr_payload_->jointPosttAt(0)[2]+cable_length); 


    ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.pose().post[0], //
    ptr_Cooperative->ptr_payload_->jointPosttAt(1)[0]); 

    ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.pose().post[1], //
    ptr_Cooperative->ptr_payload_->jointPosttAt(1)[1]); 

    ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.pose().post[2], //
    ptr_Cooperative->ptr_payload_->jointPosttAt(1)[2]+cable_length); 

    // compare 3rd mav with 3rd joint
    ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(2)->mav_.pose().post[0], //
    ptr_Cooperative->ptr_payload_->jointPosttAt(2)[0]);

    ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(2)->mav_.pose().post[1], //
    ptr_Cooperative->ptr_payload_->jointPosttAt(2)[1]); 

    ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.pose().post[2], //
    ptr_Cooperative->ptr_payload_->jointPosttAt(2)[2]+cable_length); 

    // with 4th joint
    ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.pose().post[0], //
    ptr_Cooperative->ptr_payload_->jointPosttAt(3)[0]);

    ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.pose().post[1], //
    ptr_Cooperative->ptr_payload_->jointPosttAt(3)[1]);

    ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.pose().post[2], //
    ptr_Cooperative->ptr_payload_->jointPosttAt(3)[2]+cable_length);

}









// test static equilibrium
TEST_F(rotorTMCooperative4MAV, checkVerticalStaticEquilibrium){

//     // set initial posts for mavs and payload
        Eigen::Vector3d payload_init_post = Eigen::Vector3d::Zero();
        ptr_Cooperative->SetPayloadInitPost(payload_init_post);

//     // input mav controllers' inputs to hover
//     // 4 mav + 1 payload =  0.25 * 5 = 1.25
//     // mav thrust = 1.25*9.8/4 = 3.0625

//     Eigen::VectorXd v_mavs_thrusts = Eigen::MatrixXd::Constant(4,1,13.475);
        std::vector<Eigen::Vector3d> v_mavs_torques(4, Eigen::Vector3d::Zero());
        std::vector<double> v_mavs_thrusts(4, 3.0625);

        // std::cout<< "fuck point cooperative test 3"<<std::endl;
        const double dt = 0.01;
        for(double t=dt ; t<=100*dt ; t+= dt)
        {
            std::cout<<"-----------------" << t << "-----------------" <<std::endl;

            ptr_Cooperative->InputControllerInput4MAVs(v_mavs_thrusts, v_mavs_torques);


            // compute interation wrenches and vars for MAVs and payload
            ptr_Cooperative->UpdateJointAndCableStatus();            

            ptr_Cooperative->UpdateJointAndCableStatus();       
            
            ptr_Cooperative->ComputeInteractWrenches();

            ptr_Cooperative->DoOneStepInt4Robots();
            // printf("current step is %.3f \n", t);
        }
       


        const double cable_length = ptr_Cooperative->v_ptr_uavcables_.at(0)->cable_.length();

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.pose().post[0], //
        ptr_Cooperative->ptr_payload_->jointPosttAt(0)[0]); 

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.pose().post[1], //
        ptr_Cooperative->ptr_payload_->jointPosttAt(0)[1]); 

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.pose().post[2], //
        ptr_Cooperative->ptr_payload_->jointPosttAt(0)[2]+cable_length); 


        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.pose().post[0], //
        ptr_Cooperative->ptr_payload_->jointPosttAt(1)[0]); 

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.pose().post[1], //
        ptr_Cooperative->ptr_payload_->jointPosttAt(1)[1]); 

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.pose().post[2], //
        ptr_Cooperative->ptr_payload_->jointPosttAt(1)[2]+cable_length); 

        // compare 3rd mav with 3rd joint
        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(2)->mav_.pose().post[0], //
        ptr_Cooperative->ptr_payload_->jointPosttAt(2)[0]);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(2)->mav_.pose().post[1], //
        ptr_Cooperative->ptr_payload_->jointPosttAt(2)[1]); 

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.pose().post[2], //
        ptr_Cooperative->ptr_payload_->jointPosttAt(2)[2]+cable_length); 

        // with 4th joint
        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.pose().post[0], //
        ptr_Cooperative->ptr_payload_->jointPosttAt(3)[0]);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.pose().post[1], //
        ptr_Cooperative->ptr_payload_->jointPosttAt(3)[1]);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.pose().post[2], //
        ptr_Cooperative->ptr_payload_->jointPosttAt(3)[2]+cable_length);       
}




TEST_F(rotorTMCooperative4MAV, checkVerticalStaticEquilibriumRandomPost){

//     // set initial posts for mavs and payload
        Eigen::Vector3d payload_init_post = Eigen::Vector3d::Random();
        ptr_Cooperative->SetPayloadInitPost(payload_init_post);

//     // input mav controllers' inputs to hover
//     // 4 mav + 1 payload =  0.25 * 5 = 1.25
//     // mav thrust = 1.25*9.8/4 = 3.0625

//     Eigen::VectorXd v_mavs_thrusts = Eigen::MatrixXd::Constant(4,1,13.475);
        std::vector<Eigen::Vector3d> v_mavs_torques(4, Eigen::Vector3d::Zero());
        std::vector<double> v_mavs_thrusts(4, 3.0625);

        // std::cout<< "fuck point cooperative test 3"<<std::endl;
        const double dt = 0.01;
        for(double t=dt ; t<=100*dt ; t+= dt)
        {
            std::cout<<"-----------------" << t << "-----------------" <<std::endl;

            ptr_Cooperative->InputControllerInput4MAVs(v_mavs_thrusts, v_mavs_torques);


            // compute interation wrenches and vars for MAVs and payload
            ptr_Cooperative->UpdateJointAndCableStatus();            

            ptr_Cooperative->UpdateJointAndCableStatus();       
            
            ptr_Cooperative->ComputeInteractWrenches();

            ptr_Cooperative->DoOneStepInt4Robots();
            // printf("current step is %.3f \n", t);
        }
       


        const double cable_length = ptr_Cooperative->v_ptr_uavcables_.at(0)->cable_.length();

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.pose().post[0], //
        ptr_Cooperative->ptr_payload_->jointPosttAt(0)[0]); 

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.pose().post[1], //
        ptr_Cooperative->ptr_payload_->jointPosttAt(0)[1]); 

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.pose().post[2], //
        ptr_Cooperative->ptr_payload_->jointPosttAt(0)[2]+cable_length); 


        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.pose().post[0], //
        ptr_Cooperative->ptr_payload_->jointPosttAt(1)[0]); 

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.pose().post[1], //
        ptr_Cooperative->ptr_payload_->jointPosttAt(1)[1]); 

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.pose().post[2], //
        ptr_Cooperative->ptr_payload_->jointPosttAt(1)[2]+cable_length); 

        // compare 3rd mav with 3rd joint
        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(2)->mav_.pose().post[0], //
        ptr_Cooperative->ptr_payload_->jointPosttAt(2)[0]);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(2)->mav_.pose().post[1], //
        ptr_Cooperative->ptr_payload_->jointPosttAt(2)[1]); 

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.pose().post[2], //
        ptr_Cooperative->ptr_payload_->jointPosttAt(2)[2]+cable_length); 

        // with 4th joint
        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.pose().post[0], //
        ptr_Cooperative->ptr_payload_->jointPosttAt(3)[0]);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.pose().post[1], //
        ptr_Cooperative->ptr_payload_->jointPosttAt(3)[1]);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.pose().post[2], //
        ptr_Cooperative->ptr_payload_->jointPosttAt(3)[2]+cable_length);       
}



// TEST_F(rotorTMCooperative4MAV, checkVerticalStaticEquilibrium){

// //     // set initial posts for mavs and payload
//         Eigen::Vector3d payload_init_post = Eigen::Vector3d::Zero();
//         ptr_Cooperative->SetPayloadInitPost(payload_init_post);

// //     // input mav controllers' inputs to hover
// //     // 4 mav + 1 payload =  0.25 * 5 = 1.25
// //     // mav thrust = 1.25*9.8/4 = 3.0625

// //     Eigen::VectorXd v_mavs_thrusts = Eigen::MatrixXd::Constant(4,1,13.475);
//         std::vector<Eigen::Vector3d> v_mavs_torques(4, Eigen::Vector3d::Zero());
//         std::vector<double> v_mavs_thrusts(4, 3.0625);

//         // std::cout<< "fuck point cooperative test 3"<<std::endl;
//         const double dt = 0.01;
//         for(double t=dt ; t<=100*dt ; t+= dt)
//         {
//             std::cout<<"-----------------" << t << "-----------------" <<std::endl;

//             ptr_Cooperative->InputControllerInput4MAVs(v_mavs_thrusts, v_mavs_torques);


//             // compute interation wrenches and vars for MAVs and payload
//             ptr_Cooperative->UpdateJointAndCableStatus();            

//             ptr_Cooperative->UpdateJointAndCableStatus();       
            
//             ptr_Cooperative->ComputeInteractWrenches();

//             ptr_Cooperative->DoOneStepInt4Robots();
//             // printf("current step is %.3f \n", t);
//         }
       


//         const double cable_length = ptr_Cooperative->v_ptr_uavcables_.at(0)->cable_.length();

//         ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.pose().post[0], //
//         ptr_Cooperative->ptr_payload_->jointPosttAt(0)[0]); 

//         ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.pose().post[1], //
//         ptr_Cooperative->ptr_payload_->jointPosttAt(0)[1]); 

//         ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.pose().post[2], //
//         ptr_Cooperative->ptr_payload_->jointPosttAt(0)[2]+cable_length); 


//         ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.pose().post[0], //
//         ptr_Cooperative->ptr_payload_->jointPosttAt(1)[0]); 

//         ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.pose().post[1], //
//         ptr_Cooperative->ptr_payload_->jointPosttAt(1)[1]); 

//         ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.pose().post[2], //
//         ptr_Cooperative->ptr_payload_->jointPosttAt(1)[2]+cable_length); 

//         // compare 3rd mav with 3rd joint
//         ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(2)->mav_.pose().post[0], //
//         ptr_Cooperative->ptr_payload_->jointPosttAt(2)[0]);

//         ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(2)->mav_.pose().post[1], //
//         ptr_Cooperative->ptr_payload_->jointPosttAt(2)[1]); 

//         ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.pose().post[2], //
//         ptr_Cooperative->ptr_payload_->jointPosttAt(2)[2]+cable_length); 

//         // with 4th joint
//         ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.pose().post[0], //
//         ptr_Cooperative->ptr_payload_->jointPosttAt(3)[0]);

//         ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.pose().post[1], //
//         ptr_Cooperative->ptr_payload_->jointPosttAt(3)[1]);

//         ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.pose().post[2], //
//         ptr_Cooperative->ptr_payload_->jointPosttAt(3)[2]+cable_length);       
// }



TEST_F(rotorTMCooperative4MAV, checkVerticalConstAcc){
//     // set initial posts for mavs and payload
        Eigen::Vector3d payload_init_post = Eigen::Vector3d::Zero();
        ptr_Cooperative->SetPayloadInitPost(payload_init_post);

//     // input mav controllers' inputs to hover
//     // 4 mav + 1 payload =  0.25 * 5 = 1.25
//     // linear acc = 1m/s^2
//     // mav thrust = 1.25*(9.8 + 1)/4 = 3.375

//     Eigen::VectorXd v_mavs_thrusts = Eigen::MatrixXd::Constant(4,1,13.475);
        std::vector<Eigen::Vector3d> v_mavs_torques(4, Eigen::Vector3d::Zero());
        std::vector<double> v_mavs_thrusts(4, 3.375);

        // std::cout<< "fuck point cooperative test 3"<<std::endl;
        const double dt = 0.01;
        const double num_steps = 10;
        for(double t=dt ; t<=num_steps*dt ; t+= dt)
        {
            std::cout<<"-----------------" << t << "-----------------" <<std::endl;

            ptr_Cooperative->InputControllerInput4MAVs(v_mavs_thrusts, v_mavs_torques);


            // compute interation wrenches and vars for MAVs and payload
            ptr_Cooperative->UpdateJointAndCableStatus();            

            ptr_Cooperative->UpdateVelsCollidedUAVsPayload();       
            
            ptr_Cooperative->ComputeInteractWrenches();

            ptr_Cooperative->DoOneStepInt4Robots();
            // printf("current step is %.3f \n", t);
        }
       


        const double cable_length = ptr_Cooperative->v_ptr_uavcables_.at(0)->cable_.length();

        // payload post
        ASSERT_FLOAT_EQ(ptr_Cooperative->ptr_payload_->pose().post[0], //
        payload_init_post[0]);         

        ASSERT_FLOAT_EQ(ptr_Cooperative->ptr_payload_->pose().post[1], //
        payload_init_post[1]); 

        ASSERT_FLOAT_EQ(ptr_Cooperative->ptr_payload_->pose().post[2], //
        payload_init_post[2] + 0.5 * pow(num_steps*dt,2)); 

        // payload att
        ASSERT_FLOAT_EQ(ptr_Cooperative->ptr_payload_->pose().att.x(), //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->ptr_payload_->pose().att.y(), //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->ptr_payload_->pose().att.z(), //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->ptr_payload_->pose().att.w(), //
        1);        

        // payload vel
        ASSERT_FLOAT_EQ(ptr_Cooperative->ptr_payload_->vels().linear_vel[0], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->ptr_payload_->vels().linear_vel[1], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->ptr_payload_->vels().linear_vel[2], //
        num_steps*dt);

        // payload bodyrate
        ASSERT_FLOAT_EQ(ptr_Cooperative->ptr_payload_->vels().bodyrate[0], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->ptr_payload_->vels().bodyrate[1], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->ptr_payload_->vels().bodyrate[2], //
        0);

        // payload linear acc
        ASSERT_FLOAT_EQ(ptr_Cooperative->ptr_payload_->accs().linear_acc[0], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->ptr_payload_->accs().linear_acc[1], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->ptr_payload_->accs().linear_acc[2], //
        1);

        // payload angular acc
        ASSERT_FLOAT_EQ(ptr_Cooperative->ptr_payload_->accs().angular_acc[0], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->ptr_payload_->accs().angular_acc[1], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->ptr_payload_->accs().angular_acc[2], //
        0);

        // mav 0
        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.pose().post[0], //
        payload_init_post[0] + v_attach_point_post.at(0)[0]); 

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.pose().post[1], //
        payload_init_post[1] + v_attach_point_post.at(0)[1]); 

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.pose().post[2], //
        payload_init_post[2] + v_attach_point_post.at(0)[2] + cable_length + 0.5 * pow(num_steps*dt,2)); 

        // mav 0 att
        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.pose().att.x(), //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.pose().att.y(), //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.pose().att.z(), //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.pose().att.w(), //
        1);

        // mav 0 vel
        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.vels().linear_vel[0], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.vels().linear_vel[1], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.vels().linear_vel[2], //
        num_steps*dt);

        // mav 0 bodyrate
        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.vels().bodyrate[0], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.vels().bodyrate[1], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.vels().bodyrate[2], //
        0);

        // mav 0 acc
        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.accs().linear_acc[0], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.accs().linear_acc[1], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.accs().linear_acc[2], //
        1);

        // mav 0 angular acc
        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.accs().angular_acc[0], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.accs().angular_acc[1], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.accs().angular_acc[2], //
        0);




        // mav 1
        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.pose().post[0], //
        payload_init_post[0] + v_attach_point_post.at(1)[0]);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.pose().post[1], //
        payload_init_post[1] + v_attach_point_post.at(1)[1]);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.pose().post[2], //
        payload_init_post[2] + v_attach_point_post.at(1)[2] + cable_length + 0.5 * pow(num_steps*dt,2));


        // mav 1 att
        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.pose().att.x(), //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.pose().att.y(), //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.pose().att.z(), //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.pose().att.w(), //
        1);


        // mav 1 vel
        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.vels().linear_vel[0], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.vels().linear_vel[1], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.vels().linear_vel[2], //
        num_steps*dt);

        // mav 1 bodyrate
        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.vels().bodyrate[0], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.vels().bodyrate[1], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.vels().bodyrate[2], //
        0);

        // mav 1 acc
        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.accs().linear_acc[0], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.accs().linear_acc[1], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.accs().linear_acc[2], //
        1);

        // mav 1 angular acc
        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.accs().angular_acc[0], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.accs().angular_acc[1], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.accs().angular_acc[2], //
        0);

        // mav 2
        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(2)->mav_.pose().post[0], //
        payload_init_post[0] + v_attach_point_post.at(2)[0]);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(2)->mav_.pose().post[1], //
        payload_init_post[1] + v_attach_point_post.at(2)[1]);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(2)->mav_.pose().post[2], //
        payload_init_post[2] + v_attach_point_post.at(2)[2] + cable_length + 0.5 * pow(num_steps*dt,2));

        // mav 2 att
        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(2)->mav_.pose().att.x(), //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(2)->mav_.pose().att.y(), //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(2)->mav_.pose().att.z(), //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(2)->mav_.pose().att.w(), //
        1);

        // mav 2 vel
        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(2)->mav_.vels().linear_vel[0], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(2)->mav_.vels().linear_vel[1], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(2)->mav_.vels().linear_vel[2], //
        num_steps*dt);

        // mav 2 bodyrate
        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(2)->mav_.vels().bodyrate[0], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(2)->mav_.vels().bodyrate[1], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(2)->mav_.vels().bodyrate[2], //
        0);

        // mav 2 acc
        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(2)->mav_.accs().linear_acc[0], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(2)->mav_.accs().linear_acc[1], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(2)->mav_.accs().linear_acc[2], //
        1);

        // mav 2 angular acc
        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(2)->mav_.accs().angular_acc[0], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(2)->mav_.accs().angular_acc[1], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(2)->mav_.accs().angular_acc[2], //
        0);

        // mav 3
        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.pose().post[0], //
        payload_init_post[0] + v_attach_point_post.at(3)[0]);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.pose().post[1], //
        payload_init_post[1] + v_attach_point_post.at(3)[1]);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.pose().post[2], //
        payload_init_post[2] + v_attach_point_post.at(3)[2] + cable_length + 0.5 * pow(num_steps*dt,2));

        // mav 3 att
        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.pose().att.x(), //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.pose().att.y(), //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.pose().att.z(), //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.pose().att.w(), //
        1);

        // mav 3 vel
        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.vels().linear_vel[0], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.vels().linear_vel[1], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.vels().linear_vel[2], //
        num_steps*dt);

        // mav 3 bodyrate
        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.vels().bodyrate[0], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.vels().bodyrate[1], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.vels().bodyrate[2], //
        0);

        // mav 3 acc
        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.accs().linear_acc[0], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.accs().linear_acc[1], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.accs().linear_acc[2], //
        1);

        // mav 3 angular acc
        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.accs().angular_acc[0], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.accs().angular_acc[1], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.accs().angular_acc[2], //
        0);


}



TEST_F(rotorTMCooperative4MAV, checkVerticalConstAccRandomPost){
//     // set initial posts for mavs and payload
        Eigen::Vector3d payload_init_post = Eigen::Vector3d::Random();
        ptr_Cooperative->SetPayloadInitPost(payload_init_post);

//     // input mav controllers' inputs to hover
//     // 4 mav + 1 payload =  0.25 * 5 = 1.25
//     // linear acc = 1m/s^2
//     // mav thrust = 1.25*(9.8 + 1)/4 = 3.375

//     Eigen::VectorXd v_mavs_thrusts = Eigen::MatrixXd::Constant(4,1,13.475);
        std::vector<Eigen::Vector3d> v_mavs_torques(4, Eigen::Vector3d::Zero());
        std::vector<double> v_mavs_thrusts(4, 3.375);

        // std::cout<< "fuck point cooperative test 3"<<std::endl;
        const double dt = 0.01;
        const double num_steps = 10;
        for(double t=dt ; t<=num_steps*dt ; t+= dt)
        {
            std::cout<<"-----------------" << t << "-----------------" <<std::endl;

            ptr_Cooperative->InputControllerInput4MAVs(v_mavs_thrusts, v_mavs_torques);


            // compute interation wrenches and vars for MAVs and payload
            ptr_Cooperative->UpdateJointAndCableStatus();            

            ptr_Cooperative->UpdateVelsCollidedUAVsPayload();       
            
            ptr_Cooperative->ComputeInteractWrenches();

            ptr_Cooperative->DoOneStepInt4Robots();
            // printf("current step is %.3f \n", t);
        }
       


        const double cable_length = ptr_Cooperative->v_ptr_uavcables_.at(0)->cable_.length();

        // payload post
        ASSERT_FLOAT_EQ(ptr_Cooperative->ptr_payload_->pose().post[0], //
        payload_init_post[0]);         

        ASSERT_FLOAT_EQ(ptr_Cooperative->ptr_payload_->pose().post[1], //
        payload_init_post[1]); 

        ASSERT_FLOAT_EQ(ptr_Cooperative->ptr_payload_->pose().post[2], //
        payload_init_post[2] + 0.5 * pow(num_steps*dt,2)); 

        // payload att
        ASSERT_FLOAT_EQ(ptr_Cooperative->ptr_payload_->pose().att.x(), //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->ptr_payload_->pose().att.y(), //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->ptr_payload_->pose().att.z(), //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->ptr_payload_->pose().att.w(), //
        1);        

        // payload vel
        ASSERT_FLOAT_EQ(ptr_Cooperative->ptr_payload_->vels().linear_vel[0], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->ptr_payload_->vels().linear_vel[1], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->ptr_payload_->vels().linear_vel[2], //
        num_steps*dt);

        // payload bodyrate
        ASSERT_FLOAT_EQ(ptr_Cooperative->ptr_payload_->vels().bodyrate[0], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->ptr_payload_->vels().bodyrate[1], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->ptr_payload_->vels().bodyrate[2], //
        0);

        // payload linear acc
        ASSERT_FLOAT_EQ(ptr_Cooperative->ptr_payload_->accs().linear_acc[0], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->ptr_payload_->accs().linear_acc[1], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->ptr_payload_->accs().linear_acc[2], //
        1);

        // payload angular acc
        ASSERT_FLOAT_EQ(ptr_Cooperative->ptr_payload_->accs().angular_acc[0], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->ptr_payload_->accs().angular_acc[1], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->ptr_payload_->accs().angular_acc[2], //
        0);

        // mav 0
        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.pose().post[0], //
        payload_init_post[0] + v_attach_point_post.at(0)[0]); 

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.pose().post[1], //
        payload_init_post[1] + v_attach_point_post.at(0)[1]); 

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.pose().post[2], //
        payload_init_post[2] + v_attach_point_post.at(0)[2] + cable_length + 0.5 * pow(num_steps*dt,2)); 

        // mav 0 att
        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.pose().att.x(), //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.pose().att.y(), //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.pose().att.z(), //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.pose().att.w(), //
        1);

        // mav 0 vel
        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.vels().linear_vel[0], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.vels().linear_vel[1], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.vels().linear_vel[2], //
        num_steps*dt);

        // mav 0 bodyrate
        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.vels().bodyrate[0], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.vels().bodyrate[1], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.vels().bodyrate[2], //
        0);

        // mav 0 acc
        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.accs().linear_acc[0], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.accs().linear_acc[1], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.accs().linear_acc[2], //
        1);

        // mav 0 angular acc
        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.accs().angular_acc[0], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.accs().angular_acc[1], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.accs().angular_acc[2], //
        0);




        // mav 1
        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.pose().post[0], //
        payload_init_post[0] + v_attach_point_post.at(1)[0]);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.pose().post[1], //
        payload_init_post[1] + v_attach_point_post.at(1)[1]);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.pose().post[2], //
        payload_init_post[2] + v_attach_point_post.at(1)[2] + cable_length + 0.5 * pow(num_steps*dt,2));


        // mav 1 att
        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.pose().att.x(), //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.pose().att.y(), //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.pose().att.z(), //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.pose().att.w(), //
        1);


        // mav 1 vel
        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.vels().linear_vel[0], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.vels().linear_vel[1], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.vels().linear_vel[2], //
        num_steps*dt);

        // mav 1 bodyrate
        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.vels().bodyrate[0], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.vels().bodyrate[1], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.vels().bodyrate[2], //
        0);

        // mav 1 acc
        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.accs().linear_acc[0], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.accs().linear_acc[1], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.accs().linear_acc[2], //
        1);

        // mav 1 angular acc
        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.accs().angular_acc[0], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.accs().angular_acc[1], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.accs().angular_acc[2], //
        0);

        // mav 2
        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(2)->mav_.pose().post[0], //
        payload_init_post[0] + v_attach_point_post.at(2)[0]);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(2)->mav_.pose().post[1], //
        payload_init_post[1] + v_attach_point_post.at(2)[1]);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(2)->mav_.pose().post[2], //
        payload_init_post[2] + v_attach_point_post.at(2)[2] + cable_length + 0.5 * pow(num_steps*dt,2));

        // mav 2 att
        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(2)->mav_.pose().att.x(), //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(2)->mav_.pose().att.y(), //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(2)->mav_.pose().att.z(), //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(2)->mav_.pose().att.w(), //
        1);

        // mav 2 vel
        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(2)->mav_.vels().linear_vel[0], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(2)->mav_.vels().linear_vel[1], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(2)->mav_.vels().linear_vel[2], //
        num_steps*dt);

        // mav 2 bodyrate
        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(2)->mav_.vels().bodyrate[0], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(2)->mav_.vels().bodyrate[1], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(2)->mav_.vels().bodyrate[2], //
        0);

        // mav 2 acc
        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(2)->mav_.accs().linear_acc[0], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(2)->mav_.accs().linear_acc[1], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(2)->mav_.accs().linear_acc[2], //
        1);

        // mav 2 angular acc
        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(2)->mav_.accs().angular_acc[0], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(2)->mav_.accs().angular_acc[1], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(2)->mav_.accs().angular_acc[2], //
        0);

        // mav 3
        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.pose().post[0], //
        payload_init_post[0] + v_attach_point_post.at(3)[0]);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.pose().post[1], //
        payload_init_post[1] + v_attach_point_post.at(3)[1]);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.pose().post[2], //
        payload_init_post[2] + v_attach_point_post.at(3)[2] + cable_length + 0.5 * pow(num_steps*dt,2));

        // mav 3 att
        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.pose().att.x(), //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.pose().att.y(), //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.pose().att.z(), //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.pose().att.w(), //
        1);

        // mav 3 vel
        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.vels().linear_vel[0], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.vels().linear_vel[1], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.vels().linear_vel[2], //
        num_steps*dt);

        // mav 3 bodyrate
        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.vels().bodyrate[0], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.vels().bodyrate[1], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.vels().bodyrate[2], //
        0);

        // mav 3 acc
        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.accs().linear_acc[0], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.accs().linear_acc[1], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.accs().linear_acc[2], //
        1);

        // mav 3 angular acc
        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.accs().angular_acc[0], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.accs().angular_acc[1], //
        0);

        ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.accs().angular_acc[2], //
        0);


}



// test force and torque balance
TEST_F(rotorTMCooperative4MAV, checkOneMAVWrench2Payload){
    
        // set initial posts for mavs and payload
        Eigen::Vector3d payload_init_post = Eigen::Vector3d::Zero();
        ptr_Cooperative->SetPayloadInitPost(payload_init_post);


        // assigen zero torques and balance thrust to mavs
        std::vector<Eigen::Vector3d> v_mavs_torques(4, Eigen::Vector3d::Zero());
        std::vector<double> v_mavs_thrusts(4, 3.0625);


        // static equilibrium
        ptr_Cooperative->InputControllerInput4MAVs(v_mavs_thrusts, v_mavs_torques);

        // compute interation wrenches and vars for MAVs and payload
        ptr_Cooperative->UpdateJointAndCableStatus();            

        ptr_Cooperative->UpdateVelsCollidedUAVsPayload();       
            
        ptr_Cooperative->ComputeInteractWrenches();

        ptr_Cooperative->DoOneStepInt4Robots();

        // change onely mav0
        v_mavs_thrusts.at(0) = 3.0625 + 0.5;

        // static equilibrium
        ptr_Cooperative->InputControllerInput4MAVs(v_mavs_thrusts, v_mavs_torques);

        // compute interation wrenches and vars for MAVs and payload
        ptr_Cooperative->UpdateJointAndCableStatus();            

        ptr_Cooperative->UpdateVelsCollidedUAVsPayload();       
            
        ptr_Cooperative->ComputeInteractWrenches();        

        // total force and torque
        Eigen::Vector3d total_force = Eigen::Vector3d::UnitZ() * (3.0625 * 4 +  0.5);
        auto mav0_torque_2_payload  = (v_attach_point_post.at(0)).cross(Eigen::Vector3d::UnitZ() * v_mavs_thrusts.at(0));
        auto mav1_torque_2_payload  = (v_attach_point_post.at(1)).cross(Eigen::Vector3d::UnitZ() * v_mavs_thrusts.at(1));
        auto mav2_torque_2_payload  = (v_attach_point_post.at(2)).cross(Eigen::Vector3d::UnitZ() * v_mavs_thrusts.at(2));
        auto mav3_torque_2_payload  = (v_attach_point_post.at(3)).cross(Eigen::Vector3d::UnitZ() * v_mavs_thrusts.at(3));

        Eigen::Vector3d total_torque =  mav0_torque_2_payload + mav1_torque_2_payload + mav2_torque_2_payload + mav3_torque_2_payload;

        std::cout<< "total torque is " << total_torque.transpose()<<std::endl; 
       
       // check total force
       auto payload_net_force = ptr_Cooperative->NetMavsWrenchToPayload().force;
       auto payload_net_torque = ptr_Cooperative->NetMavsWrenchToPayload().torque;

       // check force
       EXPECT_FLOAT_EQ(total_force[0], payload_net_force[0]);
       EXPECT_FLOAT_EQ(total_force[1], payload_net_force[1]);
       EXPECT_FLOAT_EQ(total_force[2], payload_net_force[2]);

       // check torque
        EXPECT_FLOAT_EQ(total_torque[0], payload_net_torque[0]);
        EXPECT_FLOAT_EQ(total_torque[1], payload_net_torque[1]);
        EXPECT_FLOAT_EQ(total_torque[2], payload_net_torque[2]);
 }



TEST_F(rotorTMCooperative4MAV, checkOneMAVWrench2PayloadwithRandom){
    
        // set initial posts for mavs and payload
        Eigen::Vector3d payload_init_post = Eigen::Vector3d::Zero();
        ptr_Cooperative->SetPayloadInitPost(payload_init_post);


        // assigen zero torques and balance thrust to mavs
        std::vector<Eigen::Vector3d> v_mavs_torques(4, Eigen::Vector3d::Zero());
        std::vector<double> v_mavs_thrusts(4, 3.0625);


        // static equilibrium
        ptr_Cooperative->InputControllerInput4MAVs(v_mavs_thrusts, v_mavs_torques);

        // compute interation wrenches and vars for MAVs and payload
        ptr_Cooperative->UpdateJointAndCableStatus();            

        ptr_Cooperative->UpdateVelsCollidedUAVsPayload();       
            
        ptr_Cooperative->ComputeInteractWrenches();

        ptr_Cooperative->DoOneStepInt4Robots();

        // change onely mav0
        double mav0_change_of_thrust =  RandomGenerate(-2,2); 
        v_mavs_thrusts.at(0) = 3.0625 + mav0_change_of_thrust;

        // static equilibrium
        ptr_Cooperative->InputControllerInput4MAVs(v_mavs_thrusts, v_mavs_torques);

        // compute interation wrenches and vars for MAVs and payload
        ptr_Cooperative->UpdateJointAndCableStatus();            

        ptr_Cooperative->UpdateVelsCollidedUAVsPayload();       
            
        ptr_Cooperative->ComputeInteractWrenches();        

        // total force and torque
        Eigen::Vector3d total_force = Eigen::Vector3d::UnitZ() * (3.0625 * 4 +  mav0_change_of_thrust);
        auto mav0_torque_2_payload  = (v_attach_point_post.at(0)).cross(Eigen::Vector3d::UnitZ() * v_mavs_thrusts.at(0));
        auto mav1_torque_2_payload  = (v_attach_point_post.at(1)).cross(Eigen::Vector3d::UnitZ() * v_mavs_thrusts.at(1));
        auto mav2_torque_2_payload  = (v_attach_point_post.at(2)).cross(Eigen::Vector3d::UnitZ() * v_mavs_thrusts.at(2));
        auto mav3_torque_2_payload  = (v_attach_point_post.at(3)).cross(Eigen::Vector3d::UnitZ() * v_mavs_thrusts.at(3));

        Eigen::Vector3d total_torque =  mav0_torque_2_payload + mav1_torque_2_payload + mav2_torque_2_payload + mav3_torque_2_payload;

        std::cout<< "total torque is " << total_torque.transpose()<<std::endl; 
       
       // check total force
       auto payload_net_force = ptr_Cooperative->NetMavsWrenchToPayload().force;
       auto payload_net_torque = ptr_Cooperative->NetMavsWrenchToPayload().torque;

       // check force
       EXPECT_FLOAT_EQ(total_force[0], payload_net_force[0]);
       EXPECT_FLOAT_EQ(total_force[1], payload_net_force[1]);
       EXPECT_FLOAT_EQ(total_force[2], payload_net_force[2]);

       // check torque
        EXPECT_FLOAT_EQ(total_torque[0], payload_net_torque[0]);
        EXPECT_FLOAT_EQ(total_torque[1], payload_net_torque[1]);
        EXPECT_FLOAT_EQ(total_torque[2], payload_net_torque[2]);
 }



TEST_F(rotorTMCooperative4MAV, checkTwoMAVWrench2PayloadwithRandom){
    
        // set initial posts for mavs and payload
        Eigen::Vector3d payload_init_post = Eigen::Vector3d::Zero();
        ptr_Cooperative->SetPayloadInitPost(payload_init_post);


        // assigen zero torques and balance thrust to mavs
        std::vector<Eigen::Vector3d> v_mavs_torques(4, Eigen::Vector3d::Zero());
        std::vector<double> v_mavs_thrusts(4, 3.0625);


        // static equilibrium
        ptr_Cooperative->InputControllerInput4MAVs(v_mavs_thrusts, v_mavs_torques);

        // compute interation wrenches and vars for MAVs and payload
        ptr_Cooperative->UpdateJointAndCableStatus();            

        ptr_Cooperative->UpdateVelsCollidedUAVsPayload();       
            
        ptr_Cooperative->ComputeInteractWrenches();

        ptr_Cooperative->DoOneStepInt4Robots();

        // change thrusts of mav0 and mav1
        double mav0_change_of_thrust =  RandomGenerate(-3,3); 
        v_mavs_thrusts.at(0) = 3.0625 + mav0_change_of_thrust;

        double mav1_change_of_thrust =  RandomGenerate(-3,3); 
        v_mavs_thrusts.at(1) = 3.0625 + mav1_change_of_thrust;


        // static equilibrium
        ptr_Cooperative->InputControllerInput4MAVs(v_mavs_thrusts, v_mavs_torques);

        // compute interation wrenches and vars for MAVs and payload
        ptr_Cooperative->UpdateJointAndCableStatus();            

        ptr_Cooperative->UpdateVelsCollidedUAVsPayload();       
            
        ptr_Cooperative->ComputeInteractWrenches();        

        // total force and torque
        Eigen::Vector3d total_force = Eigen::Vector3d::UnitZ() * (3.0625 * 4 +  mav0_change_of_thrust + mav1_change_of_thrust);
        auto mav0_torque_2_payload  = (v_attach_point_post.at(0)).cross(Eigen::Vector3d::UnitZ() * v_mavs_thrusts.at(0));
        auto mav1_torque_2_payload  = (v_attach_point_post.at(1)).cross(Eigen::Vector3d::UnitZ() * v_mavs_thrusts.at(1));
        auto mav2_torque_2_payload  = (v_attach_point_post.at(2)).cross(Eigen::Vector3d::UnitZ() * v_mavs_thrusts.at(2));
        auto mav3_torque_2_payload  = (v_attach_point_post.at(3)).cross(Eigen::Vector3d::UnitZ() * v_mavs_thrusts.at(3));

        Eigen::Vector3d total_torque =  mav0_torque_2_payload + mav1_torque_2_payload + mav2_torque_2_payload + mav3_torque_2_payload;

        std::cout<< "total torque is " << total_torque.transpose()<<std::endl; 
       
       // check total force
       auto payload_net_force = ptr_Cooperative->NetMavsWrenchToPayload().force;
       auto payload_net_torque = ptr_Cooperative->NetMavsWrenchToPayload().torque;

       // check force
       EXPECT_FLOAT_EQ(total_force[0], payload_net_force[0]);
       EXPECT_FLOAT_EQ(total_force[1], payload_net_force[1]);
       EXPECT_FLOAT_EQ(total_force[2], payload_net_force[2]);

       // check torque
        EXPECT_FLOAT_EQ(total_torque[0], payload_net_torque[0]);
        EXPECT_FLOAT_EQ(total_torque[1], payload_net_torque[1]);
        EXPECT_FLOAT_EQ(total_torque[2], payload_net_torque[2]);
 }



TEST_F(rotorTMCooperative4MAV, checkFourMAVWrench2PayloadwithRandom){
    
        // set initial posts for mavs and payload
        Eigen::Vector3d payload_init_post = Eigen::Vector3d::Zero();
        ptr_Cooperative->SetPayloadInitPost(payload_init_post);


        // assigen zero torques and balance thrust to mavs
        std::vector<Eigen::Vector3d> v_mavs_torques(4, Eigen::Vector3d::Zero());
        std::vector<double> v_mavs_thrusts(4, 3.0625);


        // static equilibrium
        ptr_Cooperative->InputControllerInput4MAVs(v_mavs_thrusts, v_mavs_torques);

        // compute interation wrenches and vars for MAVs and payload
        ptr_Cooperative->UpdateJointAndCableStatus();            

        ptr_Cooperative->UpdateVelsCollidedUAVsPayload();       
            
        ptr_Cooperative->ComputeInteractWrenches();

        ptr_Cooperative->DoOneStepInt4Robots();

        // change thrusts of mav0 - mav3
        double mav0_change_of_thrust =  RandomGenerate(-3,3); 
        v_mavs_thrusts.at(0) = 3.0625 + mav0_change_of_thrust;

        double mav1_change_of_thrust =  RandomGenerate(-3,3); 
        v_mavs_thrusts.at(1) = 3.0625 + mav1_change_of_thrust;

        double mav2_change_of_thrust =  RandomGenerate(-3,3); 
        v_mavs_thrusts.at(2) = 3.0625 + mav2_change_of_thrust;

        double mav3_change_of_thrust =  RandomGenerate(-3,3); 
        v_mavs_thrusts.at(3) = 3.0625 + mav3_change_of_thrust;


        // static equilibrium
        ptr_Cooperative->InputControllerInput4MAVs(v_mavs_thrusts, v_mavs_torques);

        // compute interation wrenches and vars for MAVs and payload
        ptr_Cooperative->UpdateJointAndCableStatus();            

        ptr_Cooperative->UpdateVelsCollidedUAVsPayload();       
            
        ptr_Cooperative->ComputeInteractWrenches();        

        // total force and torque
        Eigen::Vector3d total_force = Eigen::Vector3d::UnitZ() * (3.0625 * 4 +  mav0_change_of_thrust + mav1_change_of_thrust + mav2_change_of_thrust + mav3_change_of_thrust);
        
        auto mav0_torque_2_payload  = (v_attach_point_post.at(0)).cross(Eigen::Vector3d::UnitZ() * v_mavs_thrusts.at(0));
        auto mav1_torque_2_payload  = (v_attach_point_post.at(1)).cross(Eigen::Vector3d::UnitZ() * v_mavs_thrusts.at(1));
        auto mav2_torque_2_payload  = (v_attach_point_post.at(2)).cross(Eigen::Vector3d::UnitZ() * v_mavs_thrusts.at(2));
        auto mav3_torque_2_payload  = (v_attach_point_post.at(3)).cross(Eigen::Vector3d::UnitZ() * v_mavs_thrusts.at(3));

        Eigen::Vector3d total_torque =  mav0_torque_2_payload + mav1_torque_2_payload + mav2_torque_2_payload + mav3_torque_2_payload;

        std::cout<< "total torque is " << total_torque.transpose()<<std::endl; 
       
       // check total force
       auto payload_net_force = ptr_Cooperative->NetMavsWrenchToPayload().force;
       auto payload_net_torque = ptr_Cooperative->NetMavsWrenchToPayload().torque;

       // check force
       EXPECT_FLOAT_EQ(total_force[0], payload_net_force[0]);
       EXPECT_FLOAT_EQ(total_force[1], payload_net_force[1]);
       EXPECT_FLOAT_EQ(total_force[2], payload_net_force[2]);

       // check torque
        EXPECT_FLOAT_EQ(total_torque[0], payload_net_torque[0]);
        EXPECT_FLOAT_EQ(total_torque[1], payload_net_torque[1]);
        EXPECT_FLOAT_EQ(total_torque[2], payload_net_torque[2]);
 }


// TEST_F(rotorTMCooperative4MAV, checkVerticalVarAcc){
// //     // set initial posts for mavs and payload
//         Eigen::Vector3d payload_init_post = Eigen::Vector3d::Zero();
//         ptr_Cooperative->SetPayloadInitPost(payload_init_post);

// //     // input mav controllers' inputs to hover
// //     // 4 mav + 1 payload =  0.25 * 5 = 1.25
// //     // linear acc = 1m/s^2
// //     // mav thrust = 1.25*(9.8 + 1)/4 = 3.375

// //     Eigen::VectorXd v_mavs_thrusts = Eigen::MatrixXd::Constant(4,1,13.475);
//         std::vector<Eigen::Vector3d> v_mavs_torques(4, Eigen::Vector3d::Zero());
//         std::vector<double> v_mavs_thrusts(4, 3.375);

//         // std::cout<< "fuck point cooperative test 3"<<std::endl;
//         const double dt = 0.01;
//         const double num_steps = 10;
//         for(double t=dt ; t<=num_steps*dt ; t+= dt)
//         {
//             std::cout<<"-----------------" << t << "-----------------" <<std::endl;

//             ptr_Cooperative->InputControllerInput4MAVs(v_mavs_thrusts, v_mavs_torques);


//             // compute interation wrenches and vars for MAVs and payload
//             ptr_Cooperative->UpdateJointAndCableStatus();            

//             ptr_Cooperative->UpdateVelsCollidedUAVsPayload();       
            
//             ptr_Cooperative->ComputeInteractWrenches();

//             ptr_Cooperative->DoOneStepInt4Robots();
//             // printf("current step is %.3f \n", t);
//         }
       
//         // destiatin of payload
//         auto payload_destination_1 =  ptr_Cooperative->ptr_payload_->pose().post;

//         // desitation of mavs
//         auto mav0_desination_1 = ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.pose().post;
//         auto mav1_desination_1 = ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.pose().post;
//         auto mav2_desination_1 = ptr_Cooperative->v_ptr_uavcables_.at(2)->mav_.pose().post;
//         auto mav3_desination_1 = ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.pose().post;


//         // payload vel
//         auto payload_vel = ptr_Cooperative->ptr_payload_->vels().linear_vel;

//         std::cout<< "linear vel is " << payload_vel.transpose()<<std::endl;    


//         auto mav0_vel = ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.vels().linear_vel;
//         // write code to get mav1_vel and mav2_vel and mav3_vel
//         auto mav1_vel = ptr_Cooperative->v_ptr_uavcables_.at(1)->mav_.vels().linear_vel;
//         auto mav2_vel = ptr_Cooperative->v_ptr_uavcables_.at(2)->mav_.vels().linear_vel;
//         auto mav3_vel = ptr_Cooperative->v_ptr_uavcables_.at(3)->mav_.vels().linear_vel;


//         const double cable_length = ptr_Cooperative->v_ptr_uavcables_.at(0)->cable_.length();



// //     // input mav controllers' inputs to hover
// //     // 4 mav + 1 payload =  0.25 * 5 = 1.25
// //     // linear acc = acc2
// //     // mav thrust = 1.25*(9.8 + acc2)/4

// //     Eigen::VectorXd v_mavs_thrusts = Eigen::MatrixXd::Constant(4,1,13.475);
//         const double acc2 = 10;
//         std::vector<Eigen::Vector3d> v_mavs_torques_2(4, Eigen::Vector3d::Zero());
//         const double mavs_thrust_2 = 1.25 * (9.8 + acc2)/4;
//         std::vector<double> v_mavs_thrusts_2(4, mavs_thrust_2);

//         // std::cout<< "fuck point cooperative test 3"<<std::endl;
//         size_t num_steps_2 =2;
//         for(double t=dt ; t<=num_steps_2*dt ; t+= dt)
//         {
//             std::cout<<"-----------------" << t << "-----------------" <<std::endl;

//             ptr_Cooperative->InputControllerInput4MAVs(v_mavs_thrusts_2, v_mavs_torques_2);


//             // compute interation wrenches and vars for MAVs and payload
//             ptr_Cooperative->UpdateJointAndCableStatus();            

//             ptr_Cooperative->UpdateVelsCollidedUAVsPayload();       
            
//             ptr_Cooperative->ComputeInteractWrenches();

//             ptr_Cooperative->DoOneStepInt4Robots();
//             // printf("current step is %.3f \n", t);
//         }        



//         std::cout<<"----------------------------------------------------" <<std::endl;

//         // std::cout<<" payload_destination_1[2] is "<<  payload_destination_1[2] <<std::endl;
//         // std::cout<<"  0.5 * pow(num_steps_2*dt,2) is "<<   0.5 * pow(num_steps_2*dt,2) <<std::endl;
//         // std::cout<<"  payload_vel[2]*num_steps_2*dt is "<<   payload_vel[2]*num_steps_2*dt  <<std::endl;

//         ASSERT_FLOAT_EQ(ptr_Cooperative->ptr_payload_->pose().post[0], //
//         payload_destination_1[0]);         

//         ASSERT_FLOAT_EQ(ptr_Cooperative->ptr_payload_->pose().post[1], //
//         payload_destination_1[1]); 

//         ASSERT_FLOAT_EQ(ptr_Cooperative->ptr_payload_->pose().post[2], //
//         payload_destination_1[2] + 0.5 * acc2* pow(num_steps_2*dt,2) + payload_vel[2]*num_steps_2*dt); 

//         // payload att
//         ASSERT_FLOAT_EQ(ptr_Cooperative->ptr_payload_->pose().att.x(), //
//         0);

//         ASSERT_FLOAT_EQ(ptr_Cooperative->ptr_payload_->pose().att.y(), //
//         0);

//         ASSERT_FLOAT_EQ(ptr_Cooperative->ptr_payload_->pose().att.z(), //
//         0);

//         ASSERT_FLOAT_EQ(ptr_Cooperative->ptr_payload_->pose().att.w(), //
//         1);          


//         // mav 0 
//         ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.pose().post[0], //
//         mav0_desination_1[0]);         

//         ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.pose().post[1], //
//         mav0_desination_1[1]); 

//         ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.pose().post[2], //
//         mav0_desination_1[2] + 0.5 * acc2* pow(num_steps_2*dt,2) + mav0_vel[2]*num_steps_2*dt); 

//         // payload att
//         ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.pose().att.x(), //
//         0);

//         ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.pose().att.y(), //
//         0);

//         ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.pose().att.z(), //
//         0);

//         ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.pose().att.w(), //
//         1);              


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