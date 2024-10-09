#include <iostream> 
#include <gtest/gtest.h>
#include <memory>
#include <cmath>
#include <cstdlib>
#include <random>
#include "rotor_tm_sim/lib_cooperative.hpp"

double RandomGenerate(const double &minValue, const double &maxValue);

Eigen::Vector3d RandomUnitVector3d();


class rotorTMCooperative1MAV : public ::testing::Test
{
public:

rotorTMCooperative1MAV(){

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

    const Eigen::Vector3d v_attach_point_post{0,0,0};


    ptr_joint = std::make_shared<Joint>(v_attach_point_post);

    ptr_uavcable = std::make_shared<UAVCable>(mav_mass_property, cable_length, ptr_joint, dt); 


    ptr_payload = std::make_shared<Payload>(payload_mass_property, ptr_joint, dt);

    ptr_Cooperative = std::make_shared<Cooperative>(ptr_payload, ptr_joint, ptr_uavcable);
}

~rotorTMCooperative1MAV(){
}

protected:
    std::shared_ptr<Cooperative> ptr_Cooperative;
    std::shared_ptr<Payload> ptr_payload;
    std::shared_ptr<Joint>    ptr_joint;
    std::shared_ptr<UAVCable> ptr_uavcable;
};

// test if gTest is well integrated
TEST_F(rotorTMCooperative1MAV, checkGTest){
    ASSERT_TRUE(true);
}

// test if instance is created
TEST_F(rotorTMCooperative1MAV, checkInstanceClass){
    ASSERT_TRUE(ptr_Cooperative!=nullptr);
}

// test if initial posts are set for mavs
// test if instance is created
TEST_F(rotorTMCooperative1MAV, checkInitialPostsNoInput){
    // set position as default that payload is {0,0,0}
    ptr_Cooperative->SetPayloadInitPost();


    const double cable_length = ptr_Cooperative->v_ptr_uavcables_.at(0)->cable_.length();

    // std::cout << ptr_Cooperative->v_ptr_uavcables_.size()<<std::endl;

    // std::cout << "ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.pose().post" <<ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.pose().post<<std::endl;

    ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.pose().post[0], //
    ptr_Cooperative->ptr_payload_->pose().post[0]); 
    ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.pose().post[1], //
    ptr_Cooperative->ptr_payload_->pose().post[1]); 
    ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.pose().post[2], //
    ptr_Cooperative->ptr_payload_->pose().post[2]+cable_length); 

}




TEST_F(rotorTMCooperative1MAV, checkInitialPostsInput){

    Eigen::Vector3d payload_init_post = Eigen::Vector3d::Random();
    ptr_Cooperative->SetPayloadInitPost(payload_init_post);

   
    const double cable_length = ptr_Cooperative->v_ptr_uavcables_.at(0)->cable_.length();



    ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.pose().post[0], //
    payload_init_post[0]); 
    ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.pose().post[1], //
    payload_init_post[1]); 
    ASSERT_FLOAT_EQ(ptr_Cooperative->v_ptr_uavcables_.at(0)->mav_.pose().post[2], //
    payload_init_post[2]+cable_length); 

}



// // test if initial posts are set for mavs
TEST_F(rotorTMCooperative1MAV, checkVerticalStaticEquilibrium){

//     // set initial posts for mavs and payload
        Eigen::Vector3d payload_init_post = Eigen::Vector3d::Zero();
        ptr_Cooperative->SetPayloadInitPost(payload_init_post);

//     // input mav controllers' inputs to hover
//     // 1 + 1 payload =  0.25 + 0.25 = 0.5
//     // mav thrust = 0.5*9.8 = 4.9N

//     Eigen::VectorXd v_mavs_thrusts = Eigen::MatrixXd::Constant(4,1,13.475);
        std::vector<Eigen::Vector3d> v_mavs_torques(1, Eigen::Vector3d::Zero());
        std::vector<double> v_mavs_thrusts(1, 4.9);

        // std::cout<< "fuck point cooperative test 3"<<std::endl;
        const double dt = 0.01;
        for(double t=dt ; t<=2*dt ; t+= dt)
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
       
        std::shared_ptr<UAVCable> ptr_uavcable_local = ptr_Cooperative->v_ptr_uavcables_.at(0);
        std::shared_ptr<Payload> ptr_payload_local = ptr_Cooperative->ptr_payload_;

        const double cable_length = ptr_uavcable_local->cable_.length();

        ASSERT_FLOAT_EQ(ptr_uavcable_local->mav_.pose().post[0], //
        payload_init_post[0]); 
        ASSERT_FLOAT_EQ(ptr_uavcable_local->mav_.pose().post[1], //
        payload_init_post[1]); 
        ASSERT_FLOAT_EQ(ptr_uavcable_local->mav_.pose().post[2], //
        payload_init_post[2]+cable_length); 

        ASSERT_FLOAT_EQ(ptr_payload_local->pose().post[0], //
        payload_init_post[0]); 
        ASSERT_FLOAT_EQ(ptr_payload_local->pose().post[1], //
        payload_init_post[1]); 
        ASSERT_FLOAT_EQ(ptr_payload_local->pose().post[2], //
        payload_init_post[2]);         
}


// // test if initial posts are set for mavs
TEST_F(rotorTMCooperative1MAV, checkVerticalStaticEquilibriumRandPost){

//     // set initial posts for mavs and payload
        Eigen::Vector3d payload_init_post = Eigen::Vector3d::Random();
        ptr_Cooperative->SetPayloadInitPost(payload_init_post);

        std::vector<Eigen::Vector3d> v_mavs_torques(1, Eigen::Vector3d::Zero());
        std::vector<double> v_mavs_thrusts(1, 4.9);

        // std::cout<< "fuck point cooperative test 3"<<std::endl;
        const double dt = 0.01;
        for(double t=dt ; t<=1000*dt ; t+= dt)
        {
            std::cout<<"-----------------" << t << "-----------------" <<std::endl;

            ptr_Cooperative->InputControllerInput4MAVs(v_mavs_thrusts, v_mavs_torques);


            // compute interation wrenches and vars for MAVs and payload
            ptr_Cooperative->UpdateJointAndCableStatus();                
            
            ptr_Cooperative->ComputeInteractWrenches();

            ptr_Cooperative->DoOneStepInt4Robots();
            // printf("current step is %.3f \n", t);
        }
       
        std::shared_ptr<UAVCable> ptr_uavcable_local = ptr_Cooperative->v_ptr_uavcables_.at(0);
        std::shared_ptr<Payload> ptr_payload_local = ptr_Cooperative->ptr_payload_;

        const double cable_length = ptr_uavcable_local->cable_.length();

        ASSERT_FLOAT_EQ(ptr_uavcable_local->mav_.pose().post[0], //
        payload_init_post[0]); 

        // ASSERT_FLOAT_EQ(ptr_payload_local->v_ptr_joints_.at(0)->ptr_UAVCable()->mav_.pose().post[0], //
        // payload_init_post[0]); 
        
        
        ASSERT_FLOAT_EQ(ptr_uavcable_local->mav_.pose().post[1], //
        payload_init_post[1]); 
        ASSERT_FLOAT_EQ(ptr_uavcable_local->mav_.pose().post[2], //
        payload_init_post[2]+cable_length); 

        ASSERT_FLOAT_EQ(ptr_payload_local->pose().post[0], //
        payload_init_post[0]); 
        ASSERT_FLOAT_EQ(ptr_payload_local->pose().post[1], //
        payload_init_post[1]); 
        ASSERT_FLOAT_EQ(ptr_payload_local->pose().post[2], //
        payload_init_post[2]);      
}



// // test if initial posts are set for mavs
TEST_F(rotorTMCooperative1MAV, checkVerticalEquilibriumWithVel){

//     // set initial posts for mavs and payload
        Eigen::Vector3d payload_init_post = Eigen::Vector3d::Zero();
        ptr_Cooperative->SetPayloadInitPost(payload_init_post);

        std::vector<Eigen::Vector3d> v_mavs_torques(1, Eigen::Vector3d::Zero());
        std::vector<double> v_mavs_thrusts(1, 4.9);

        std::shared_ptr<UAVCable> ptr_uavcable_local = ptr_Cooperative->v_ptr_uavcables_.at(0);
        std::shared_ptr<Payload> ptr_payload_local = ptr_Cooperative->ptr_payload_;

        // set a constant velocity for payload and mav
        Eigen::Vector3d vertical_vel(0,0,1);
        ptr_payload_local->SetLinearVel(vertical_vel);

        ptr_uavcable_local->mav_.SetLinearVel(vertical_vel);

        // std::cout<< "fuck point cooperative test 3"<<std::endl;
        const double dt = 0.01;
        int num_steps =1000;
        for(double t=dt ; t<=num_steps*dt ; t+= dt)
        {
            std::cout<<"-----------------" << t << "-----------------" <<std::endl;

            ptr_Cooperative->InputControllerInput4MAVs(v_mavs_thrusts, v_mavs_torques);


            // compute interation wrenches and vars for MAVs and payload
            ptr_Cooperative->UpdateJointAndCableStatus();                
            
            ptr_Cooperative->ComputeInteractWrenches();

            ptr_Cooperative->DoOneStepInt4Robots();
            // printf("current step is %.3f \n", t);
        }
       


        const double cable_length = ptr_uavcable_local->cable_.length();

        ASSERT_FLOAT_EQ(ptr_uavcable_local->mav_.pose().post[0], //
        payload_init_post[0] + vertical_vel[0] * num_steps*dt ); 

        // ASSERT_FLOAT_EQ(ptr_payload_local->v_ptr_joints_.at(0)->ptr_UAVCable()->mav_.pose().post[0], //
        // payload_init_post[0]); 
                
        ASSERT_FLOAT_EQ(ptr_uavcable_local->mav_.pose().post[1], //
        payload_init_post[1] + vertical_vel[1] * num_steps*dt ); 

        ASSERT_FLOAT_EQ(ptr_uavcable_local->mav_.pose().post[2], //
        payload_init_post[2]+cable_length  + vertical_vel[2] * num_steps*dt ); 

        ASSERT_FLOAT_EQ(ptr_payload_local->pose().post[0], //
        payload_init_post[0] +  vertical_vel[0] *  num_steps*dt ); 
        ASSERT_FLOAT_EQ(ptr_payload_local->pose().post[1], //
        payload_init_post[1]  +  vertical_vel[1] *  num_steps*dt ); 
        ASSERT_FLOAT_EQ(ptr_payload_local->pose().post[2], //
        payload_init_post[2] +  vertical_vel[2] * num_steps*dt ); 
}



TEST_F(rotorTMCooperative1MAV, checkVerticalConstAcc){

//     // set initial posts for mavs and payload
        Eigen::Vector3d payload_init_post = Eigen::Vector3d::Zero();
        ptr_Cooperative->SetPayloadInitPost(payload_init_post);

//     // input mav controllers' inputs to generate 1m/s^2 acc
//     // 1 + 1 payload =  0.25 + 0.25 = 0.5
//     // mav thrust = 0.5*(1+9.8) = 5.4N
//     // tension force should be t = 2.7 from t-0.25*9.8 = 0.25*1

//     Eigen::VectorXd v_mavs_thrusts = Eigen::MatrixXd::Constant(4,1,13.475);
        std::vector<Eigen::Vector3d> v_mavs_torques(1, Eigen::Vector3d::Zero());
        std::vector<double> v_mavs_thrusts(1, 5.4);

        // // std::cout<< "fuck point cooperative test 1"<<std::endl;
        // ptr_Cooperative->InputControllerInput4MAVs(v_mavs_thrusts, v_mavs_torques);

        // // compute interation wrenches and vars for MAVs and payload
        // ptr_Cooperative->UpdateJointAndCableStatus();



        // call one step dynamic simulation for MAVs and payload
        // std::cout<< "fuck point cooperative test 3"<<std::endl;
        const double dt = 0.01;
        const double num_steps = 1000;
        for(double t=dt ; t<=num_steps*dt ; t+= dt)
        {
                 std:: cout << "------------------" << t <<"------------------" <<std::endl;
                 ptr_Cooperative->InputControllerInput4MAVs(v_mavs_thrusts, v_mavs_torques);
                 ptr_Cooperative->UpdateJointAndCableStatus();  
                 std::cout << "joint acc is "<< ptr_joint->accs().linear_acc.transpose()<<std::endl;
                 std::cout << "payload acc is "<< ptr_payload->accs().linear_acc.transpose()<<std::endl;

                 std::cout << "joint post is "<< ptr_joint->pose().post.transpose()<<std::endl;
                 ptr_Cooperative->ComputeInteractWrenches();

                 ptr_Cooperative->DoOneStepInt4Robots();
                 std:: cout << "------------------------------------" <<std::endl;
                // printf("current step is %.3f \n", t);
        }
       
        std::shared_ptr<UAVCable> ptr_uavcable_local = ptr_Cooperative->v_ptr_uavcables_.at(0);
        std::shared_ptr<Payload> ptr_payload_local = ptr_Cooperative->ptr_payload_;

        const double cable_length = ptr_uavcable_local->cable_.length();

        ASSERT_FLOAT_EQ(ptr_uavcable_local->mav_.pose().post[0], //
        payload_init_post[0]); 
        ASSERT_FLOAT_EQ(ptr_uavcable_local->mav_.pose().post[1], //
        payload_init_post[1]); 
        ASSERT_FLOAT_EQ(ptr_uavcable_local->mav_.pose().post[2], //
        0.5 * pow(num_steps*dt,2) + payload_init_post[2]+cable_length); 

        ASSERT_FLOAT_EQ(ptr_payload_local->pose().post[0], //
        payload_init_post[0]); 
        ASSERT_FLOAT_EQ(ptr_payload_local->pose().post[1], //
        payload_init_post[1]); 
        ASSERT_FLOAT_EQ(ptr_payload_local->pose().post[2], //
         0.5 * pow(num_steps*dt,2) + payload_init_post[2]);         
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