#define UNIT_TEST
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

  
    double dt = 0.01;

    double payload_mass =  0.250;
    Eigen::Matrix3d payload_inertia = Eigen::Matrix3d::Identity();
    payload_inertia(0,0)= 0.000601;
    payload_inertia(1,1)= 0.000589;
    payload_inertia(2,2)= 0.01076;

    // {Ixx: 0.000601, Ixy: 0.0, Ixz: 0.0, Iyx: 0.0, Iyy: 0.000589, Iyz: 0.0, Izx: 0.0, Izy: 0.0, Izz: 0.01076}

    MassProperty payload_mass_property = {payload_mass, payload_inertia};


    ptr_payload = std::make_shared<Payload>(payload_mass_property, dt);

}

~rotorTMPayloadTest(){
}

protected:
    std::shared_ptr<Payload> ptr_payload;
};




// test if gTest is well integrated
TEST_F(rotorTMPayloadTest, checkGTest){
    ASSERT_TRUE(true);
}

// test if instance is created
TEST_F(rotorTMPayloadTest, checkInstanceClass){
    ASSERT_TRUE(ptr_payload!=nullptr);
}


//
TEST_F(rotorTMPayloadTest, checkDynamicModelOutput){

//     pl_state [-4.27156898e-01  3.01191057e-01 -4.57448164e-04  5.70588302e-03
//  -5.37341883e-02  8.59151985e-04  9.99912240e-01  1.25964140e-02
//   3.83292512e-03  1.47006375e-03 -3.49605431e-02  4.76719939e-02
//   1.68719662e-02]
    // write a std vector with a size of 13
    object_state payload_state_before = {-4.27156898e-01,  3.01191057e-01, -4.57448164e-04,  5.70588302e-03,//
                                         -5.37341883e-02,  8.59151985e-04,  9.99912240e-01,  1.25964140e-02,//
                                         3.83292512e-03, 1.47006375e-03, -3.49605431e-02,  4.76719939e-02,//
                                        1.68719662e-02};
    //                                    
    ptr_payload->SetPayloadStates(payload_state_before);

    // input net wrenches
    Wrench mavs_net_wrench;
    // mavs_net_wrench.force = {0.05499161, -0.0303093,  12.26226555};
    // mavs_net_wrench.torque = {-0.05462257, -0.02200961, -0.00163718};

    // pl_net_F = np.array([1, 2, 12.26226555])

    // pl_net_M = np.array([-0.5462257, -0.2200961, -0.163718])
    mavs_net_wrench.force = {1, 2, 24};
    mavs_net_wrench.torque = {0.1,0.2,0.3};

    // robot inertia para
    // invML [[ 3.99952167e+00 -1.01852847e-05 -1.44153950e-02]
    //  [-1.01852847e-05  3.99952653e+00  7.88281800e-03]
    //  [-1.44153950e-02  7.88281800e-03  8.00119073e-01]]
    // C [[-1.22111436e-03  8.96552891e-04 -5.21153784e-03]
    //  [ 9.83579874e-04  1.35992018e-03 -7.43747245e-04]
    //  [-2.74156775e-05 -2.78317334e-05 -1.59890823e-04]]
    // D [[ 1.22111436e-03 -9.83579874e-04  2.74156775e-05]
    //  [-8.96552891e-04 -1.35992018e-03  2.78317334e-05]
    //  [ 5.21153784e-03  7.43747245e-04  1.59890823e-04]]
    // E [[-9.24693356e-02  5.78602926e-04 -3.02249039e-04]
    //  [ 5.78602926e-04 -9.33014117e-02  2.13581421e-03]
    //  [-3.02249039e-04  2.13581421e-03 -5.33502009e-05]]    
    CooperIntertPara   interaction_parameters;
    Eigen::Matrix3d C;
    C << -1.22111436e-03, 8.96552891e-04, -5.21153784e-03,
        9.83579874e-04, 1.35992018e-03, -7.43747245e-04,
        -2.74156775e-05, -2.78317334e-05, -1.59890823e-04;

    Eigen::Matrix3d D;
    D << 1.22111436e-03, -9.83579874e-04, 2.74156775e-05,
        -8.96552891e-04, -1.35992018e-03, 2.78317334e-05,
        5.21153784e-03, 7.43747245e-04, 1.59890823e-04;

    Eigen::Matrix3d E;
    E << -9.24693356e-02, 5.78602926e-04, -3.02249039e-04,
        5.78602926e-04, -9.33014117e-02, 2.13581421e-03,
        -3.02249039e-04, 2.13581421e-03, -5.33502009e-05;

    Eigen::Matrix3d m_mass_matrix;
    // assign m_mass_matrix with [[ 2.50046137e-01 -8.24240601e-06  4.50505296e-03]
//  [-8.24240601e-06  2.50034451e-01 -2.46350194e-03]
//  [ 4.50505296e-03 -2.46350194e-03  1.24991941e+00]]
    m_mass_matrix << 2.50046137e-01, -8.24240601e-06, 4.50505296e-03,
        -8.24240601e-06, 2.50034451e-01, -2.46350194e-03,
        4.50505296e-03, -2.46350194e-03, 1.24991941e+00;

    interaction_parameters.m_C = C;
    interaction_parameters.m_D = D;
    interaction_parameters.m_E = E;
    interaction_parameters.m_mass_matrix = m_mass_matrix;

    ptr_payload->InputPayloadInteractPara(interaction_parameters);

    // input net wrenches

    ptr_payload->InputDronesNetWrenches(mavs_net_wrench);

    // call integration 
    ptr_payload->DoOneStepInt();


    auto payload_acc = ptr_payload->accs().linear_acc;

    auto payload_angular_acc = ptr_payload->accs().angular_acc;



    EXPECT_FLOAT_EQ(payload_acc[0], //
        3.655430966467343  
        );

    EXPECT_FLOAT_EQ(payload_acc[1], //
        8.169051452588482
        ); 

    EXPECT_FLOAT_EQ(payload_acc[2], //
         9.417998504300883
        ); 


    EXPECT_FLOAT_EQ(payload_angular_acc[0], //
        2.044370358127562   
        ); 

    EXPECT_FLOAT_EQ(payload_angular_acc[1], //
        2.787783111380493
        ); 

    EXPECT_FLOAT_EQ(payload_angular_acc[2], //
        28.55143028273914 
        );         
        
}




TEST_F(rotorTMPayloadTest, checkDynamicModelOutput2){

//     pl_state [-4.27156898e-01  3.01191057e-01 -4.57448164e-04  5.70588302e-03
//  -5.37341883e-02  8.59151985e-04  9.99912240e-01  1.25964140e-02
//   3.83292512e-03  1.47006375e-03 -3.49605431e-02  4.76719939e-02
//   1.68719662e-02]
    // write a std vector with a size of 13
    object_state payload_state_before = {-4.27156898e-01,  3.01191057e-01, -4.57448164e-04,  5.70588302e-03,//
                                         -5.37341883e-02,  8.59151985e-04,  9.99912240e-01,  1.25964140e-02,//
                                         3.83292512e-03, 1.47006375e-03, -3.49605431e-02,  4.76719939e-02,//
                                        1.68719662e-02};
    //                                    
    ptr_payload->SetPayloadStates(payload_state_before);

    // input net wrenches
    Wrench mavs_net_wrench;
    // mavs_net_wrench.force = {0.05499161, -0.0303093,  12.26226555};
    // mavs_net_wrench.torque = {-0.05462257, -0.02200961, -0.00163718};

    // pl_net_F = np.array([2, 1, 25])

    // pl_net_M = np.array([0.12, 0.21, 0.31])
    mavs_net_wrench.force = {2, 1, 25};
    mavs_net_wrench.torque = {0.12, 0.21, 0.31};

    // robot inertia para
    // invML [[ 3.99952167e+00 -1.01852847e-05 -1.44153950e-02]
    //  [-1.01852847e-05  3.99952653e+00  7.88281800e-03]
    //  [-1.44153950e-02  7.88281800e-03  8.00119073e-01]]
    // C [[-1.22111436e-03  8.96552891e-04 -5.21153784e-03]
    //  [ 9.83579874e-04  1.35992018e-03 -7.43747245e-04]
    //  [-2.74156775e-05 -2.78317334e-05 -1.59890823e-04]]
    // D [[ 1.22111436e-03 -9.83579874e-04  2.74156775e-05]
    //  [-8.96552891e-04 -1.35992018e-03  2.78317334e-05]
    //  [ 5.21153784e-03  7.43747245e-04  1.59890823e-04]]
    // E [[-9.24693356e-02  5.78602926e-04 -3.02249039e-04]
    //  [ 5.78602926e-04 -9.33014117e-02  2.13581421e-03]
    //  [-3.02249039e-04  2.13581421e-03 -5.33502009e-05]]    
    CooperIntertPara   interaction_parameters;
    Eigen::Matrix3d C;
    C << -1.22111436e-03, 8.96552891e-04, -5.21153784e-03,
        9.83579874e-04, 1.35992018e-03, -7.43747245e-04,
        -2.74156775e-05, -2.78317334e-05, -1.59890823e-04;

    Eigen::Matrix3d D;
    D << 1.22111436e-03, -9.83579874e-04, 2.74156775e-05,
        -8.96552891e-04, -1.35992018e-03, 2.78317334e-05,
        5.21153784e-03, 7.43747245e-04, 1.59890823e-04;

    Eigen::Matrix3d E;
    E << -9.24693356e-02, 5.78602926e-04, -3.02249039e-04,
        5.78602926e-04, -9.33014117e-02, 2.13581421e-03,
        -3.02249039e-04, 2.13581421e-03, -5.33502009e-05;

    Eigen::Matrix3d m_mass_matrix;
    // assign m_mass_matrix with [[ 2.50046137e-01 -8.24240601e-06  4.50505296e-03]
//  [-8.24240601e-06  2.50034451e-01 -2.46350194e-03]
//  [ 4.50505296e-03 -2.46350194e-03  1.24991941e+00]]
    m_mass_matrix << 2.50046137e-01, -8.24240601e-06, 4.50505296e-03,
        -8.24240601e-06, 2.50034451e-01, -2.46350194e-03,
        4.50505296e-03, -2.46350194e-03, 1.24991941e+00;

    interaction_parameters.m_C = C;
    interaction_parameters.m_D = D;
    interaction_parameters.m_E = E;
    interaction_parameters.m_mass_matrix = m_mass_matrix;

    ptr_payload->InputPayloadInteractPara(interaction_parameters);

    // input net wrenches

    ptr_payload->InputDronesNetWrenches(mavs_net_wrench);

    // call integration 
    ptr_payload->DoOneStepInt();


    auto payload_acc = ptr_payload->accs().linear_acc;

    auto payload_angular_acc = ptr_payload->accs().angular_acc;


// acc [ 7.641717287065457  4.175445224772504 10.19747196327354 ]
// angular_acc [ 2.391523414732093  2.940448766655036 29.508017003339532]

    EXPECT_FLOAT_EQ(payload_acc[0], //
        7.641717287065457
        );

    EXPECT_FLOAT_EQ(payload_acc[1], //
        4.175445224772504
        ); 

    EXPECT_FLOAT_EQ(payload_acc[2], //
        10.19747196327354
        ); 


    EXPECT_FLOAT_EQ(payload_angular_acc[0], //
        2.391523414732093
        ); 

    EXPECT_FLOAT_EQ(payload_angular_acc[1], //
        2.940448766655036
        ); 

    EXPECT_FLOAT_EQ(payload_angular_acc[2], //
         29.508017003339532
        );         
        
}


TEST_F(rotorTMPayloadTest, checkDynamicModelOutput3){

//     pl_state [-4.27156898e-01  3.01191057e-01 -4.57448164e-04  5.70588302e-03
//  -5.37341883e-02  8.59151985e-04  9.99912240e-01  1.25964140e-02
//   3.83292512e-03  1.47006375e-03 -3.49605431e-02  4.76719939e-02
//   1.68719662e-02]
    // write a std vector with a size of 13
    object_state payload_state_before = {-4.27156898e-01,  3.01191057e-01, -4.57448164e-04,  5.70588302e-03,//
                                         -5.37341883e-02,  8.59151985e-04,  9.99912240e-01,  1.25964140e-02,//
                                         3.83292512e-03, 1.47006375e-03, -3.49605431e-02,  4.76719939e-02,//
                                        1.68719662e-02};
    //                                    
    ptr_payload->SetPayloadStates(payload_state_before);

    // input net wrenches
    Wrench mavs_net_wrench;
    // mavs_net_wrench.force = {0.05499161, -0.0303093,  12.26226555};
    // mavs_net_wrench.torque = {-0.05462257, -0.02200961, -0.00163718};

// pl_net_F = np.array([10.123, 12.432, 25.123])

// pl_net_M = np.array([0.12234, 0.3523, 0.71])
    mavs_net_wrench.force = {10.123, 12.432, 25.123};
    mavs_net_wrench.torque = {0.12234, 0.3523, 0.71};

    // robot inertia para
    // invML [[ 3.99952167e+00 -1.01852847e-05 -1.44153950e-02]
    //  [-1.01852847e-05  3.99952653e+00  7.88281800e-03]
    //  [-1.44153950e-02  7.88281800e-03  8.00119073e-01]]
    // C [[-1.22111436e-03  8.96552891e-04 -5.21153784e-03]
    //  [ 9.83579874e-04  1.35992018e-03 -7.43747245e-04]
    //  [-2.74156775e-05 -2.78317334e-05 -1.59890823e-04]]
    // D [[ 1.22111436e-03 -9.83579874e-04  2.74156775e-05]
    //  [-8.96552891e-04 -1.35992018e-03  2.78317334e-05]
    //  [ 5.21153784e-03  7.43747245e-04  1.59890823e-04]]
    // E [[-9.24693356e-02  5.78602926e-04 -3.02249039e-04]
    //  [ 5.78602926e-04 -9.33014117e-02  2.13581421e-03]
    //  [-3.02249039e-04  2.13581421e-03 -5.33502009e-05]]    
    CooperIntertPara   interaction_parameters;
    Eigen::Matrix3d C;
    C << -1.22111436e-03, 8.96552891e-04, -5.21153784e-03,
        9.83579874e-04, 1.35992018e-03, -7.43747245e-04,
        -2.74156775e-05, -2.78317334e-05, -1.59890823e-04;

    Eigen::Matrix3d D;
    D << 1.22111436e-03, -9.83579874e-04, 2.74156775e-05,
        -8.96552891e-04, -1.35992018e-03, 2.78317334e-05,
        5.21153784e-03, 7.43747245e-04, 1.59890823e-04;

    Eigen::Matrix3d E;
    E << -9.24693356e-02, 5.78602926e-04, -3.02249039e-04,
        5.78602926e-04, -9.33014117e-02, 2.13581421e-03,
        -3.02249039e-04, 2.13581421e-03, -5.33502009e-05;

    Eigen::Matrix3d m_mass_matrix;
    // assign m_mass_matrix with [[ 2.50046137e-01 -8.24240601e-06  4.50505296e-03]
//  [-8.24240601e-06  2.50034451e-01 -2.46350194e-03]
//  [ 4.50505296e-03 -2.46350194e-03  1.24991941e+00]]
    m_mass_matrix << 2.50046137e-01, -8.24240601e-06, 4.50505296e-03,
        -8.24240601e-06, 2.50034451e-01, -2.46350194e-03,
        4.50505296e-03, -2.46350194e-03, 1.24991941e+00;

    interaction_parameters.m_C = C;
    interaction_parameters.m_D = D;
    interaction_parameters.m_E = E;
    interaction_parameters.m_mass_matrix = m_mass_matrix;

    ptr_payload->InputPayloadInteractPara(interaction_parameters);

    // input net wrenches

    ptr_payload->InputDronesNetWrenches(mavs_net_wrench);

    // call integration 
    ptr_payload->DoOneStepInt();


    auto payload_acc = ptr_payload->accs().linear_acc;

    auto payload_angular_acc = ptr_payload->accs().angular_acc;


// acc [ 7.641717287065457  4.175445224772504 10.19747196327354 ]
// angular_acc [ 2.391523414732093  2.940448766655036 29.508017003339532]

    EXPECT_FLOAT_EQ(payload_acc[0], //
        40.12610720675136
        );

    EXPECT_FLOAT_EQ(payload_acc[1], //
        49.896066176693466 
        ); 

    EXPECT_FLOAT_EQ(payload_acc[2], //
        10.27410419369454 
        ); 


    EXPECT_FLOAT_EQ(payload_angular_acc[0], //
        2.293552619512362   
        ); 

    EXPECT_FLOAT_EQ(payload_angular_acc[1], //
        4.305774264765707
        ); 

    EXPECT_FLOAT_EQ(payload_angular_acc[2], //
        66.97291031887794 
        );         
        
}




//
TEST_F(rotorTMPayloadTest, checkTransRotDynamics){

//     pl_state [-4.27156898e-01  3.01191057e-01 -4.57448164e-04  5.70588302e-03
//  -5.37341883e-02  8.59151985e-04  9.99912240e-01  1.25964140e-02
//   3.83292512e-03  1.47006375e-03 -3.49605431e-02  4.76719939e-02
//   1.68719662e-02]
    // write a std vector with a size of 13
    object_state payload_state_before = {-4.27156898e-01,  3.01191057e-01, -4.57448164e-04,  5.70588302e-03,//
                                         -5.37341883e-02,  8.59151985e-04,  9.99912240e-01,  1.25964140e-02,//
                                         3.83292512e-03, 1.47006375e-03, -3.49605431e-02,  4.76719939e-02,//
                                         1.68719662e-02};
    //                                    
    ptr_payload->SetPayloadStates(payload_state_before);

    // input net wrenches
    Wrench mavs_net_wrench;
    // input net wrenches
    // pl_net_F [ 0.05499161 -0.0303093  12.26226555]
    // pl_net_M [-0.05462257 -0.02200961 -0.00163718]       
    mavs_net_wrench.force = {0.05499161, -0.0303093,  12.26226555};
    mavs_net_wrench.torque = {-0.05462257, -0.02200961, -0.00163718};


    // robot inertia para
    // invML [[ 3.99952167e+00 -1.01852847e-05 -1.44153950e-02]
    //  [-1.01852847e-05  3.99952653e+00  7.88281800e-03]
    //  [-1.44153950e-02  7.88281800e-03  8.00119073e-01]]
    // C [[-1.22111436e-03  8.96552891e-04 -5.21153784e-03]
    //  [ 9.83579874e-04  1.35992018e-03 -7.43747245e-04]
    //  [-2.74156775e-05 -2.78317334e-05 -1.59890823e-04]]
    // D [[ 1.22111436e-03 -9.83579874e-04  2.74156775e-05]
    //  [-8.96552891e-04 -1.35992018e-03  2.78317334e-05]
    //  [ 5.21153784e-03  7.43747245e-04  1.59890823e-04]]
    // E [[-9.24693356e-02  5.78602926e-04 -3.02249039e-04]
    //  [ 5.78602926e-04 -9.33014117e-02  2.13581421e-03]
    //  [-3.02249039e-04  2.13581421e-03 -5.33502009e-05]]    
    CooperIntertPara   interaction_parameters;
    Eigen::Matrix3d C;
    C << -1.22111436e-03, 8.96552891e-04, -5.21153784e-03,
        9.83579874e-04, 1.35992018e-03, -7.43747245e-04,
        -2.74156775e-05, -2.78317334e-05, -1.59890823e-04;

    Eigen::Matrix3d D;
    D << 1.22111436e-03, -9.83579874e-04, 2.74156775e-05,
        -8.96552891e-04, -1.35992018e-03, 2.78317334e-05,
        5.21153784e-03, 7.43747245e-04, 1.59890823e-04;

    Eigen::Matrix3d E;
    E << -9.24693356e-02, 5.78602926e-04, -3.02249039e-04,
        5.78602926e-04, -9.33014117e-02, 2.13581421e-03,
        -3.02249039e-04, 2.13581421e-03, -5.33502009e-05;

    Eigen::Matrix3d m_mass_matrix;
    // assign m_mass_matrix with [[ 2.50046137e-01 -8.24240601e-06  4.50505296e-03]
//  [-8.24240601e-06  2.50034451e-01 -2.46350194e-03]
//  [ 4.50505296e-03 -2.46350194e-03  1.24991941e+00]]
    m_mass_matrix << 2.50046137e-01, -8.24240601e-06, 4.50505296e-03,
        -8.24240601e-06, 2.50034451e-01, -2.46350194e-03,
        4.50505296e-03, -2.46350194e-03, 1.24991941e+00;

    interaction_parameters.m_C = C;
    interaction_parameters.m_D = D;
    interaction_parameters.m_E = E;
    interaction_parameters.m_mass_matrix = m_mass_matrix;

    ptr_payload->InputPayloadInteractPara(interaction_parameters);

 
    ptr_payload->InputDronesNetWrenches(mavs_net_wrench);

    // call integration 
    // ptr_payload->DoOneStepInt();



    // get the state after integration
    // pl_vel [ 0.00570588 -0.05373419  0.00085915]
    // pl_acc [ 4.36127361e-02 -2.35743782e-02 -1.47000610e-05]
    // pl_quat [ 0.0001164  -0.01748144  0.02370194  0.00880249]
    // pl_angularacc [-0.03773087 -0.1579497  -0.03644387]
    object_state payload_state_new = ptr_payload->state();

    auto payload_angular_acc = ptr_payload->ComputeRotDynamics();

    // std::printf("payload_angular_acc: %.10f, %.10f, %.10f\n", payload_angular_acc[0], payload_angular_acc[1], payload_angular_acc[2]);

    ptr_payload->SetAngularAcc(payload_angular_acc);
    auto payload_acc = ptr_payload->ComputeTransDynamics();

    
    // std::cout<<"payload_acc: "<<payload_acc.transpose()<<std::endl;
    
// pl_acc [ 4.36127361e-02 -2.35743782e-02 -1.47000610e-05]
// pl_angularacc [-0.03773087 -0.1579497  -0.03644387]
    EXPECT_DOUBLE_EQ(payload_angular_acc[0], //
       -0.037730865525696
        ); 

    EXPECT_DOUBLE_EQ(payload_angular_acc[1], //
       -0.157949661187575
        ); 

    EXPECT_DOUBLE_EQ(payload_angular_acc[2], //
        -0.036443916622249
        );   


    EXPECT_DOUBLE_EQ(payload_acc[0], //
        0.043612733868192    
        );

    EXPECT_DOUBLE_EQ(payload_acc[1], //
        -0.02357439290819
        ); 

    EXPECT_NEAR(payload_acc[2], //
         0.009985299106157, 1e-6
        ); 
        
}



TEST_F(rotorTMPayloadTest, checkTransRotDynamicsCase2){

//     pl_state [-4.27147024e-01  3.01102123e-01 -4.56031150e-04
//   5.78293027e-03  -5.37728396e-02  8.59330522e-04  
//   9.99912428e-01  1.25673556e-02 3.87118281e-03  1.48438391e-03 
//   -3.50203897e-02  4.74107821e-02 1.68105557e-02]
    // write a std vector with a size of 13
    object_state payload_state_before = {-4.27147024e-01,  3.01102123e-01, -4.56031150e-04,
    5.78293027e-03,  -5.37728396e-02,  8.59330522e-04,  
    9.99912428e-01,  1.25673556e-02, 3.87118281e-03,  1.48438391e-03, 
    -3.50203897e-02,  4.74107821e-02, 1.68105557e-02};
    //                                    
    ptr_payload->SetPayloadStates(payload_state_before);

    // input net wrenches
    Wrench mavs_net_wrench;
    // input net wrenches
// pl_net_F [ 0.05601574 -0.03019857 12.26230337]
// pl_net_M [-0.05447809 -0.02199673 -0.00164717]     
    mavs_net_wrench.force = {0.05601574, -0.03019857, 12.26230337};
    mavs_net_wrench.torque = {-0.05447809, -0.02199673, -0.00164717};


    // robot inertia para
// C [[-1.22489500e-03  8.87934325e-04 -5.19997860e-03]
//  [ 9.68583320e-04  1.36406204e-03 -7.42227793e-04]
//  [-2.70354651e-05 -2.78888803e-05 -1.60659371e-04]]


    CooperIntertPara   interaction_parameters;
    Eigen::Matrix3d C;
    C << -1.22489500e-03,  8.87934325e-04, -5.19997860e-03,
          9.68583320e-04,  1.36406204e-03, -7.42227793e-04,
         -2.70354651e-05, -2.78888803e-05, -1.60659371e-04;

// D [[ 1.22489500e-03 -9.68583320e-04  2.70354651e-05]
//  [-8.87934325e-04 -1.36406204e-03  2.78888803e-05]
//  [ 5.19997860e-03  7.42227793e-04  1.60659371e-04]]

    Eigen::Matrix3d D;
    D << 1.22489500e-03, -9.68583320e-04,  2.70354651e-05,
        -8.87934325e-04, -1.36406204e-03,  2.78888803e-05,
        5.19997860e-03,  7.42227793e-04,  1.60659371e-04;

// E [[-9.24733373e-02  5.80365557e-04 -3.01691934e-04]
//  [ 5.80365557e-04 -9.32948498e-02  2.13094794e-03]
//  [-3.01691934e-04  2.13094794e-03 -5.30405940e-05]]
    Eigen::Matrix3d E;
    E <<-9.24733373e-02,  5.80365557e-04, -3.01691934e-04,
        5.80365557e-04, -9.32948498e-02,  2.13094794e-03,
        -3.01691934e-04,  2.13094794e-03, -5.30405940e-05;

    Eigen::Matrix3d m_mass_matrix;
    // assign m_mass_matrix with array([[ 2.500466700089323e-01, -8.502346069232481e-06,
    //      4.588417208777992e-03],
    //    [-8.502346069232483e-06,  2.500343647169297e-01,
    //     -2.454342367846149e-03],
    //    [ 4.588417208777993e-03, -2.454342367846148e-03,
    //      1.249918964499427e+00]])
    m_mass_matrix << 2.500466700089323e-01, -8.502346069232481e-06, 4.588417208777992e-03,
        -8.502346069232483e-06,  2.500343647169297e-01, -2.454342367846149e-03,
        4.588417208777993e-03, -2.454342367846148e-03,    1.249918964499427e+00;

    interaction_parameters.m_C = C;
    interaction_parameters.m_D = D;
    interaction_parameters.m_E = E;
    interaction_parameters.m_mass_matrix = m_mass_matrix;

    ptr_payload->InputPayloadInteractPara(interaction_parameters);

 
    ptr_payload->InputDronesNetWrenches(mavs_net_wrench);

    // call integration 
    // ptr_payload->DoOneStepInt();



    // get the state after integration
    object_state payload_state_new = ptr_payload->state();

    auto payload_angular_acc = ptr_payload->ComputeRotDynamics();



    ptr_payload->SetAngularAcc(payload_angular_acc);
    auto payload_acc = ptr_payload->ComputeTransDynamics();

    // std::printf("payload_angular_acc: %.20f, %.20f, %.20f\n", payload_angular_acc[0], payload_angular_acc[1], payload_angular_acc[2]);


    // std::printf("payload_acc: %.20f, %.20f, %.20f\n", payload_acc[0], payload_acc[1], payload_acc[2]);

    
    // std::cout<<"payload_acc: "<<payload_acc.transpose()<<std::endl;
    
    // dbg(payload_angular_acc);
    // dbg(payload_acc);
    EXPECT_DOUBLE_EQ(payload_angular_acc[0], //
       -0.037386122996028094
        ); 
    // -0.03738612299602832312

    EXPECT_DOUBLE_EQ(payload_angular_acc[1], //
        -0.157984189667433
        );
    //  -0.15798418966743302261, 

    EXPECT_DOUBLE_EQ(payload_angular_acc[2], //
        -0.036617819456007714
        ); 
    //  -0.03661781945600777605

    EXPECT_DOUBLE_EQ(payload_acc[0], //
        0.04442904011000038
        ); 
    //  0.04442904010999863124, 
    EXPECT_DOUBLE_EQ(payload_acc[1], //
        -0.02349029249940795
        );
    // -0.02349029249940797967, 
    EXPECT_DOUBLE_EQ(payload_acc[2], //
        0.010015413620452662
        ); 
    //  0.01001541362045088590    
}












TEST_F(rotorTMPayloadTest, checkIntwithPythonData){

//     pl_state [-4.27156898e-01  3.01191057e-01 -4.57448164e-04  5.70588302e-03
//  -5.37341883e-02  8.59151985e-04  9.99912240e-01  1.25964140e-02
//   3.83292512e-03  1.47006375e-03 -3.49605431e-02  4.76719939e-02
//   1.68719662e-02]
    // write a std vector with a size of 13
    object_state payload_state_before = {-4.27156898e-01,  3.01191057e-01, -4.57448164e-04,  5.70588302e-03,//
                                         -5.37341883e-02,  8.59151985e-04,  9.99912240e-01,  1.25964140e-02,//
                                         3.83292512e-03, 1.47006375e-03, -3.49605431e-02,  4.76719939e-02,//
                                         1.68719662e-02};
    //                                    
    ptr_payload->SetPayloadStates(payload_state_before);

    // input net wrenches
    Wrench mavs_net_wrench;
    // input net wrenches
    // pl_net_F [ 0.05499161 -0.0303093  12.26226555]
    // pl_net_M [-0.05462257 -0.02200961 -0.00163718]       
    mavs_net_wrench.force = {0.05499161, -0.0303093,  12.26226555};
    mavs_net_wrench.torque = {-0.05462257, -0.02200961, -0.00163718};


    // robot inertia para
    // invML [[ 3.99952167e+00 -1.01852847e-05 -1.44153950e-02]
    //  [-1.01852847e-05  3.99952653e+00  7.88281800e-03]
    //  [-1.44153950e-02  7.88281800e-03  8.00119073e-01]]
    // C [[-1.22111436e-03  8.96552891e-04 -5.21153784e-03]
    //  [ 9.83579874e-04  1.35992018e-03 -7.43747245e-04]
    //  [-2.74156775e-05 -2.78317334e-05 -1.59890823e-04]]
    // D [[ 1.22111436e-03 -9.83579874e-04  2.74156775e-05]
    //  [-8.96552891e-04 -1.35992018e-03  2.78317334e-05]
    //  [ 5.21153784e-03  7.43747245e-04  1.59890823e-04]]
    // E [[-9.24693356e-02  5.78602926e-04 -3.02249039e-04]
    //  [ 5.78602926e-04 -9.33014117e-02  2.13581421e-03]
    //  [-3.02249039e-04  2.13581421e-03 -5.33502009e-05]]    
    CooperIntertPara   interaction_parameters;
    Eigen::Matrix3d C;
    C << -1.22111436e-03, 8.96552891e-04, -5.21153784e-03,
        9.83579874e-04, 1.35992018e-03, -7.43747245e-04,
        -2.74156775e-05, -2.78317334e-05, -1.59890823e-04;

    Eigen::Matrix3d D;
    D << 1.22111436e-03, -9.83579874e-04, 2.74156775e-05,
        -8.96552891e-04, -1.35992018e-03, 2.78317334e-05,
        5.21153784e-03, 7.43747245e-04, 1.59890823e-04;

    Eigen::Matrix3d E;
    E << -9.24693356e-02, 5.78602926e-04, -3.02249039e-04,
        5.78602926e-04, -9.33014117e-02, 2.13581421e-03,
        -3.02249039e-04, 2.13581421e-03, -5.33502009e-05;

    Eigen::Matrix3d m_mass_matrix;
    // assign m_mass_matrix with [[ 2.50046137e-01 -8.24240601e-06  4.50505296e-03]
//  [-8.24240601e-06  2.50034451e-01 -2.46350194e-03]
//  [ 4.50505296e-03 -2.46350194e-03  1.24991941e+00]]
    m_mass_matrix << 2.50046137e-01, -8.24240601e-06, 4.50505296e-03,
        -8.24240601e-06, 2.50034451e-01, -2.46350194e-03,
        4.50505296e-03, -2.46350194e-03, 1.24991941e+00;

    interaction_parameters.m_C = C;
    interaction_parameters.m_D = D;
    interaction_parameters.m_E = E;
    interaction_parameters.m_mass_matrix = m_mass_matrix;

    ptr_payload->InputPayloadInteractPara(interaction_parameters);

 
    ptr_payload->InputDronesNetWrenches(mavs_net_wrench);

    // call integration 
    ptr_payload->DoOneStepInt();



    // get the state after integration
    // pl_vel [ 0.00570588 -0.05373419  0.00085915]
    // pl_acc [ 4.36127361e-02 -2.35743782e-02 -1.47000610e-05]
    // pl_quat [ 0.0001164  -0.01748144  0.02370194  0.00880249]
    // pl_angularacc [-0.03773087 -0.1579497  -0.03644387]
    object_state payload_state_new = ptr_payload->state();

    // write code to obtain state'postion, velocity, acceleration, quaternion, angular acceleration from payload_state_new
    // payload_state_new is std::array<double, 13>
    // write code to obtain payload post, vel, quat, angular rate, angular acc
    std::vector<double> payload_pos(payload_state_new.begin(), payload_state_new.begin() + 3);
    std::vector<double> payload_vel(payload_state_new.begin() + 3, payload_state_new.begin() + 6);
    std::vector<double> payload_quat(payload_state_new.begin() + 6, payload_state_new.begin() + 10);
    std::vector<double> payload_angular_rate(payload_state_new.begin() + 10, payload_state_new.begin() + 13);

    // std::printf("payload_pos: %.10f, %.10f, %.10f\n", payload_pos[0], payload_pos[1], payload_pos[2]);
    // std::printf("payload_vel: %.10f, %.10f, %.10f\n", payload_vel[0], payload_vel[1], payload_vel[2]);
    // std::printf("payload_quat: %.10f, %.10f, %.10f, %.10f\n", payload_quat[0], payload_quat[1], payload_quat[2], payload_quat[3]);
    // std::printf("payload_angular_rate: %.10f, %.10f, %.10f\n", payload_angular_rate[0], payload_angular_rate[1], payload_angular_rate[2]);
    
    // payload_pos: -0.4270976585, 0.3006525364, -0.0004483574
    
    // new payload_pos: -0.4270976585, 0.3006525364, -0.0004483574

    // pl_state [-4.27147024e-01  3.01102123e-01 -4.56031150e-04
    // 5.78293027e-03  -5.37728396e-02  8.59330522e-04  
    // 9.99912428e-01  1.25673556e-02 3.87118281e-03  1.48438391e-03 
    // -3.50203897e-02  4.74107821e-02 1.68105557e-02]
    // EXPECT_FLOAT_EQ(payload_angular_acc[0], //
    //    -0.037730865525696
    //     ); 

    // check acc and angular acc
    auto payload_acc = ptr_payload->accs().linear_acc;
    auto payload_angular_acc = ptr_payload->accs().angular_acc;

    EXPECT_FLOAT_EQ(payload_angular_acc[0], //
        -0.037730865525696 
        );

    EXPECT_FLOAT_EQ(payload_angular_acc[1], //
         -0.157949661187575
        );

    EXPECT_FLOAT_EQ(payload_angular_acc[2], //
        -0.036443916622249
        );

        
    EXPECT_FLOAT_EQ(payload_acc[0], //
        0.043612733868192    
        );

    EXPECT_FLOAT_EQ(payload_acc[1], //
        -0.02357439290819
        );

    EXPECT_NEAR(payload_acc[2], //
        0.009985299106157, 1e-6  
        );



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