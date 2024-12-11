#include <iostream> 
#include <gtest/gtest.h>
#include <memory>
#include <cmath>
#include <cstdlib>
#include <random>
#include "rotor_tm_sim/lib_payload.hpp"
#include "rotor_tm_sim/lib_uav_cable.hpp"

#include "lib4test/payload_data_reader.cpp"

struct TestData;
class MultiFileDataReader;

class rotorTMPayloadTestWithData : public ::testing::Test {
public:
    rotorTMPayloadTestWithData() {
        // Initialize the data reader with paths to your data files
        // std::string input_file = "test_data/eom_test_data_inputs_filtered.txt";
        // std::string output_file = "test_data/eom_test_data_outputs_filtered.txt";
        // std::string param_file = "test_data/eom_test_data_params_filtered.txt";
        // std::string constants_file = "test_data/eom_test_data_constants_filtered.txt";


        std::string input_file = std::string(TEST_DATA_DIR) + "/eom_test_data_inputs_filtered.txt";
        std::string output_file = std::string(TEST_DATA_DIR) + "/eom_test_data_outputs_filtered.txt";
        std::string param_file = std::string(TEST_DATA_DIR) + "/eom_test_data_params_filtered.txt";
        std::string constants_file = std::string(TEST_DATA_DIR) + "/eom_test_data_constants_filtered.txt";

        data_reader = std::make_unique<MultiFileDataReader>(input_file, output_file, param_file, constants_file);
      
        // data_reader = std::make_unique<MultiFileDataReader>(INPUT_FILE_PATH, OUTPUT_FILE_PATH, PARAM_FILE_PATH, CONSTANTS_FILE_PATH);


        // Get constants using the public getter
        auto constants = data_reader->getConstants();

        // Extract payload mass and inertia
        double payload_mass = constants.mass;
        Eigen::Matrix3d payload_inertia = constants.I;

        // Initialize Payload
        double dt = 0.01;
        MassProperty payload_mass_property = {payload_mass, payload_inertia};
        ptr_payload = std::make_shared<Payload>(payload_mass_property, dt);
    }

    ~rotorTMPayloadTestWithData() {}

protected:
    std::shared_ptr<Payload> ptr_payload;
    std::unique_ptr<MultiFileDataReader> data_reader;
};


TEST_F(rotorTMPayloadTestWithData, checkInstanceClass) {
    ASSERT_TRUE(ptr_payload != nullptr);
}



TEST_F(rotorTMPayloadTestWithData, checkDynamicsWithPythonData) {

    // define index or call number 
    int index = 100;

    // Read test case data
    auto test_data = data_reader->readAllData(index);  // Read first test case

    // Set initial state
    object_state payload_state_before;
    // Convert from Eigen::VectorXd to array
    for(int i = 0; i < 13; i++) {
        payload_state_before[i] = test_data.state[i];
    }

    ptr_payload->SetPayloadStates(payload_state_before);
    
    // Set wrench using Eigen::Vector3d
    Wrench mavs_net_wrench;
    mavs_net_wrench.force = test_data.force;  // Directly assign Eigen::Vector3d
    mavs_net_wrench.torque = test_data.torque;  // Directly assign Eigen::Vector3d
    
    // Set parameters
    CooperIntertPara interaction_parameters;
    interaction_parameters.m_C = test_data.C;
    interaction_parameters.m_D = test_data.D;
    interaction_parameters.m_E = test_data.E;
    interaction_parameters.m_mass_matrix = test_data.ML;
    
    ptr_payload->InputPayloadInteractPara(interaction_parameters);
    ptr_payload->InputDronesNetWrenches(mavs_net_wrench);
    
    ptr_payload->DoOneStepInt();

    // Get accelerations compuated by class
    // std cout test_data.state (Eigen::VectorXd state;) and input force and torque
    // std::cout<<"test_data.state: "<<test_data.state<<std::endl;
    // std::cout<<"test_data.force: "<<test_data.force<<std::endl;
    // std::cout<<"test_data.torque: "<<test_data.torque<<std::endl;
    // // court ML
    // std::cout<<"test_data.ML: "<<test_data.ML<<std::endl;
    // std::cout<<"test_data.sdot: "<<test_data.sdot<<std::endl;
    // // cout invML, C, D and E
    // std::cout<<"test_data.invML: "<<test_data.invML<<std::endl;
    // std::cout<<"test_data.C: "<<test_data.C<<std::endl;
    // std::cout<<"test_data.D: "<<test_data.D<<std::endl;
    // std::cout<<"test_data.E: "<<test_data.E<<std::endl;
    // // cout test_data mass inertia
    // std::cout<<"test_data.mass: "<<test_data.mass<<std::endl;
    // std::cout<<"test_data.I: "<<test_data.I<<std::endl;
    // // gravity
    // std::cout<<"test_data.grav: "<<test_data.grav<<std::endl;
    


    // check mass of payload
    EXPECT_DOUBLE_EQ(test_data.mass, ptr_payload->mass());

    EXPECT_DOUBLE_EQ(test_data.grav, Utils::gravity);
    
    //write to compare inertia matrix
    EXPECT_DOUBLE_EQ(test_data.I(0,0), ptr_payload->inertia()(0,0));
    EXPECT_DOUBLE_EQ(test_data.I(0,1), ptr_payload->inertia()(0,1));
    EXPECT_DOUBLE_EQ(test_data.I(0,2), ptr_payload->inertia()(0,2));
    EXPECT_DOUBLE_EQ(test_data.I(1,0), ptr_payload->inertia()(1,0));
    EXPECT_DOUBLE_EQ(test_data.I(1,1), ptr_payload->inertia()(1,1));
    EXPECT_DOUBLE_EQ(test_data.I(1,2), ptr_payload->inertia()(1,2));
    EXPECT_DOUBLE_EQ(test_data.I(2,0), ptr_payload->inertia()(2,0));
    EXPECT_DOUBLE_EQ(test_data.I(2,1), ptr_payload->inertia()(2,1));
    EXPECT_DOUBLE_EQ(test_data.I(2,2), ptr_payload->inertia()(2,2));

    // check input force
    EXPECT_DOUBLE_EQ(test_data.force[0], ptr_payload->mavs_net_wrench().force[0]);
    EXPECT_DOUBLE_EQ(test_data.force[1], ptr_payload->mavs_net_wrench().force[1]);
    EXPECT_DOUBLE_EQ(test_data.force[2], ptr_payload->mavs_net_wrench().force[2]);

    // check input torque
    EXPECT_DOUBLE_EQ(test_data.torque[0], ptr_payload->mavs_net_wrench().torque[0]);
    EXPECT_DOUBLE_EQ(test_data.torque[1], ptr_payload->mavs_net_wrench().torque[1]);
    EXPECT_DOUBLE_EQ(test_data.torque[2], ptr_payload->mavs_net_wrench().torque[2]);

    // check cooper_interact_para()
    // check C
    EXPECT_DOUBLE_EQ(test_data.C(0,0), ptr_payload->cooper_interact_para().m_C(0,0));
    EXPECT_DOUBLE_EQ(test_data.C(0,1), ptr_payload->cooper_interact_para().m_C(0,1));
    EXPECT_DOUBLE_EQ(test_data.C(0,2), ptr_payload->cooper_interact_para().m_C(0,2));
    EXPECT_DOUBLE_EQ(test_data.C(1,0), ptr_payload->cooper_interact_para().m_C(1,0));
    EXPECT_DOUBLE_EQ(test_data.C(1,1), ptr_payload->cooper_interact_para().m_C(1,1));
    EXPECT_DOUBLE_EQ(test_data.C(1,2), ptr_payload->cooper_interact_para().m_C(1,2));
    EXPECT_DOUBLE_EQ(test_data.C(2,0), ptr_payload->cooper_interact_para().m_C(2,0));
    EXPECT_DOUBLE_EQ(test_data.C(2,1), ptr_payload->cooper_interact_para().m_C(2,1));
    EXPECT_DOUBLE_EQ(test_data.C(2,2), ptr_payload->cooper_interact_para().m_C(2,2));

    // check D
    EXPECT_DOUBLE_EQ(test_data.D(0,0), ptr_payload->cooper_interact_para().m_D(0,0));
    EXPECT_DOUBLE_EQ(test_data.D(0,1), ptr_payload->cooper_interact_para().m_D(0,1));
    EXPECT_DOUBLE_EQ(test_data.D(0,2), ptr_payload->cooper_interact_para().m_D(0,2));
    EXPECT_DOUBLE_EQ(test_data.D(1,0), ptr_payload->cooper_interact_para().m_D(1,0));
    EXPECT_DOUBLE_EQ(test_data.D(1,1), ptr_payload->cooper_interact_para().m_D(1,1));
    EXPECT_DOUBLE_EQ(test_data.D(1,2), ptr_payload->cooper_interact_para().m_D(1,2));
    EXPECT_DOUBLE_EQ(test_data.D(2,0), ptr_payload->cooper_interact_para().m_D(2,0));
    EXPECT_DOUBLE_EQ(test_data.D(2,1), ptr_payload->cooper_interact_para().m_D(2,1));
    EXPECT_DOUBLE_EQ(test_data.D(2,2), ptr_payload->cooper_interact_para().m_D(2,2));

    // check E
    EXPECT_DOUBLE_EQ(test_data.E(0,0), ptr_payload->cooper_interact_para().m_E(0,0));
    EXPECT_DOUBLE_EQ(test_data.E(0,1), ptr_payload->cooper_interact_para().m_E(0,1));
    EXPECT_DOUBLE_EQ(test_data.E(0,2), ptr_payload->cooper_interact_para().m_E(0,2));
    EXPECT_DOUBLE_EQ(test_data.E(1,0), ptr_payload->cooper_interact_para().m_E(1,0));
    EXPECT_DOUBLE_EQ(test_data.E(1,1), ptr_payload->cooper_interact_para().m_E(1,1));
    EXPECT_DOUBLE_EQ(test_data.E(1,2), ptr_payload->cooper_interact_para().m_E(1,2));
    EXPECT_DOUBLE_EQ(test_data.E(2,0), ptr_payload->cooper_interact_para().m_E(2,0));
    EXPECT_DOUBLE_EQ(test_data.E(2,1), ptr_payload->cooper_interact_para().m_E(2,1));
    EXPECT_DOUBLE_EQ(test_data.E(2,2), ptr_payload->cooper_interact_para().m_E(2,2));

    // check mass_matrix
    EXPECT_DOUBLE_EQ(test_data.ML(0,0), ptr_payload->cooper_interact_para().m_mass_matrix(0,0));
    EXPECT_DOUBLE_EQ(test_data.ML(0,1), ptr_payload->cooper_interact_para().m_mass_matrix(0,1));
    EXPECT_DOUBLE_EQ(test_data.ML(0,2), ptr_payload->cooper_interact_para().m_mass_matrix(0,2));
    EXPECT_DOUBLE_EQ(test_data.ML(1,0), ptr_payload->cooper_interact_para().m_mass_matrix(1,0));
    EXPECT_DOUBLE_EQ(test_data.ML(1,1), ptr_payload->cooper_interact_para().m_mass_matrix(1,1));
    EXPECT_DOUBLE_EQ(test_data.ML(1,2), ptr_payload->cooper_interact_para().m_mass_matrix(1,2));
    EXPECT_DOUBLE_EQ(test_data.ML(2,0), ptr_payload->cooper_interact_para().m_mass_matrix(2,0));
    EXPECT_DOUBLE_EQ(test_data.ML(2,1), ptr_payload->cooper_interact_para().m_mass_matrix(2,1));
    EXPECT_DOUBLE_EQ(test_data.ML(2,2), ptr_payload->cooper_interact_para().m_mass_matrix(2,2));


    //obtain computed payload acc and angular acc
    auto payload_acc = ptr_payload->accs().linear_acc;
    auto payload_angular_acc = ptr_payload->accs().angular_acc;
    
    // obtain accs from test_data   
    auto test_data_acc = test_data.sdot.segment<3>(3);
    auto test_data_angular_acc = test_data.sdot.tail<3>();

    // cout payload_acc and test_data_acc
    std::cout<<"payload_acc: "<<payload_acc.transpose()<<std::endl;
    std::cout<<"test_data_acc: "<<test_data_acc.transpose()<<std::endl;
    std::cout<<"payload_angular_acc: "<<payload_angular_acc.transpose()<<std::endl;
    // cout test payload angular acc
    std::cout<<"test_data_angular_acc: "<<test_data_angular_acc.transpose()<<std::endl;
    
    // check accs to compare payload_acc and test_data_acc
    EXPECT_NEAR(payload_acc[0], test_data_acc[0], 1e-14);
    EXPECT_NEAR(payload_acc[1], test_data_acc[1], 1e-14);
    EXPECT_NEAR(payload_acc[2], test_data_acc[2], 1e-14);

    // check angular accs to compare payload_angular_acc and test_data_angular_acc
    EXPECT_NEAR(payload_angular_acc[0], test_data_angular_acc[0], 1e-14);
    EXPECT_NEAR(payload_angular_acc[1], test_data_angular_acc[1], 1e-14);
    EXPECT_NEAR(payload_angular_acc[2], test_data_angular_acc[2], 1e-14);

}



TEST_F(rotorTMPayloadTestWithData, checkDynamicsWithPythonDataSet) {

    // Get total number of test cases
    // each line of text corresponds to a test case
    int total_test_cases = data_reader->getTotalInputLines();
    std::cout << "Total test cases: " << total_test_cases << std::endl;

    // define index or call number 
    for (int index =1; index <total_test_cases; index++) 
    {
            std::cout<<"run data set with index: "<<index<<std::endl;
            
            // Read test case data
            auto test_data = data_reader->readAllData(index);  // Read first test case

            // Step 1. Input from test data set test_data

            // payload sates
            object_state payload_state_before;
            // Convert from Eigen::VectorXd to array
            for(int i = 0; i < 13; i++) {
                payload_state_before[i] = test_data.state[i];
            }

            ptr_payload->SetPayloadStates(payload_state_before);
            
            // net wrenches applied to payload
            Wrench mavs_net_wrench;
            mavs_net_wrench.force = test_data.force;  // Directly assign Eigen::Vector3d
            mavs_net_wrench.torque = test_data.torque;  // Directly assign Eigen::Vector3d
            
            // interaction matrix C, D, E and mass matrix ML
            CooperIntertPara interaction_parameters;
            interaction_parameters.m_C = test_data.C;
            interaction_parameters.m_D = test_data.D;
            interaction_parameters.m_E = test_data.E;
            interaction_parameters.m_mass_matrix = test_data.ML;
            
            ptr_payload->InputPayloadInteractPara(interaction_parameters);

            // Step 2 compute interaction forces and torques
            ptr_payload->InputDronesNetWrenches(mavs_net_wrench);


            // Step 3 call integration 
            ptr_payload->DoOneStepInt();

            // Step 4 compare outputs from CXX and test data set

            // compare contants
            // mass and gravity
            EXPECT_DOUBLE_EQ(test_data.mass, ptr_payload->mass());

            EXPECT_DOUBLE_EQ(test_data.grav, Utils::gravity);
            
            // inertia matrix
            EXPECT_DOUBLE_EQ(test_data.I(0,0), ptr_payload->inertia()(0,0));
            EXPECT_DOUBLE_EQ(test_data.I(0,1), ptr_payload->inertia()(0,1));
            EXPECT_DOUBLE_EQ(test_data.I(0,2), ptr_payload->inertia()(0,2));
            EXPECT_DOUBLE_EQ(test_data.I(1,0), ptr_payload->inertia()(1,0));
            EXPECT_DOUBLE_EQ(test_data.I(1,1), ptr_payload->inertia()(1,1));
            EXPECT_DOUBLE_EQ(test_data.I(1,2), ptr_payload->inertia()(1,2));
            EXPECT_DOUBLE_EQ(test_data.I(2,0), ptr_payload->inertia()(2,0));
            EXPECT_DOUBLE_EQ(test_data.I(2,1), ptr_payload->inertia()(2,1));
            EXPECT_DOUBLE_EQ(test_data.I(2,2), ptr_payload->inertia()(2,2));

            // check input force
            EXPECT_DOUBLE_EQ(test_data.force[0], ptr_payload->mavs_net_wrench().force[0]);
            EXPECT_DOUBLE_EQ(test_data.force[1], ptr_payload->mavs_net_wrench().force[1]);
            EXPECT_DOUBLE_EQ(test_data.force[2], ptr_payload->mavs_net_wrench().force[2]);

            // check input torque
            EXPECT_DOUBLE_EQ(test_data.torque[0], ptr_payload->mavs_net_wrench().torque[0]);
            EXPECT_DOUBLE_EQ(test_data.torque[1], ptr_payload->mavs_net_wrench().torque[1]);
            EXPECT_DOUBLE_EQ(test_data.torque[2], ptr_payload->mavs_net_wrench().torque[2]);

            // check cooper_interact_para()
            // check C
            EXPECT_DOUBLE_EQ(test_data.C(0,0), ptr_payload->cooper_interact_para().m_C(0,0));
            EXPECT_DOUBLE_EQ(test_data.C(0,1), ptr_payload->cooper_interact_para().m_C(0,1));
            EXPECT_DOUBLE_EQ(test_data.C(0,2), ptr_payload->cooper_interact_para().m_C(0,2));
            EXPECT_DOUBLE_EQ(test_data.C(1,0), ptr_payload->cooper_interact_para().m_C(1,0));
            EXPECT_DOUBLE_EQ(test_data.C(1,1), ptr_payload->cooper_interact_para().m_C(1,1));
            EXPECT_DOUBLE_EQ(test_data.C(1,2), ptr_payload->cooper_interact_para().m_C(1,2));
            EXPECT_DOUBLE_EQ(test_data.C(2,0), ptr_payload->cooper_interact_para().m_C(2,0));
            EXPECT_DOUBLE_EQ(test_data.C(2,1), ptr_payload->cooper_interact_para().m_C(2,1));
            EXPECT_DOUBLE_EQ(test_data.C(2,2), ptr_payload->cooper_interact_para().m_C(2,2));

            // check D
            EXPECT_DOUBLE_EQ(test_data.D(0,0), ptr_payload->cooper_interact_para().m_D(0,0));
            EXPECT_DOUBLE_EQ(test_data.D(0,1), ptr_payload->cooper_interact_para().m_D(0,1));
            EXPECT_DOUBLE_EQ(test_data.D(0,2), ptr_payload->cooper_interact_para().m_D(0,2));
            EXPECT_DOUBLE_EQ(test_data.D(1,0), ptr_payload->cooper_interact_para().m_D(1,0));
            EXPECT_DOUBLE_EQ(test_data.D(1,1), ptr_payload->cooper_interact_para().m_D(1,1));
            EXPECT_DOUBLE_EQ(test_data.D(1,2), ptr_payload->cooper_interact_para().m_D(1,2));
            EXPECT_DOUBLE_EQ(test_data.D(2,0), ptr_payload->cooper_interact_para().m_D(2,0));
            EXPECT_DOUBLE_EQ(test_data.D(2,1), ptr_payload->cooper_interact_para().m_D(2,1));
            EXPECT_DOUBLE_EQ(test_data.D(2,2), ptr_payload->cooper_interact_para().m_D(2,2));

            // check E
            EXPECT_DOUBLE_EQ(test_data.E(0,0), ptr_payload->cooper_interact_para().m_E(0,0));
            EXPECT_DOUBLE_EQ(test_data.E(0,1), ptr_payload->cooper_interact_para().m_E(0,1));
            EXPECT_DOUBLE_EQ(test_data.E(0,2), ptr_payload->cooper_interact_para().m_E(0,2));
            EXPECT_DOUBLE_EQ(test_data.E(1,0), ptr_payload->cooper_interact_para().m_E(1,0));
            EXPECT_DOUBLE_EQ(test_data.E(1,1), ptr_payload->cooper_interact_para().m_E(1,1));
            EXPECT_DOUBLE_EQ(test_data.E(1,2), ptr_payload->cooper_interact_para().m_E(1,2));
            EXPECT_DOUBLE_EQ(test_data.E(2,0), ptr_payload->cooper_interact_para().m_E(2,0));
            EXPECT_DOUBLE_EQ(test_data.E(2,1), ptr_payload->cooper_interact_para().m_E(2,1));
            EXPECT_DOUBLE_EQ(test_data.E(2,2), ptr_payload->cooper_interact_para().m_E(2,2));

            // check mass_matrix
            EXPECT_DOUBLE_EQ(test_data.ML(0,0), ptr_payload->cooper_interact_para().m_mass_matrix(0,0));
            EXPECT_DOUBLE_EQ(test_data.ML(0,1), ptr_payload->cooper_interact_para().m_mass_matrix(0,1));
            EXPECT_DOUBLE_EQ(test_data.ML(0,2), ptr_payload->cooper_interact_para().m_mass_matrix(0,2));
            EXPECT_DOUBLE_EQ(test_data.ML(1,0), ptr_payload->cooper_interact_para().m_mass_matrix(1,0));
            EXPECT_DOUBLE_EQ(test_data.ML(1,1), ptr_payload->cooper_interact_para().m_mass_matrix(1,1));
            EXPECT_DOUBLE_EQ(test_data.ML(1,2), ptr_payload->cooper_interact_para().m_mass_matrix(1,2));
            EXPECT_DOUBLE_EQ(test_data.ML(2,0), ptr_payload->cooper_interact_para().m_mass_matrix(2,0));
            EXPECT_DOUBLE_EQ(test_data.ML(2,1), ptr_payload->cooper_interact_para().m_mass_matrix(2,1));
            EXPECT_DOUBLE_EQ(test_data.ML(2,2), ptr_payload->cooper_interact_para().m_mass_matrix(2,2));


            //obtain computed payload acc and angular acc
            auto payload_acc = ptr_payload->accs().linear_acc;
            auto payload_angular_acc = ptr_payload->accs().angular_acc;
            
            // obtain accs from test_data   
            auto test_data_acc = test_data.sdot.segment<3>(3);
            auto test_data_angular_acc = test_data.sdot.tail<3>();

            // cout payload_acc and test_data_acc

            // check accs to compare payload_acc and test_data_acc
            EXPECT_NEAR(payload_acc[0], test_data_acc[0], 1e-13);
            EXPECT_NEAR(payload_acc[1], test_data_acc[1], 1e-13);
            EXPECT_NEAR(payload_acc[2], test_data_acc[2], 1e-13);

            // check angular accs to compare payload_angular_acc and test_data_angular_acc
            EXPECT_NEAR(payload_angular_acc[0], test_data_angular_acc[0], 1e-13);
            EXPECT_NEAR(payload_angular_acc[1], test_data_angular_acc[1], 1e-13);
            EXPECT_NEAR(payload_angular_acc[2], test_data_angular_acc[2], 1e-13);

        }
}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
