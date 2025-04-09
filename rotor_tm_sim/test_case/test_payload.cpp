#include "rotor_tm_sim/lib_payload.hpp"
#include "rotor_tm_sim/lib_uav_cable.hpp"
#include <cmath>
#include <cstdlib>
#include <gtest/gtest.h>
#include <iostream>
#include <memory>
#include <random>

#include "lib4test/payload_data_reader.cpp"
// libs to print logs
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <Eigen/Dense>
#include <sstream>
#include <iomanip>
struct TestData;
class MultiFileDataReader;



int main(int argc, char **argv) {


    std::string input_file =
        std::string(TEST_DATA_DIR) + "eom_test_data_inputs_filtered.txt";
    std::string output_file =
        std::string(TEST_DATA_DIR) + "eom_test_data_outputs_filtered.txt";
    std::string param_file =
        std::string(TEST_DATA_DIR) + "eom_test_data_params_filtered.txt";
    std::string constants_file =
        std::string(TEST_DATA_DIR) + "eom_test_data_constants_filtered.txt";

    std::unique_ptr<MultiFileDataReader> data_reader = std::make_unique<MultiFileDataReader>(
        input_file, output_file, param_file, constants_file);

    // data_reader = std::make_unique<MultiFileDataReader>(INPUT_FILE_PATH,
    // OUTPUT_FILE_PATH, PARAM_FILE_PATH, CONSTANTS_FILE_PATH);

    // Get constants using the public getter
    auto constants = data_reader->getConstants();

    // Extract payload mass and inertia
    double payload_mass = constants.mass;
    Eigen::Matrix3d payload_inertia = constants.I;

    // Initialize Payload
    double dt = 0.01;
    MassProperty payload_mass_property = {payload_mass, payload_inertia};
    std::shared_ptr<Payload>  ptr_payload = std::make_shared<Payload>(payload_mass_property, dt);

     // define index or call number
    const int index = 100;

    // create log file
    std::string log_file = std::string(TEST_DATA_DIR) + std::to_string(index) + "-3-result.txt";
    ptr_payload->EnableLogging(log_file);

    // Read test case data
    auto test_data = data_reader->readAllData(index); // Read first test case

    // Set initial state
    object_state payload_state_current;
    // Convert from Eigen::VectorXd to array
    for (int i = 0; i < 13; i++) {
        payload_state_current[i] = test_data.state[i];
    }

    ptr_payload->SetPayloadStates(payload_state_current);
    // logger->info("Py_current_state = {}", Utils::arrayToString(payload_state_current, 17));
    // logger->info("Cxx_current_state = {}", Utils::arrayToString(ptr_payload->state(), 17));

    ptr_payload->get_spdlog_logger()->info("Py_current_state = {}", Utils::arrayToString(payload_state_current, 17));
    // ptr_payload->get_spdlog_logger()->info("Cxx_current_state = {}", Utils::arrayToString(ptr_payload->state(), 17));
    ptr_payload->get_spdlog_logger()->info("Py_differenate_current_state = {}", Utils::EigenVectorToString(test_data.sdot, 17));


    // Set wrench using Eigen::Vector3d
    Wrench mavs_net_wrench;
    mavs_net_wrench.force = test_data.force;   // Directly assign Eigen::Vector3d
    mavs_net_wrench.torque = test_data.torque; // Directly assign Eigen::Vector3d

    ptr_payload->InputDronesNetWrenches(mavs_net_wrench);

    // Set parameters
    CooperIntertPara interaction_parameters;
    interaction_parameters.m_C = test_data.C;
    interaction_parameters.m_D = test_data.D;
    interaction_parameters.m_E = test_data.E;
    interaction_parameters.m_mass_matrix = test_data.ML;


    ptr_payload->InputPayloadInteractPara(interaction_parameters);

    // get acc and angular acc from the last step
    constexpr int index_previous = index - 1;
    auto test_data_previous_step = data_reader->readAllData(index_previous);

    auto test_data_acc_previous = test_data_previous_step.sdot.segment<3>(3);
    auto test_data_angular_acc_previous = test_data_previous_step.sdot.tail<3>();
    ptr_payload->SetLinearAcc(test_data_acc_previous);
    ptr_payload->SetAngularAcc(test_data_angular_acc_previous);




    // compute dynamics
    ptr_payload->ComputeDynamics();

    // int 
    ptr_payload->DoOneStepInt();
}
