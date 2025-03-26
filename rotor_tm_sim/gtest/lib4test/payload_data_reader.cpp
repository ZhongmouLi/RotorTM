#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <Eigen/Dense>
#include <iomanip>
#include <vector>

class MultiFileDataReader {
public:
    struct TestData {
        Eigen::VectorXd state = Eigen::VectorXd(13);
        Eigen::Vector3d force = Eigen::Vector3d::Zero();
        Eigen::Vector3d torque = Eigen::Vector3d::Zero();
        Eigen::Matrix3d ML = Eigen::Matrix3d::Zero();
        Eigen::VectorXd sdot = Eigen::VectorXd(13);
        Eigen::Matrix3d invML = Eigen::Matrix3d::Zero();
        Eigen::Matrix3d C = Eigen::Matrix3d::Zero();
        Eigen::Matrix3d D = Eigen::Matrix3d::Zero();
        Eigen::Matrix3d E = Eigen::Matrix3d::Zero();
        double mass = 0.0;
        double grav = 0.0;
        Eigen::Matrix3d I = Eigen::Matrix3d::Zero();
        Eigen::Matrix3d invI = Eigen::Matrix3d::Zero();
    };

    MultiFileDataReader(const std::string& input_file_path, 
                        const std::string& output_file_path, 
                        const std::string& param_file_path, 
                        const std::string& constants_file_path)
        : input_file_path_(input_file_path),
          output_file_path_(output_file_path),
          param_file_path_(param_file_path),
          constants_file_path_(constants_file_path),
          constants_data_() {

        // Print test data addresses
        // std::cout<<"input_file_path_"<<input_file_path_<<std::endl;
        // std::cout<<"output_file_path_"<<output_file_path_<<std::endl;
        // std::cout<<"param_file_path_"<<param_file_path_<<std::endl;
        // std::cout<<"constants_file_path_"<<constants_file_path_<<std::endl;

        // Load constants once
        readConstants(constants_data_.mass, constants_data_.grav, constants_data_.I, constants_data_.invI);

        // Validate line counts
        validateLineCounts();
    }

    TestData readAllData(int index) {
        TestData data = constants_data_; // Start with constants

        readInputData(index, data.state, data.force, data.torque, data.ML);
        readOutputData(index, data.sdot);
        readParamData(index, data.invML, data.C, data.D, data.E);
        return data;
    }

    TestData getConstants() const {
        return constants_data_;
    }


    int getTotalInputLines() const {
        return getTotalLines(input_file_path_);
    }
private:
    std::string input_file_path_;
    std::string output_file_path_;
    std::string param_file_path_;
    std::string constants_file_path_;

    TestData constants_data_;

    int total_lines_ = 0;


    void validateLineCounts() const {
        int input_lines = getTotalLines(input_file_path_);
        int output_lines = getTotalLines(output_file_path_);
        int param_lines = getTotalLines(param_file_path_);

        if (input_lines != output_lines || input_lines != param_lines) {
            throw std::runtime_error("Line count mismatch: "
                                     "Input file has " + std::to_string(input_lines) + " lines, "
                                     "Output file has " + std::to_string(output_lines) + " lines, "
                                     "Param file has " + std::to_string(param_lines) + " lines.");
        }
    }

    int getTotalLines(const std::string& file_path) const {
        std::ifstream file(file_path);
        if (!file.is_open()) {
            throw std::runtime_error("Cannot open file: " + file_path);
        }

        std::string line;
        int line_count = 0;
        while (std::getline(file, line)) {
            ++line_count;
        }
        return line_count - 1; // Subtract 1 to account for the header
    }


    void readInputData(int index, Eigen::VectorXd& state, Eigen::Vector3d& force, Eigen::Vector3d& torque, Eigen::Matrix3d& ML) {
        std::ifstream file(input_file_path_);
        if (!file.is_open()) {
            throw std::runtime_error("Cannot open input file: " + input_file_path_);
        }

        std::string line;
        if (!std::getline(file, line)) {
            throw std::runtime_error("Input file is empty or missing header");
        }

        while (std::getline(file, line)) {
            std::istringstream iss(line);
            std::string value;

            if (!std::getline(iss, value, ',')) continue;
            int line_idx = std::stoi(value);

            if (line_idx == index) {
                for (int i = 0; i < 13; ++i) {
                    if (!std::getline(iss, value, ',')) {
                        throw std::runtime_error("Failed to read state value");
                    }
                    state[i] = std::stod(value);
                }

                for (int i = 0; i < 3; ++i) {
                    if (!std::getline(iss, value, ',')) {
                        throw std::runtime_error("Failed to read force value");
                    }
                    force[i] = std::stod(value);
                }

                for (int i = 0; i < 3; ++i) {
                    if (!std::getline(iss, value, ',')) {
                        throw std::runtime_error("Failed to read torque value");
                    }
                    torque[i] = std::stod(value);
                }

                for (int i = 0; i < 3; ++i) {
                    for (int j = 0; j < 3; ++j) {
                        if (!std::getline(iss, value, ',')) {
                            throw std::runtime_error("Failed to read ML value");
                        }
                        ML(i, j) = std::stod(value);
                    }
                }
                return;
            }
        }

        throw std::runtime_error("Index not found in input file");
    }

    void readOutputData(int index, Eigen::VectorXd& sdot) {
        std::ifstream file(output_file_path_);
        if (!file.is_open()) {
            throw std::runtime_error("Cannot open output file: " + output_file_path_);
        }

        std::string line;
        if (!std::getline(file, line)) {
            throw std::runtime_error("Output file is empty or missing header");
        }

        while (std::getline(file, line)) {
            std::istringstream iss(line);
            std::string value;

            if (!std::getline(iss, value, ',')) continue;
            int line_idx = std::stoi(value);

            if (line_idx == index) {
                for (int i = 0; i < sdot.size(); ++i) {
                    if (!std::getline(iss, value, ',')) {
                        throw std::runtime_error("Failed to read sdot value");
                    }
                    sdot[i] = std::stod(value);
                }
                return;
            }
        }

        throw std::runtime_error("Index not found in output file");
    }

    void readParamData(int index, Eigen::Matrix3d& invML, Eigen::Matrix3d& C, Eigen::Matrix3d& D, Eigen::Matrix3d& E) {
        std::ifstream file(param_file_path_);
        if (!file.is_open()) {
            throw std::runtime_error("Cannot open param file: " + param_file_path_);
        }

        std::string line;
        if (!std::getline(file, line)) {
            throw std::runtime_error("Param file is empty or missing header");
        }

        while (std::getline(file, line)) {
            std::istringstream iss(line);
            std::string value;

            if (!std::getline(iss, value, ',')) continue;
            int line_idx = std::stoi(value);

            if (line_idx == index) {
                for (int i = 0; i < 3; ++i) {
                    for (int j = 0; j < 3; ++j) {
                        if (!std::getline(iss, value, ',')) {
                            throw std::runtime_error("Failed to read invML value");
                        }
                        invML(i, j) = std::stod(value);
                    }
                }

                for (int i = 0; i < 3; ++i) {
                    for (int j = 0; j < 3; ++j) {
                        if (!std::getline(iss, value, ',')) {
                            throw std::runtime_error("Failed to read C value");
                        }
                        C(i, j) = std::stod(value);
                    }
                }

                for (int i = 0; i < 3; ++i) {
                    for (int j = 0; j < 3; ++j) {
                        if (!std::getline(iss, value, ',')) {
                            throw std::runtime_error("Failed to read D value");
                        }
                        D(i, j) = std::stod(value);
                    }
                }

                for (int i = 0; i < 3; ++i) {
                    for (int j = 0; j < 3; ++j) {
                        if (!std::getline(iss, value, ',')) {
                            throw std::runtime_error("Failed to read E value");
                        }
                        E(i, j) = std::stod(value);
                    }
                }
                return;
            }
        }

        throw std::runtime_error("Index not found in param file");
    }

    void readConstants(double& mass, double& grav, Eigen::Matrix3d& I, Eigen::Matrix3d& invI) {
        std::ifstream file(constants_file_path_);
        if (!file.is_open()) {
            throw std::runtime_error("Cannot open constants file: " + constants_file_path_);
        }

        std::string line;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            std::string key;

            // Extract the key and handle cases with trailing commas
            if (!std::getline(iss, key, ',')) continue;

            if (key == "mass") {
                if (!(iss >> mass)) {
                    throw std::runtime_error("Failed to read mass");
                }
                // std::cout << "Parsed mass: " << mass << std::endl;
            } else if (key == "grav") {
                if (!(iss >> grav)) {
                    throw std::runtime_error("Failed to read grav");
                }
                // std::cout << "Parsed grav: " << grav << std::endl;
            } else if (key == "I") {
                if (!std::getline(file, line)) {
                    throw std::runtime_error("Failed to read I matrix");
                }
                parseMatrixFromLine(line, I);
                // std::cout << "Parsed I matrix:\n" << I << std::endl;
            } else if (key == "invI") {
                if (!std::getline(file, line)) {
                    throw std::runtime_error("Failed to read invI matrix");
                }
                parseMatrixFromLine(line, invI);
                // std::cout << "Parsed invI matrix:\n" << invI << std::endl;
            }
        }
    }

    void parseMatrixFromLine(const std::string& line, Eigen::Matrix3d& matrix) {
        std::istringstream iss(line);
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                if (!(iss >> matrix(i, j))) {
                    throw std::runtime_error("Failed to parse matrix element (" + std::to_string(i) + "," + std::to_string(j) + ")");
                }
                if (iss.peek() == ',') iss.ignore(); // Skip commas
            }
        }
    }
};
