#ifndef PAYLOAD_SIMULATOR_H
#define PAYLOAD_SIMULATOR_H

#include <iostream>
#include <boost/numeric/odeint.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <memory>
#include "rotor_tm_sim/base/lib_rigidbody.hpp"
#include "rotor_tm_sim/lib_joint.hpp"
#include "rotor_tm_sim/lib_uav_cable.hpp"
// #include "rotor_tm_sim/lib_rigidbody.hpp"

using namespace boost::numeric::odeint;

struct CooperIntertPara{
                
                Eigen::Matrix3d m_C;

                Eigen::Matrix3d m_D;

                Eigen::Matrix3d m_E;

                Eigen::Matrix3d m_mass_matrix;
};


class Payload: public RigidBody{

    private:

    size_t num_robot_;
        
    // a vector of unique pointers
    // each pointer points to a joint
    std::vector<std::unique_ptr<Joint>> v_ptr_joints_; 
    // std::unique<Joint> v_ptr_joints_;  
    // net force applied by drones
    //    Eigen::Vector3d drones_net_force_;

    //    // net torque applied by drones
    //    Eigen::Vector3d drones_net_torque_;

    Wrench mavs_net_wrench_;

    //    Eigen::Matrix3d m_mass_matrix_;



    CooperIntertPara cooper_interact_para_;


    Eigen::MatrixXd ComputeMatrixJi(const Cable &cable, const std::unique_ptr<Joint> &ptr_joint);

    Eigen::VectorXd ComputeVectorbi(const Quadrotor &mav, const Cable &cable, const std::unique_ptr<Joint> &ptr_joint);

        // // ComputeMatrixJi is a function for updating vels after collision
        // // compute matrix Ji in Eq46
        // Eigen::MatrixXd ComputeMatrixJi(const Eigen::Vector3d &cable_direction, const Eigen::Quaterniond &payload_attitude, const Eigen::Vector3d &attach_point_body_frame);

        // // ComputeVectorbi is a function for updating vels after collision    
        // // compute vector bi in Eq46
        // // Eigen::VectorXd ComputeVectorbi(const double &drone_mass, const Eigen::Vector3d &drone_vel, const Eigen::Vector3d &cable_direction, const Eigen::Quaterniond &payload_attitude, const Eigen::Vector3d &attach_point_body_frame);
        // Eigen::VectorXd ComputeVectorbi(const double &drone_mass, const Eigen::Vector3d &drone_vel, const Eigen::Vector3d &cable_direction, const Eigen::Quaterniond &payload_attitude, const Eigen::Vector3d &attach_point_body_frame);

        // translational dynamic model
        // Eigen::Vector3d ComputeTransDynamics(const Eigen::Vector3d &drones_net_forces, const Eigen::Matrix3d &mass_matrix, const Eigen::Matrix3d &m_D,  const Eigen::Vector3d &payload_bodyrate_acc);
        Eigen::Vector3d ComputeTransDynamics();


        // rotational dynamic model
        // Eigen::Vector3d ComputeRotDynamics(const Eigen::Vector3d &drones_net_forces, const Eigen::Vector3d &drones_net_torques, const Eigen::Matrix3d &m_mass_matrix, const Eigen::Vector3d &payload_bodyrate, const Eigen::Matrix3d &m_C, const Eigen::Matrix3d &m_D, const Eigen::Matrix3d &m_E);

        Eigen::Vector3d ComputeRotDynamics();
        


        Payload() = delete;
        Payload(const Payload&) = delete;
        Payload& operator=(const Payload&) = delete;
        
        double gravity_ = 9.8;

        
        Eigen::Matrix3d matirxBodyrate2EulerRate(const double &phi, const double &theta);


        // solver ruge_kutta
        runge_kutta4<Eigen::Matrix<double, 12, 1>> stepper_;

        double current_step_ = 0;

        bool intial_acc_set_ = false;

    public:


    Payload(const MassProperty &mass_property, const double &step_size);


    Payload(const MassProperty &mass_property, std::vector<std::unique_ptr<Joint>> v_ptr_joints, const double &step_size);

    // void CalVel4AttachPoint();      

    void SetInitialAccBodyRateAcc(const Eigen::Vector3d &payload_initial_acc);


    // ComputeAttachPointsKinematics computes post, vels and accs of attach points
    void ComputeJointKinematics();

    // void UpdateVelCollided(const std::vector<std::shared_ptr<UAVCable>> v_drone_cable_ptr);
    void UpdateVelCollided();


    void InputDronesNetWrenches(const Wrench &mavs_net_wrench);


    void InputPayloadInteractPara(const CooperIntertPara &cooper_interact_para);

    // // input mass matrix  
    // void InputMassMatrix(const Eigen::Matrix3d &m_mass_matrix);

    // // input translational dynamic model inputs: drones' net force to the payload and term m_D
    // void InputDronesNetForces(const Eigen::Vector3d &drones_net_force, const Eigen::Matrix3d &m_D);

    // // input rotational dynamic model inputs: drones' net force to the payload and term m_D
    // void InputDronesNetTorques(const Eigen::Vector3d &drones_net_torque, const Eigen::Matrix3d &m_C, const Eigen::Matrix3d &m_E);    

    void ComputeAccBodyRateAcc();

    // void GetOneAttachPoint(const size_t &i, AttachPoint &attach_point) const;

  
    // void doOneStepInt();

    // virtual void DoPayloadOneStepInt() final;

    void operator() (const object_state &x , object_state &dxdt, const double time) override;
};

#endif
