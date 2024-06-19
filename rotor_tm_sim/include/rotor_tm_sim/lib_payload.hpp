#ifndef PAYLOAD_SIMULATOR_H
#define PAYLOAD_SIMULATOR_H

#include <iostream>
#include <boost/numeric/odeint.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include "lib_rigidbody.hpp"
#include "lib_uav_cable.hpp"


using namespace boost::numeric::odeint;

class Payload: public RigidBody{

    private:

    struct AttachPoint: public Pose, public Kinematics
    {
        Eigen::Vector3d post_bodyframe;
    };

    std::vector<AttachPoint> v_attach_points_; 

    size_t num_robot_;
        
        // vector of attach point positions in body frame
    //     // it is fixed during initilisation
    //    std::vector<Eigen::Vector3d> v_attach_points_posts_body_frame_; 

    //    // vector of attach point positions in world frame
    //    // a vector of 3d vectors and each of them represent an attach point's post
    //    std::vector<Eigen::Vector3d> v_attach_points_posts_;

    //    // vector of attach point vels     
    //    std::vector<Eigen::Vector3d> v_attach_points_vels_; 

    //     // vector of attach point accs     
    //    std::vector<Eigen::Vector3d> v_attach_points_accs_; 
       std::vector<AttachPoint> v_attach_points_; 

       // net force applied by drones
    //    Eigen::Vector3d drones_net_force_;

    //    // net torque applied by drones
    //    Eigen::Vector3d drones_net_torque_;

       Wrench mavs_net_wrench_;

    //    Eigen::Matrix3d m_mass_matrix_;

       struct CooperIntertPara{
                
                Eigen::Matrix3d m_C;

                Eigen::Matrix3d m_D;

                Eigen::Matrix3d m_E;

                Eigen::Matrix3d m_mass_matrix;
       };

       CooperIntertPara cooper_interact_para_;

    //    Eigen::Matrix3d m_C_;

    //    Eigen::Matrix3d m_D_;

    //    Eigen::Matrix3d m_E_;


        // // ComputeMatrixJi is a function for updating vels after collision
        // // compute matrix Ji in Eq46
        // Eigen::MatrixXd ComputeMatrixJi(const Eigen::Vector3d &cable_direction, const Eigen::Quaterniond &payload_attitude, const Eigen::Vector3d &attach_point_body_frame);

        // // ComputeVectorbi is a function for updating vels after collision    
        // // compute vector bi in Eq46
        // // Eigen::VectorXd ComputeVectorbi(const double &drone_mass, const Eigen::Vector3d &drone_vel, const Eigen::Vector3d &cable_direction, const Eigen::Quaterniond &payload_attitude, const Eigen::Vector3d &attach_point_body_frame);
        // Eigen::VectorXd ComputeVectorbi(const double &drone_mass, const Eigen::Vector3d &drone_vel, const Eigen::Vector3d &cable_direction, const Eigen::Quaterniond &payload_attitude, const Eigen::Vector3d &attach_point_body_frame);

        // translational dynamic model
        // Eigen::Vector3d ComputeTransDynamics(const Eigen::Vector3d &drones_net_forces, const Eigen::Matrix3d &mass_matrix, const Eigen::Matrix3d &m_D,  const Eigen::Vector3d &payload_bodyrate_acc);
       Eigen::Vector3d ComputeTransDynamics(const Wrench &mavs_net_wrench, const CooperIntertPara &cooper_interact_para,  const Kinematics &kinematics);


        // rotational dynamic model
        // Eigen::Vector3d ComputeRotDynamics(const Eigen::Vector3d &drones_net_forces, const Eigen::Vector3d &drones_net_torques, const Eigen::Matrix3d &m_mass_matrix, const Eigen::Vector3d &payload_bodyrate, const Eigen::Matrix3d &m_C, const Eigen::Matrix3d &m_D, const Eigen::Matrix3d &m_E);

        Eigen::Vector3d ComputeRotDynamics(const Wrench &mavs_net_wrench, const AttachPoint &attach_point, const CooperIntertPara &cooper_interact_para, const Kinematics &kinematics);
        


        Payload();

        double gravity_ = 9.8;

        
        Eigen::Matrix3d matirxBodyrate2EulerRate(const double &phi, const double &theta);


        // solver ruge_kutta
        runge_kutta4<Eigen::Matrix<double, 12, 1>> stepper_;

        double current_step_ = 0;

        bool intial_acc_set_ = false;

    public:


    Payload(const std::vector<Eigen::Vector3d> &v_attach_point_post_bf, const MassProperty &mass_property, const double &step_size);

    // void CalVel4AttachPoint();      

    void SetInitialAccBodyRateAcc(const Eigen::Vector3d &payload_initial_acc);


    // ComputeAttachPointsKinematics computes post, vels and accs of attach points
    void ComputeAttachPointsKinematics();

    // void UpdateVelCollided(const std::vector<std::shared_ptr<UAVCable>> v_drone_cable_ptr);
    void UpdateVelCollided(const std::vector<UAVCable> &v_drone_cable);


    void InputDronesNetWrenches(const Wrench &mavs_net_wrench);


    void InputPayloadInteractPara(const CooperIntertPara &cooper_interact_para);

    // // input mass matrix  
    // void InputMassMatrix(const Eigen::Matrix3d &m_mass_matrix);

    // // input translational dynamic model inputs: drones' net force to the payload and term m_D
    // void InputDronesNetForces(const Eigen::Vector3d &drones_net_force, const Eigen::Matrix3d &m_D);

    // // input rotational dynamic model inputs: drones' net force to the payload and term m_D
    // void InputDronesNetTorques(const Eigen::Vector3d &drones_net_torque, const Eigen::Matrix3d &m_C, const Eigen::Matrix3d &m_E);    

    void ComputeAccBodyRateAcc();

    void GetOneAttachPoint(const size_t &i, AttachPoint &attach_point) const;

    // // GetAttachPointsPos obtain (i+1)th attach points post
    // void GetOneAttachPointPost(const size_t &i, Eigen::Vector3d &attach_point_post) const;

    // // GetAttachPointsPos obtain (i+1)th attach points post in body frame
    // void GetOneAttachPointPostBodyFrame(const size_t &i, Eigen::Vector3d &attach_point_post_bodyframe) const; 

    // // GetAttachPointsVel obtain (i+1)th attach points vel
    // void GetOneAttachPointVel(const size_t &i, Eigen::Vector3d &attach_point_vel) const;    

    // // GetAttachPointsVel obtain (i+1)th attach points acc
    // void GetOneAttachPointAcc(const size_t &i, Eigen::Vector3d &attach_point_acc) const;    


    // void doOneStepInt();

    virtual void DoPayloadOneStepInt() final;

    void operator() (const object_state &x , object_state &dxdt, const double time) override;
};

#endif
