#ifndef QUADROTORCOOPERATIVE_SIMULATOR_H
#define QUADROTORCOOPERATIVE_SIMULATOR_H


#include <iostream>
#include <Eigen/Dense>
#include <iomanip>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>
#include <string> 
#include <iomanip> // For std::setprecision
#include "lib_uav_cable.hpp"
#include "lib_payload.hpp"


class Cooperative{

    private:

        // number of robots
        size_t number_robots_;

        // controller inputs for n MAVs
        // for each MAV, the control input is wrench        
        std::vector<ThrustWrench> v_controllers_inputs_;

        Cooperative() =  delete;

        Wrench net_mavs_wrench_to_payload_;

        CooperIntertPara interaction_parameters_;

        void ClearNetWrenches2Payload();

        bool flag_first_iteration_ = true;

    public:

        // put them in private LATER
        // vector of cable instances
        std::shared_ptr<Payload> ptr_payload_;

        std::vector<std::shared_ptr<Joint>> v_ptr_joints_;

        std::vector<std::shared_ptr<UAVCable>> v_ptr_uavcables_;



        Cooperative(const std::shared_ptr<Payload>& ptr_payload, const std::vector<std::shared_ptr<Joint>>&v_ptr_joints, const std::vector<std::shared_ptr<UAVCable>>& v_ptr_uavcables_);

        Cooperative(const std::shared_ptr<Payload>& ptr_payload, const std::shared_ptr<Joint> &ptr_joint, const std::shared_ptr<UAVCable>& ptr_uavcable_);

        // CheckCollisions4MAVsPayload checks if collission happens for every cable
        // void CheckCollisions4MAVCables();

        // set init post of payload
        void SetPayloadInitPost();

        void SetPayloadInitPost(const Eigen::Vector3d &payload_init_post);    

        void SetPayloadInitPost(const Eigen::Vector3d &payload_init_post, const double& tilt_angle);

        // void SetPayloadInitialAccAndBodyrateACC();
        void UpdateJointAndCableStatus();

        // update vels of collided MAVs and payload 
        // method: eq (39)-(42)
        void UpdateVelsCollidedUAVsPayload();

        void InputControllerInput4MAVs(const std::vector<double> v_mavs_thrust, const std::vector<Eigen::Vector3d> v_mavs_torque);

        // compute interation wrenches and vars for MAVs and payload
        void ComputeInteractWrenches();

        // call one step dynamic simulation for MAVs and payload
        void DoOneStepInt4Robots();

        Wrench NetMavsWrenchToPayload() const {return net_mavs_wrench_to_payload_;};
};


#endif