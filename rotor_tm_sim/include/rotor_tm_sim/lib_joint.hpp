#ifndef JOINT_SIMULATOR_H
#define JOINT_SIMULATOR_H

#include <iostream>
#include <Eigen/Dense>
#include <memory>
#include "rotor_tm_sim/base/lib_base.hpp"

class UAVCable; //forward declaration


class Joint
{
    private:
        Pose pose_;

        Eigen::Vector3d post_body_frame_;

        Vels vels_;

        Accs accs_;

        Joint() = delete;

        std::shared_ptr<const UAVCable> ptr_UAVCable_;
    
    public:

        inline Pose pose() const {return pose_;};

        inline Eigen::Vector3d post_body_frame() const {return post_body_frame_;};

        inline Vels vels() const {return vels_;};

        inline Accs accs() const {return accs_;};

        Joint(const Eigen::Vector3d &post_body_frame);

        Joint(const Eigen::Vector3d &post_body_frame, const std::shared_ptr<const UAVCable>& ptr_UAVCable_);

        void LinkUAVCable(const std::shared_ptr<const UAVCable>& ptr_UAVCable) {ptr_UAVCable_ = ptr_UAVCable;};

        void SetLinearVel(const Eigen::Vector3d & linear_vel) {vels_.linear_vel = linear_vel;};

        void SetInitPost(const Eigen::Vector3d & post) {pose_.post = post;};

        void SetLinearAcc(const Eigen::Vector3d & acc) {accs_.linear_acc = acc;};

        std::shared_ptr<const UAVCable> ptr_UAVCable() const {return ptr_UAVCable_;};

        void SetVels(const Vels &vels) {vels_ = vels;};

        void SetPose(const Pose &pose) {pose_ = pose;};

        void SetAccs(const Accs &accs) {accs_ = accs;};

};

#endif
