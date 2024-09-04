#ifndef ATTACHPOINT_SIMULATOR_H
#define ATTACHPOINT_SIMULATOR_H

#include <iostream>
#include <Eigen/Dense>
#include "rotor_tm_sim/lib_base.hpp"

class AttachPoint
{
    private:
        Pose pose_;

        Eigen::Vector3d post_body_frame_;

        Vels vels_;

        Accs accs_;

        AttachPoint() = delete ;
    
    public:

        inline Pose pose() const {return pose_;};

        inline Eigen::Vector3d post_body_frame() const {return post_body_frame_;};

        inline Vels vels() const {return vels_;};

        inline Accs accs() const {return accs_;};

        AttachPoint(const Eigen::Vector3d &post_body_frame);

        void SetLinearVel(const Eigen::Vector3d & linear_vel) {vels_.linear_vel = linear_vel;};

        void SetInitPost(const Eigen::Vector3d & post) {pose_.post = post;};

        void SetLinearAcc(const Eigen::Vector3d & acc) {accs_.linear_acc = acc;};

};

#endif
