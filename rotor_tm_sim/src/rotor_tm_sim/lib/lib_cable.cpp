#include "rotor_tm_sim/lib_cable.hpp"



Cable::Cable(const double &length):length_(length)
{

};


void Cable::ComputeCableDirection(const Eigen::Vector3d &attachpoint_post, const Eigen::Vector3d &robot_post)
{

// xi = (attach_pos - robot_pos) / np.linalg.norm(attach_pos - robot_pos, 2, 0)
    double distance = (attachpoint_post - robot_post).norm();

    xi_ = (attachpoint_post - robot_post)/distance;
};


void Cable::CheckTaut(const Eigen::Vector3d &attachpoint_post, const Eigen::Vector3d &robot_post, const Eigen::Vector3d &attachpoint_vel, const Eigen::Vector3d &robot_vel)
{

    // 1. compute the projection of relative vel of robot to attach point on cable  whose direction is represented by xi
    double vel_robot2attachpoint_projected_xi_direction;

    Eigen::Vector3d vel_robot2attachpoint = attachpoint_vel - robot_vel;

    vel_robot2attachpoint_projected_xi_direction = xi_.dot(vel_robot2attachpoint);
    
    bool flag_relative_vel = (vel_robot2attachpoint_projected_xi_direction > k_threshold);

    // 2. compute the distance between attach point and robot
    double distance = (attachpoint_post - robot_post).norm();
    bool flag_distance = (std::abs(distance - length_) > k_threshold); 

    // 3. final taut is true if relative vel is positive and distance = cable length
    // i.e. both of flag_relative_vel and flag_distance must be true

    taut_ = flag_relative_vel && flag_distance;
}


