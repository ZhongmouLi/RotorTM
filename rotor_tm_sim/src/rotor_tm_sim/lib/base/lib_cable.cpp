#include "rotor_tm_sim/base/lib_cable.hpp"



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

    // std::cout<<"relative vel is " << vel_robot2attachpoint_projected_xi_direction<<std::endl;
    
    // bool flag_relative_vel = (vel_robot2attachpoint_projected_xi_direction > k_threshold);

    bool flag_relative_vel = (vel_robot2attachpoint_projected_xi_direction >= 0);

    // 2. compute the distance between attach point and robot
    double distance = (attachpoint_post - robot_post).norm();
    bool flag_distance = (std::abs(distance - length_) > k_threshold); 

    // 3. final taut is true if relative vel is positive and distance = cable length
    // i.e. both of flag_relative_vel and flag_distance must be true

    taut_ = flag_relative_vel && flag_distance;

    // std::cout<<"flag_relative_vel is " << flag_relative_vel<<std::endl;
    // std::cout<<"flag_distance is " << flag_distance<<std::endl;    
}


void Cable::ComputeCableTensionForce(const double &mav_mass, const Eigen::Vector3d &mav_thrust_force, const Eigen::Vector3d &attach_point_acc)
{

    if (taut_ == false)
    {
       std::cout<<"WARN: the cable is slack and there is no tension"<<std::endl;

       return;
    }
    
    // python code to compute tension force for each cable
    // tension = self.uav_params[uav_idx].mass*cable_len*np.sum(cbl_omg**2) - np.matmul(xi, qd_u - self.uav_params[uav_idx].mass * self.attach_accel[uav_idx,:])
    
    // tension_vector[:,uav_idx] = tension * xi

    //for a special case one quadrotor, one cable and one paylaod, on a vertical direction
    // m_q = 1kg, m_p = 0.5kg
    // u|| = 2*9.8N = 19.6N
    // a_whole = 3.2667 m/s^2 // acc for the whole system (quadrotor + cable+ payload)
    //
    // xi = [0 0 -1]^T
    // Newton's law for payload
    // -tension_force =  m_p * (9.8+a_whole) = 6.4807N ===> tension_force =  -6.5334N

    // tension_force = - [0 0 -1] * (u|| - m_d * ( 9.8+a_whole) ) = -6.5334


    // 1. compute tension
    double tension =0;

    // Eigen::Vector3d body_rate_square = (body_rate_.array() * body_rate_.array()).matrix();
    tension = mav_mass * length_ *  body_rate_.squaredNorm() - xi_.dot(mav_thrust_force - mav_mass * attach_point_acc);

    // 2. comupute tension force
    tension_force_ = tension * xi_;

    // std::cout<<"tension is "<< tension << std::endl;

    // std::cout<< "tension force is " << tension_force_.transpose() << std::endl;
}


void Cable::ComputeCableBodyrate(const Eigen::Vector3d &robot_vel, const Eigen::Vector3d &attachpoint_vel)
{
    // obtain unit vector of relative vel of attach point to mav
    Eigen::Vector3d relative_vel_unit = (attachpoint_vel - robot_vel)/length_;

    // 
    body_rate_ = xi_.cross(relative_vel_unit);

}