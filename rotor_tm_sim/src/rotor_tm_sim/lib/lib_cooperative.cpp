#include "rotor_tm_sim/lib_cooperative.hpp"



// Cooperative::Cooperative(const std::vector<Eigen::Vector3d> &v_attach_point_post_bf, const double &mass, const Eigen::Matrix3d &m_inertia, const double &step_size): RigidBody(mass, m_inertia, step_size), v_attach_point_posts_body_frame_(v_attach_point_post_bf)
// {
//     // reserve size of vectors v_attach_point_posts_ and v_attach_point_vels_
//     v_attach_point_posts_.reserve(v_attach_point_posts_body_frame_.size());

//     v_attach_point_vels_.reserve(v_attach_point_posts_body_frame_.size());
// }

void Cooperative::CheckCollision()
{
    

}


Eigen::Vector3d Cooperative::CalVelProjPerpendicularCable(const Eigen::Vector3d drone_vel, const Eigen::Vector3d &cable_direction)
{
    // var of drone vel projected  along cable
    Eigen::Vector3d drone_vel_projection_along_cable(0,0,0);    

    // var of drone vel' projection that is perpendicular to cable
    Eigen::Vector3d drone_vel_projection_perpendicular_cable(0,0,0);

    // similar rule can be found at Eq 30
    drone_vel_projection_along_cable = cable_direction * cable_direction.transpose() *  drone_vel;

    // similar rule can be found at Eq 30
    drone_vel_projection_perpendicular_cable = drone_vel - drone_vel_projection_along_cable;

    return drone_vel_projection_perpendicular_cable;

}




void Cooperative::UpdateVelsCollidedUAVsPayload()
{


}