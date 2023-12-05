#include "rotor_tm_sim/lib_payload.hpp"



Payload::Payload(const std::vector<Eigen::Vector3d> &v_attach_point_post, const double &mass, const Eigen::Matrix3d &m_inertia, const double &step_size): RigidBody(mass, m_inertia, step_size), v_attach_point_posts_body_frame_(v_attach_point_post)
{
    
}




void Payload::UpdateMotion4AttachPoint()
{

    Eigen::Vector3d payload_position;

    Eigen::Quaterniond payload_attitude;

    getPosition(payload_position);

    getAttitude(payload_attitude);

    Eigen::Matrix3d payload_rotation_matrix = payload_attitude.toRotationMatrix();


    // compute attach point posts in world frame and store them in v_attach_point_posts_
    // formula: attach_point_world_frame = payload_position + (payload_rotation_matrix* attach_point_body_frame)
    std::transform(v_attach_point_posts_body_frame_.begin(), v_attach_point_posts_body_frame_.end(), v_attach_point_posts_.begin(),[payload_position, payload_rotation_matrix](auto attach_point_body_frame){ //
                                                                                Eigen::Vector3d attach_point_world_frame;
                                                                                attach_point_world_frame = payload_position + (payload_rotation_matrix* attach_point_body_frame);
                                                                                return attach_point_world_frame ; });

    // compute attach point vels in word frame and store them 
}