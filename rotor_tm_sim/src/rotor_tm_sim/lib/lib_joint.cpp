#include "rotor_tm_sim/lib_joint.hpp"
#include "rotor_tm_sim/lib_uav_cable.hpp"

Joint::Joint(const Eigen::Vector3d &post_body_frame)://
          pose_(post_body_frame, Eigen::Quaterniond::Identity()), //
          post_body_frame_(post_body_frame),                
          vels_(),                      
          accs_() 
{

};


Joint::Joint(const Eigen::Vector3d &post_body_frame, const std::shared_ptr<const UAVCable>& ptr_UAVCable)://
          pose_(post_body_frame, Eigen::Quaterniond::Identity()), //
          post_body_frame_(post_body_frame),                
          vels_(),                      
          accs_(),
          ptr_UAVCable_(ptr_UAVCable) 
{

};