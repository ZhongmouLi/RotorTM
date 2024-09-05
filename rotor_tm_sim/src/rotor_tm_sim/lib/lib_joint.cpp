#include "rotor_tm_sim/lib_joint.hpp"


Joint::Joint(const Eigen::Vector3d &post_body_frame)://
          pose_(post_body_frame, Eigen::Quaterniond::Identity()), //
          post_body_frame_(post_body_frame),                
          vels_(),                      
          accs_() 
{

};
