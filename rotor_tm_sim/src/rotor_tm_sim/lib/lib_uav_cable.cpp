#include "rotor_tm_sim/lib_uav_cable.hpp"
#include "rotor_tm_sim/lib_joint.hpp"

UAVCable::UAVCable(const MassProperty &mav_mass_property, const double & cable_length, const double &step_size):mav_(mav_mass_property, step_size), cable_(cable_length)
{

}

UAVCable::UAVCable(const MassProperty &mav_mass_property, const double & cable_length, const std::shared_ptr<const Joint> &ptr_joint, const double &step_size):mav_(mav_mass_property, step_size), cable_(cable_length), ptr_joint_(ptr_joint)
{
    
}


std::shared_ptr<const Joint> UAVCable::ptr_joint()
{
    // auto ptr_joint = ptr_joint_.lock(); 
    
    // if (ptr_joint)
    // {
    //     return ptr_joint;
    // }
    // else 
    // {
    //     std::cout<< "shared_ptr expired"<<std::endl;
    // };
    return ptr_joint_.lock();
};



void UAVCable::DoOneStepInt()
{

    // call one step integration for quadrotor dynamics
    mav_.DoOneStepInt();

};

void UAVCable::UpdateCable()
{
    // update cable direction
    cable_.ComputeCableDirection(ptr_joint()->pose().post, mav_.pose().post); //const Eigen::Vector3d &attachpoint_post, const Eigen::Vector3d &robot_post


    cable_.CheckTaut(ptr_joint()->pose().post, mav_.pose().post, ptr_joint()->vels().linear_vel, mav_.vels().linear_vel); 


    cable_.ComputeCableBodyrate(mav_.vels().linear_vel, ptr_joint()->vels().linear_vel);    
}

/*---------------------------- Collision----------------------------*/


// compute and set new vel after collision for mav
// void UAVCable::UpdateMAVVelCollided(const Eigen::Quaterniond &payload_attitude, const Eigen::Vector3d &attach_point_body_frame, const Eigen::Vector3d &payload_vel_collided, const Eigen::Vector3d &payload_bodyrate_collided)
void UAVCable::UpdateMAVVelCollided(const Eigen::Quaterniond &payload_attitude, const Eigen::Vector3d &payload_vel_collided, const Eigen::Vector3d &payload_bodyrate_collided)
{
    // 1. update Collided Vel only if that MAV just had collision
    // if (cable_taut_status == false)
    if (cable_.tautStatus() == false)
    {
        std::cout<<"MAV had no collision"<<std::endl;
        return;
    }

    std::cout<<"MAV had collision"<<std::endl;


    // 2. calculate drone' vel projected perpendicular to the cable direction
    Eigen::Vector3d mav_vel_proj_perpendicular_cable(0,0,0);
    // mav_vel_proj_perpendicular_cable = CalVelProjPerpendicularCable(mav_vel, xi);
    mav_vel_proj_perpendicular_cable = CalVelProjPerpendicularCable();

    // 6. compute updated drone'vel along the cable direction because of collision
    Eigen::Vector3d mav_vel_collision_along_perpendicular_cable(0,0,0);

    Eigen::Matrix3d payload_R = payload_attitude.toRotationMatrix(); // convert a quaternion to a 3x3 rotation matrix

    // ̇ Eq.56
    // 
    // Eigen::Matrix3d attach_point_post_asym = mav_.TransVector3d2SkewSymMatrix(attach_point_body_frame);
    Eigen::Matrix3d attach_point_post_asym = mav_.TransVector3d2SkewSymMatrix(ptr_joint()->post_body_frame());
    

    // python code collided_robot_vel_proj = xi * sum(xi * (collided_pl_vel + pl_rot @ utilslib.vec2asym(collided_pl_omg) @ rho_vec_list), 0)
    // mav_vel_collision_along_perpendicular_cable = xi * xi.transpose() * (payload_vel_collided - payload_R * attach_point_post_asym * payload_bodyrate_collided);

    mav_vel_collision_along_perpendicular_cable = cable_.direction() * cable_.direction().transpose() * (payload_vel_collided - payload_R * attach_point_post_asym * payload_bodyrate_collided);

    // compute final drone vel = mav_vel_proj_perpendicular_cable (not influnced by collision) + mav_vel_collision_along_perpendicular_cable (updated by sollision)
    Eigen::Vector3d mav_vel_collided;
    mav_vel_collided = mav_vel_proj_perpendicular_cable + mav_vel_collision_along_perpendicular_cable;

    // set drone's vel
    mav_.SetVel(mav_vel_collided);
}


// Eigen::Vector3d UAVCable::CalVelProjPerpendicularCable(const Eigen::Vector3d mav_vel, const Eigen::Vector3d &cable_direction)
Eigen::Vector3d UAVCable::CalVelProjPerpendicularCable()
{
    // var of drone vel projected  along cable
    Eigen::Vector3d mav_vel_projection_along_cable(0,0,0);    

    // var of drone vel' projection that is perpendicular to cable
    Eigen::Vector3d mav_vel_projection_perpendicular_cable(0,0,0);

    // similar rule can be found at Eq 30
    // mav_vel_projection_along_cable = cable_direction * cable_direction.transpose() *  mav_vel;
    mav_vel_projection_along_cable = cable_.direction() * cable_.direction().transpose() *  mav_.vels().linear_vel;

    
    // similar rule can be found at Eq 30
    // mav_vel_projection_perpendicular_cable = mav_vel - mav_vel_projection_along_cable;
    mav_vel_projection_perpendicular_cable = mav_.vels().linear_vel - mav_vel_projection_along_cable;

    return mav_vel_projection_perpendicular_cable;

}



/*-------------------------Dynamic-------------------------*/


// void UAVCable::ComputeAttachPointWrenches(const Eigen::Vector3d &attach_point_post_bf, const Eigen::Vector3d &attach_point_post, const Eigen::Vector3d &attach_point_vel, const Eigen::Quaterniond &payload_attitude, Eigen::Vector3d &payload_bodyrate)
void UAVCable::ComputeInteractionWrenches(const Eigen::Quaterniond &payload_attitude, Eigen::Vector3d &payload_bodyrate)
{

    // compute tension force of cable
    cable_.ComputeCableTensionForce(mav_.mass(), mav_input_wrench_.force, ptr_joint()->accs().linear_acc); 
    
    // std::cout<<"tension force is "<< cable_.tensionForce().transpose() <<std::endl;

    // compute net wrench applied to mav
    ComputeNetWrenchApplied2MAV();

    // NOTE: only cable is taut can apply wrenches to attach point
    // compute force applied by MAV to payload at attach point    
    // mav_attach_point_force_ = ComputeAttachPointForce(cable_direction, cable_bodyrate, attach_point_post_bf, payload_attitude, payload_bodyrate);
    mav_attach_point_wrench_.force = ComputeNetForceApplied2AttachPoint(payload_attitude, payload_bodyrate);

    // std::cout<<"[----------] UAVCable::ComputeAttachPointWrenches ComputeAttachPointTorque begin" << std::endl;
    // compute torque applied by MAV to payload at attach point    
    // mav_attach_point_torque_ = ComputeAttachPointTorque(attach_point_post_bf, payload_attitude, mav_attach_point_force_);
    mav_attach_point_wrench_.torque = ComputeNetTorqueApplied2AttachPoint(payload_attitude, mav_attach_point_wrench_.force);


    // std::cout<<"[----------] UAVCable::ComputeAttachPointWrenches ComputeAttachPointTorque end" << std::endl;
}






// ComputeAttachPointTorque computes the force applied by MAV at the attach point
// Eigen::Vector3d UAVCable::ComputeAttachPointForce(const Eigen::Vector3d &cable_direction, const Eigen::Vector3d &cable_bodyrate, const Eigen::Vector3d &attach_point_post_bf, const Eigen::Quaterniond &payload_attitude, Eigen::Vector3d &payload_bodyrate)
Eigen::Vector3d UAVCable::ComputeNetForceApplied2AttachPoint(const Eigen::Quaterniond &payload_attitude, Eigen::Vector3d &payload_bodyrate)
{

    // 1. cal uav thrust force along cable direction in world frame
    Eigen::Vector3d mav_thrust_force_along_cable{0,0,0};

    // compute mav thrust force in world frame
    // Eigen::Quaterniond mav_attitude;
    // mav_.GetAttitude(mav_attitude);

    // Eigen::Vector3d mav_thrust_force{0,0,0};
    // mav_thrust_force = mav_.pose().att.toRotationMatrix() * ( Eigen::Vector3d::UnitZ() *  mav_thrust_input_);
    
    // std::cout<<"[----------] UAVCable::ComputeAttachPointForce mav_thrust_input_ is " << mav_thrust_input_ << std::endl;
    // std::cout<<"[----------] UAVCable::ComputeAttachPointForce mav_thrust_force is " << mav_thrust_force.transpose() << std::endl;

    // obtain cable direction
    // mav_thrust_force_along_cable= cable_direction  * cable_direction.transpose() * mav_thrust_force;
    mav_thrust_force_along_cable= cable_.direction()  * cable_.direction().transpose() * mav_input_wrench_.force;
    

    // 2. compute attach point centrifugal acc
    Eigen::Vector3d attach_point_centri_acc{0,0,0};

    attach_point_centri_acc = mav_.TransVector3d2SkewSymMatrix(payload_bodyrate) * (mav_.TransVector3d2SkewSymMatrix(payload_bodyrate) * (ptr_joint()->post_body_frame()) );


    // 3. compute the force applied by drone to the attach point
    Eigen::Vector3d mav_attach_point_force;


    mav_attach_point_force = mav_thrust_force_along_cable - mav_.mass() * cable_.length() * cable_.bodyrate().squaredNorm() * cable_.bodyrate().squaredNorm() * cable_.direction() - mav_.mass()* ( (cable_.direction() * cable_.direction().transpose()) * (payload_attitude.toRotationMatrix() * attach_point_centri_acc));

    return mav_attach_point_force;

}


// ComputeAttachPointTorque computes the torque applied by MAV at the attach point
// Eigen::Vector3d UAVCable::ComputeAttachPointTorque(const Eigen::Vector3d &attach_point_post_bf, const Eigen::Quaterniond &payload_attitude, Eigen::Vector3d &attach_point_force)
Eigen::Vector3d UAVCable::ComputeNetTorqueApplied2AttachPoint(const Eigen::Quaterniond &payload_attitude, Eigen::Vector3d &attach_point_force)
{

    Eigen::Vector3d mav_attach_point_torque(0,0,0);

    mav_attach_point_torque = mav_.TransVector3d2SkewSymMatrix(ptr_joint()->post_body_frame()) * (payload_attitude.toRotationMatrix().transpose() * attach_point_force);


    return mav_attach_point_torque;
}


// void UAVCable::ComputeMatrixMDiMCiMEi(const Eigen::Vector3d & cable_direction, const Eigen::Quaterniond &payload_attitude, const Eigen::Vector3d &attach_point_post_bf)
void UAVCable::ComputeMatrixMDiMCiMEi(const Eigen::Quaterniond &payload_attitude)
{
    // double mav_mass;
    // mav_.GetMass(mav_mass);

    // Ck = self.uav_params[uav_idx].mass * np.matmul(rho_qn_asym, np.matmul(pl_rot.T, xixiT))
    // Dk = - np.transpose(Ck)
    // Ek = np.matmul(Ck, np.matmul(pl_rot, rho_qn_asym))

    // compute m_C_i = m_i * skew_matrix({payload}^p_{attach_point}) * 0^R_{payload}^T * xi * xi^T 
    // m_C_i_ = mav_mass * mav_.TransVector3d2SkewSymMatrix(attach_point_post_bf) * (payload_attitude.toRotationMatrix().transpose() * (cable_direction * cable_direction.transpose()) );
    m_C_i_ = mav_.mass() * mav_.TransVector3d2SkewSymMatrix(ptr_joint()->post_body_frame()) * (payload_attitude.toRotationMatrix().transpose() * (cable_.direction() * cable_.direction().transpose()) );


    // compute m_D_i = m_i * xi * xi^T * 0^R_{payload} * skew_matrix ( {payload}^p_{attach_point} )
    // m_D_i_ = mav_mass * cable_direction * cable_direction.transpose()* payload_attitude.toRotationMatrix()* mav_.TransVector3d2SkewSymMatrix(attach_point_post_bf);
    m_D_i_ = - m_C_i_.transpose();


    // compute m_E_i = m_i * skew_matrix({payload}^p_{attach_point}) * 0^R_{payload}^T * xi * xi^T *  0^R_{payload} * skew_matrix ( {payload}^p_{attach_point} )
    // m_E_i_ = mav_.TransVector3d2SkewSymMatrix(attach_point_post_bf) * payload_attitude.toRotationMatrix().transpose() * m_D_i_;
    // m_E_i_ =  m_C_i_ * (payload_attitude.toRotationMatrix() * mav_.TransVector3d2SkewSymMatrix(attach_point_post_bf));   
    m_E_i_ =  m_C_i_ * (payload_attitude.toRotationMatrix() * mav_.TransVector3d2SkewSymMatrix(ptr_joint()->post_body_frame()));   
}



/*-------------------------Control interface-------------------------*/

// ComputeNetWrenchApplied2MAV computes the net wrench applied to MAV based on the cable' status
// cable is taut  --> no tension force of cable
// cable is slack --> compute tension force of cable and caculate net force for MAV
// void UAVCable::ComputeNetWrenchApplied2MAV(const Eigen::Vector3d &attach_point_acc)
void UAVCable::ComputeNetWrenchApplied2MAV()
{
    // 1. call dynamic simulation based on status of cable' taut
    // define mav_net_input_wrench as the net input wrench for drone apart from gravity

    Wrench mav_net_input_wrench = mav_input_wrench_;

    // if (cable_taut_status == true)
    if (cable_.tautStatus() == true)
        {
            // compute net input force of drone
            // net input force = thrust force - cable tension force
            // mav_net_input_force = mav_thrust_force + cable_tension_force;
            mav_net_input_wrench.force = mav_input_wrench_.force + cable_.tensionForce();

        };



    mav_.InputWrench(mav_net_input_wrench);

}


// input control inputs from controller
void UAVCable::InputControllerInput(const double &mav_thrust, const Eigen::Vector3d &mav_torque)
{

    mav_input_wrench_.force = mav_.pose().att.toRotationMatrix() * ( Eigen::Vector3d::UnitZ() *  mav_thrust);

    mav_input_wrench_.torque = mav_torque;
}



// obtain parameters for payload dynamic equation
// void UAVCable::GetMatrixMDiMCiMEi(Eigen::Matrix3d &m_C_i, Eigen::Matrix3d &m_D_i, Eigen::Matrix3d &m_E_i) const
// {
//     m_C_i = m_C_i_; 
//     m_D_i = m_D_i_;
//     m_E_i = m_E_i_;
// }



void UAVCable::SetMAVInitPost(const Eigen::Vector3d &mav_post)
{
    mav_.SetInitialPost(mav_post);
    
}


void UAVCable::SetMAVInitPostCableTautWithAttachPointPost(const Eigen::Vector3d &attach_point_init_post)
{
    Eigen::Vector3d mav_init_post = attach_point_init_post + Eigen::Vector3d::UnitZ() * cable_.length();

    //
    mav_.SetInitialPost(mav_init_post);
}
