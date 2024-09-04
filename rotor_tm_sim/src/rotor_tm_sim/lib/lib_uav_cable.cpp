#include "rotor_tm_sim/lib_uav_cable.hpp"


UAVCable::UAVCable(const MassProperty &mav_mass_property, const double & cable_length, const double &step_size):mav_(mav_mass_property, step_size), cable_(cable_length)
{

}

UAVCable::UAVCable(const MassProperty &mav_mass_property, const double & cable_length, const std::shared_ptr<const AttachPoint> &ptr_attach_point, const double &step_size):mav_(mav_mass_property, step_size), cable_(cable_length), ptr_attach_point_(ptr_attach_point)
{
    
}

void UAVCable::DoOneStepInt()
{

    // call one step integration for quadrotor dynamics
    mav_.DoOneStepInt();

};

void UAVCable::UpdateCable()
{
    // update cable direction
    cable_.ComputeCableDirection(ptr_attach_point_->pose().post, mav_.pose().post); //const Eigen::Vector3d &attachpoint_post, const Eigen::Vector3d &robot_post


    cable_.CheckTaut(ptr_attach_point_->pose().post, mav_.pose().post, ptr_attach_point_->vels().linear_vel, mav_.vels().linear_vel); 


    cable_.ComputeCableBodyrate(mav_.vels().linear_vel, ptr_attach_point_->vels().linear_vel);    
}

/*---------------------------- Collision----------------------------*/


// compute and set new vel after collision for mav
// void UAVCable::UpdateMAVVelCollided(const Eigen::Quaterniond &payload_attitude, const Eigen::Vector3d &attach_point_body_frame, const Eigen::Vector3d &payload_vel_collided, const Eigen::Vector3d &payload_bodyrate_collided)
void UAVCable::UpdateMAVVelCollided(const Eigen::Quaterniond &payload_attitude, const Eigen::Vector3d &payload_vel_collided, const Eigen::Vector3d &payload_bodyrate_collided)
{
    // 1.obtain cable's taut status
    // bool cable_taut_status = false;

    // cable_.GetCableTautStatus(cable_taut_status);

    // 2. update Collided Vel only if that MAV just had collision
    // if (cable_taut_status == false)
    if (cable_.tautStatus() == false)
    {
        std::cout<<"MAV had no collision"<<std::endl;
        return;
    }

    std::cout<<"MAV had collision"<<std::endl;

    // 3. obtain drone vel
    // Eigen::Vector3d mav_vel;
    // mav_.GetVel(mav_vel);

    // // 4. obtain cable direction 
    // Eigen::Vector3d xi;
    // cable_.GetCableDirection(xi);

    // 5. calculate drone' vel projected perpendicular to the cable direction
    Eigen::Vector3d mav_vel_proj_perpendicular_cable(0,0,0);
    // mav_vel_proj_perpendicular_cable = CalVelProjPerpendicularCable(mav_vel, xi);
    mav_vel_proj_perpendicular_cable = CalVelProjPerpendicularCable();

    // 6. compute updated drone'vel along the cable direction because of collision
    Eigen::Vector3d mav_vel_collision_along_perpendicular_cable(0,0,0);

    Eigen::Matrix3d payload_R = payload_attitude.toRotationMatrix(); // convert a quaternion to a 3x3 rotation matrix

    // ̇ Eq.56
    // 
    // Eigen::Matrix3d attach_point_post_asym = mav_.TransVector3d2SkewSymMatrix(attach_point_body_frame);
    Eigen::Matrix3d attach_point_post_asym = mav_.TransVector3d2SkewSymMatrix(ptr_attach_point_->post_body_frame());
    

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
void UAVCable::ComputeAttachPointWrenches(const Eigen::Quaterniond &payload_attitude, Eigen::Vector3d &payload_bodyrate)
{

    // compute tension force of cable
    cable_.ComputeCableTensionForce(mav_.mass(), mav_input_wrench_.force, ptr_attach_point_->accs().linear_acc); 
    
    // std::cout<<"tension force is "<< cable_.tensionForce().transpose() <<std::endl;

    // compute net wrench applied to mav
    ComputeNetWrenchApplied2MAV();

    // NOTE: only cable is taut can apply wrenches to attach point
    // obtain cable direction and cable body rate
    // Eigen::Vector3d mav_post;
    // mav_.GetPosition(mav_post);
    // cable_.ComputeCableDirection(attach_point_post, mav_post);


    // Eigen::Vector3d cable_direction;
    // cable_.GetCableDirection(cable_direction);

    // Eigen::Vector3d mav_vel;
    // Eigen::Vector3d cable_bodyrate;
    // mav_.GetVel(mav_vel);

    // cable_.ComputeCableBodyrate(mav_vel, attach_point_vel);
    // cable_.GetCableBodyRate(cable_bodyrate);

    // compute force applied by MAV to payload at attach point    
    // mav_attach_point_force_ = ComputeAttachPointForce(cable_direction, cable_bodyrate, attach_point_post_bf, payload_attitude, payload_bodyrate);
    mav_attach_point_wrench_.force = ComputeAttachPointForce(payload_attitude, payload_bodyrate);

    // std::cout<<"[----------] UAVCable::ComputeAttachPointWrenches ComputeAttachPointTorque begin" << std::endl;
    // compute torque applied by MAV to payload at attach point    
    // mav_attach_point_torque_ = ComputeAttachPointTorque(attach_point_post_bf, payload_attitude, mav_attach_point_force_);
    mav_attach_point_wrench_.torque = ComputeAttachPointTorque(payload_attitude, mav_attach_point_wrench_.force);


    // std::cout<<"[----------] UAVCable::ComputeAttachPointWrenches ComputeAttachPointTorque end" << std::endl;
}






// ComputeAttachPointTorque computes the force applied by MAV at the attach point
// Eigen::Vector3d UAVCable::ComputeAttachPointForce(const Eigen::Vector3d &cable_direction, const Eigen::Vector3d &cable_bodyrate, const Eigen::Vector3d &attach_point_post_bf, const Eigen::Quaterniond &payload_attitude, Eigen::Vector3d &payload_bodyrate)
Eigen::Vector3d UAVCable::ComputeAttachPointForce(const Eigen::Quaterniond &payload_attitude, Eigen::Vector3d &payload_bodyrate)
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
    
    // std::cout<<"[----------] UAVCable::ComputeAttachPointForce cable_direciton is " << cable_direction.transpose() <<std::endl;
    // std::cout<<"[----------] UAVCable::ComputeAttachPointForce mav_thrust_force_along_cable is " << mav_thrust_force_along_cable.transpose() << std::endl;

    // 2. compute attach point centrifugal acc
    Eigen::Vector3d attach_point_centri_acc{0,0,0};

    attach_point_centri_acc = mav_.TransVector3d2SkewSymMatrix(payload_bodyrate) * (mav_.TransVector3d2SkewSymMatrix(payload_bodyrate) * (ptr_attach_point_->post_body_frame()) );


    // 3. compute the force applied by drone to the attach point
    Eigen::Vector3d mav_attach_point_force;
    // double mav_mass;
    // double cable_length;
    // mav_.GetMass(mav_mass);
    // cable_.GetCableLength(cable_length);

    mav_attach_point_force = mav_thrust_force_along_cable - mav_.mass() * cable_.length() * cable_.bodyrate().squaredNorm() * cable_.bodyrate().squaredNorm() * cable_.direction() - mav_.mass()* ( (cable_.direction() * cable_.direction().transpose()) * (payload_attitude.toRotationMatrix() * attach_point_centri_acc));

    // std::cout<<"[----------] UAVCable::ComputeAttachPointForce mav_attach_point_force is " << mav_attach_point_force.transpose() << std::endl;
    return mav_attach_point_force;

}


// ComputeAttachPointTorque computes the torque applied by MAV at the attach point
// Eigen::Vector3d UAVCable::ComputeAttachPointTorque(const Eigen::Vector3d &attach_point_post_bf, const Eigen::Quaterniond &payload_attitude, Eigen::Vector3d &attach_point_force)
Eigen::Vector3d UAVCable::ComputeAttachPointTorque(const Eigen::Quaterniond &payload_attitude, Eigen::Vector3d &attach_point_force)
{

    Eigen::Vector3d mav_attach_point_torque(0,0,0);

    mav_attach_point_torque = mav_.TransVector3d2SkewSymMatrix(ptr_attach_point_->post_body_frame()) * (payload_attitude.toRotationMatrix().transpose() * attach_point_force);

    // std::cout<<"[----------] UAVCable::ComputeAttachPointTorque mav_attach_point_force is " << attach_point_force.transpose() << std::endl;

    // std::cout<<"[----------] UAVCable::ComputeAttachPointTorque TransVector3d2SkewSymMatrix(attach_point_post) is "<< mav_.TransVector3d2SkewSymMatrix(attach_point_post_bf)<<std::endl;

    // std::cout<<"[----------] UAVCable::ComputeAttachPointTorque payload_attitude.toRotationMatrix().transpose() is "<< payload_attitude.toRotationMatrix().transpose()<<std::endl;

    // std::cout<<"[----------] UAVCable::ComputeAttachPointTorque payload_attitude.toRotationMatrix().transpose() * attach_point_force is " << payload_attitude.toRotationMatrix().transpose() * attach_point_force << std::endl;

    // std::cout<<"[----------] UAVCable::ComputeAttachPointTorque mav_attach_point_torque is "<<mav_attach_point_torque.transpose()<<std::endl;    

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
    m_C_i_ = mav_.mass() * mav_.TransVector3d2SkewSymMatrix(ptr_attach_point_->post_body_frame()) * (payload_attitude.toRotationMatrix().transpose() * (cable_.direction() * cable_.direction().transpose()) );


    // compute m_D_i = m_i * xi * xi^T * 0^R_{payload} * skew_matrix ( {payload}^p_{attach_point} )
    // m_D_i_ = mav_mass * cable_direction * cable_direction.transpose()* payload_attitude.toRotationMatrix()* mav_.TransVector3d2SkewSymMatrix(attach_point_post_bf);
    m_D_i_ = - m_C_i_.transpose();


    // compute m_E_i = m_i * skew_matrix({payload}^p_{attach_point}) * 0^R_{payload}^T * xi * xi^T *  0^R_{payload} * skew_matrix ( {payload}^p_{attach_point} )
    // m_E_i_ = mav_.TransVector3d2SkewSymMatrix(attach_point_post_bf) * payload_attitude.toRotationMatrix().transpose() * m_D_i_;
    // m_E_i_ =  m_C_i_ * (payload_attitude.toRotationMatrix() * mav_.TransVector3d2SkewSymMatrix(attach_point_post_bf));   
    m_E_i_ =  m_C_i_ * (payload_attitude.toRotationMatrix() * mav_.TransVector3d2SkewSymMatrix(ptr_attach_point_->post_body_frame()));   
    // auto fuck1 = m_C_i_;
    // auto fuck2 = payload_attitude.toRotationMatrix();
    // auto fuck3 =attach_point_post_bf;

    // std::cout<<"--------------------------------m_C_i_"<<fuck1<<std::endl;
    // std::cout<<"--------------------------------payload_attitude.toRotationMatrix()"<<fuck2<<std::endl;
    // std::cout<<"--------------------------------attach_point_post_bf"<<fuck3<<std::endl;
    //  std::cout<<"---------------------------------------"<<std::endl << fuck1 * (fuck2 * fuck3) <<std::endl;
}



/*-------------------------Control interface-------------------------*/

// ComputeNetWrenchApplied2MAV computes the net wrench applied to MAV based on the cable' status
// cable is taut  --> no tension force of cable
// cable is slack --> compute tension force of cable and caculate net force for MAV
// void UAVCable::ComputeNetWrenchApplied2MAV(const Eigen::Vector3d &attach_point_acc)
void UAVCable::ComputeNetWrenchApplied2MAV()
{
    // 1 obtain cable's taut status
    // bool cable_taut_status;

    // cable_.GetCableTautStatus(cable_taut_status);


    // 2. call dynamic simulation based on status of cable' taut
    // define mav_net_input_force as the net input force for drone apart from gravity
    // Eigen::Vector3d mav_net_input_force;

    Wrench mav_net_input_wrench = mav_input_wrench_;

    // if (cable_taut_status == true)
    if (cable_.tautStatus() == true)
        {
            // cable is taut and there is a tension force
            // Eigen::Vector3d tension_force;

            // compute tension force
            // double mav_mass;
            // mav_.GetMass(mav_mass);

            // to thrust force in world frame
            // note this = 0^R_i T_i where T_i is from control input
            // Eigen::Quaterniond mav_attitude;

            // mav_.GetAttitude(mav_attitude);

            // Eigen::Matrix3d mav_rot_matrix = mav_attitude.toRotationMatrix();
            // Eigen::Matrix3d mav_rot_matrix = mav_.pose().att.toRotationMatrix();

            // Eigen::Vector3d mav_thrust_force =  mav_rot_matrix * (Eigen::Vector3d::UnitZ() * mav_thrust_input_);

            // std::cout<<"[----------] UAVCable::ComputeNetWrenchApplied2MAV mav_thrust_input_ is " << mav_thrust_input_<< std::endl;
            // std::cout<<"[----------] UAVCable::ComputeNetWrenchApplied2MAV mav_thrust_force is " << mav_thrust_force.transpose() << std::endl;

            // Eigen::Vector3d cable_direction;
            
            // cable_.GetCableDirection(cable_direction);

            // add
            // cable_.ComputeCableTensionForce(mav_mass, mav_thrust_force, attach_point_acc);

            // Eigen::Vector3d cable_tension_force;
            // cable_.GetCableTensionForce(cable_tension_force);

            // compute net input force of drone
            // net input force = thrust force - cable tension force
            // mav_net_input_force = mav_thrust_force + cable_tension_force;
            mav_net_input_wrench.force = mav_input_wrench_.force + cable_.tensionForce();

            // input net input force for to drone
            // mav_.InputForce(mav_net_input_force);
            // mav_net_input_wrench.force = mav_net_input_force;

            // std::cout<<"[----------] UAVCable::ComputeNetWrenchApplied2MAV cable is taut"  << std::endl;
            // std::cout<<"[----------] UAVCable::ComputeNetWrenchApplied2MAV mav_net_input_force is"  << mav_net_input_force.transpose() <<  std::endl;
            // std::cout<<"[----------] UAVCable::ComputeNetWrenchApplied2MAV cable_tension_force is"  << cable_tension_force.transpose() <<  std::endl;
        }
    else
        {
          // cable is slack and there is no tension
          // mav only suffers from gravity and thrust force  
        //   mav_.InputThurst(mav_thrust_input_);
        }

        // mav rotation is independent
        // mav_.InputTorque(mav_torque_input_);

        mav_.InputWrench(mav_net_input_wrench);

}


// input control inputs from controller
void UAVCable::InputControllerInput(const double &mav_thrust, const Eigen::Vector3d &mav_torque)
{

    // mav_torque_input_ = mav_torque;
    // mav_thrust_input_ = mav_thrust;
    // mav_.InputThurst(mav_thrust);
    // std::cout<<"[----------] UAVCable:InputControllerInput mav_thrust_input_ is " << mav_thrust_input_ << std::endl;
    // mav_.InputTorque(mav_torque);

    mav_input_wrench_.force = mav_.pose().att.toRotationMatrix() * ( Eigen::Vector3d::UnitZ() *  mav_thrust);

    mav_input_wrench_.torque = mav_torque;
}



// obtain parameters for payload dynamic equation
void UAVCable::GetMatrixMDiMCiMEi(Eigen::Matrix3d &m_C_i, Eigen::Matrix3d &m_D_i, Eigen::Matrix3d &m_E_i) const
{
    m_C_i = m_C_i_; 
    m_D_i = m_D_i_;
    m_E_i = m_E_i_;
}






void UAVCable::SetMAVInitPost(const Eigen::Vector3d &mav_post)
{
    mav_.SetInitialPost(mav_post);
    
}


void UAVCable::SetMAVInitPostCableTautWithAttachPointPost(const Eigen::Vector3d &attach_point_init_post)
{
    // get cable length
    // double cable_length;

    // cable_.GetCableLength(cable_length);

    // mav init post = [x_payload, y_payload, z_payload + cable_length]
    // Eigen::Vector3d mav_init_post = attach_point_init_post + Eigen::Vector3d::UnitZ() * cable_length;

    Eigen::Vector3d mav_init_post = attach_point_init_post + Eigen::Vector3d::UnitZ() * cable_.length();

    //
    mav_.SetInitialPost(mav_init_post);
}
