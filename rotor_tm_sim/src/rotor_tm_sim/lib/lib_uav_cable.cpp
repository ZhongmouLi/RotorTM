#include "rotor_tm_sim/lib_uav_cable.hpp"

// CheckCollision first computes the cable direction and then check if it is taut or not
void UAVCable::CheckCollision(const Eigen::Vector3d &attachpoint_post, const Eigen::Vector3d &attachpoint_vel)
{

    // compute cable direction
    Eigen::Vector3d drone_vel;
    Eigen::Vector3d drone_post;

    drone_.GetVel(drone_vel);
    drone_.GetPosition(drone_post);

    cable_.ComputeCableDirection(attachpoint_post, drone_post);

    // check taut condition
    cable_.CheckTaut(attachpoint_post, drone_post, attachpoint_vel, drone_vel);
}


// 
void UAVCable::UpdateVelCollidedUAVVel(const Eigen::Quaterniond &payload_attitude, const Eigen::Vector3d &attach_point_body_frame, const Eigen::Vector3d &payload_vel_collided, const Eigen::Vector3d &payload_bodyrate_collided)
{
    // 1.obtain cable's taut status
    bool cable_taut_status = false;

    cable_.GetCableTautStatus(cable_taut_status);

    // 2. update Collided Vel only if that MAV just had collision
    if (cable_taut_status == false)
    {
        std::cout<<"MAV had no collision"<<std::endl;
        return;
    }

    std::cout<<"MAV had collision"<<std::endl;

    // 3. obtain drone vel
    Eigen::Vector3d drone_vel;
    drone_.GetVel(drone_vel);

    // 4. obtain cable direction 
    Eigen::Vector3d xi;
    cable_.GetCableDirection(xi);

    // 5. calculate drone' vel projected perpendicular to the cable direction
    Eigen::Vector3d drone_vel_proj_perpendicular_cable(0,0,0);
    drone_vel_proj_perpendicular_cable = CalVelProjPerpendicularCable(drone_vel, xi);

    // 6. compute updated drone'vel along the cable direction because of collision
    Eigen::Vector3d drone_vel_collision_along_perpendicular_cable(0,0,0);

    Eigen::Matrix3d payload_R = payload_attitude.toRotationMatrix(); // convert a quaternion to a 3x3 rotation matrix

    // ̇ Eq.56
    // 
    Eigen::Matrix3d attach_point_post_asym = drone_.TransVector3d2SkewSymMatrix(attach_point_body_frame);

    // python code collided_robot_vel_proj = xi * sum(xi * (collided_pl_vel + pl_rot @ utilslib.vec2asym(collided_pl_omg) @ rho_vec_list), 0)
    drone_vel_collision_along_perpendicular_cable = xi * xi.transpose() * (payload_vel_collided - payload_R * attach_point_post_asym * payload_bodyrate_collided);

    // compute final drone vel = drone_vel_proj_perpendicular_cable (not influnced by collision) + drone_vel_collision_along_perpendicular_cable (updated by sollision)
    Eigen::Vector3d drone_vel_collided;
    drone_vel_collided = drone_vel_proj_perpendicular_cable + drone_vel_collision_along_perpendicular_cable;

    // set drone's vel
    drone_.SetVel(drone_vel_collided);
}


Eigen::Vector3d UAVCable::CalVelProjPerpendicularCable(const Eigen::Vector3d drone_vel, const Eigen::Vector3d &cable_direction)
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



/*-------------------------Dynamic-------------------------*/
void UAVCable::ComputeMatrixMDiMCiMEi(const Eigen::Vector3d & cable_direction, const Eigen::Quaterniond &payload_attitude, const Eigen::Vector3d &attach_point_post)
{
    double drone_mass;
    drone_.GetMass(drone_mass);

    // compute m_D_i = m_i * xi * xi^T * 0^R_{payload} * skew_matrix ( {payload}^p_{attach_point} )
    m_D_i_ = drone_mass * cable_direction * cable_direction.transpose()* payload_attitude.toRotationMatrix()* drone_.TransVector3d2SkewSymMatrix(attach_point_post);

    // compute m_C_i = m_i * skew_matrix({payload}^p_{attach_point}) * 0^R_{payload}^T * xi * xi^T 
    m_C_i_ = drone_mass * drone_.TransVector3d2SkewSymMatrix(attach_point_post) * payload_attitude.toRotationMatrix().transpose() * cable_direction * cable_direction.transpose();

    // compute m_E_i = m_i * skew_matrix({payload}^p_{attach_point}) * 0^R_{payload}^T * xi * xi^T *  0^R_{payload} * skew_matrix ( {payload}^p_{attach_point} )
    m_E_i_ = drone_.TransVector3d2SkewSymMatrix(attach_point_post) * payload_attitude.toRotationMatrix().transpose() * m_D_i_;
}


void UAVCable::ComputeAttachPointWrenches(const Eigen::Vector3d &cable_direction, const Eigen::Vector3d &cable_bodyrate, const Eigen::Vector3d &attach_point_post, const Eigen::Quaterniond &payload_attitude, Eigen::Vector3d &payload_bodyrate)
{
    // compute force applied by MAV to payload at attach point    
    mav_attach_point_force_ = ComputeAttachPointForce(cable_direction, cable_bodyrate, attach_point_post, payload_attitude, payload_bodyrate);

    // compute torque applied by MAV to payload at attach point    
    mav_attach_point_torque_ = ComputeAttachPointTorque(attach_point_post, payload_attitude, mav_attach_point_force_);

}


// ComputeAttachPointTorque computes the force applied by MAV at the attach point
Eigen::Vector3d UAVCable::ComputeAttachPointForce(const Eigen::Vector3d &cable_direction, const Eigen::Vector3d &cable_bodyrate, const Eigen::Vector3d &attach_point_post, const Eigen::Quaterniond &payload_attitude, Eigen::Vector3d &payload_bodyrate)
{

    // 1. cal uav thrust force along cable direction in world frame
    Eigen::Vector3d drone_thrust_force(0,0,0);
    Eigen::Vector3d drone_thrust_force_along_cable(0,0,0);

    // compute mav thrust force in world frame
    drone_.GetThrustForce(drone_thrust_force);

    // obtain cable direction
    drone_thrust_force_along_cable= cable_direction  * cable_direction.transpose() * drone_thrust_force;

    // 2. compute attach point centrifugal acc
    Eigen::Vector3d attach_point_centri_acc(0,0,0);

    attach_point_centri_acc = drone_.TransVector3d2SkewSymMatrix(payload_bodyrate) * drone_.TransVector3d2SkewSymMatrix(payload_bodyrate) * attach_point_post;


    // 3. compute the force applied by drone to the attach point
    Eigen::Vector3d mav_attach_point_force;
    double drone_mass;
    double cable_length;
    drone_.GetMass(drone_mass);
    cable_.GetCableLength(cable_length);

    mav_attach_point_force = drone_thrust_force_along_cable - drone_mass * cable_length * cable_bodyrate.squaredNorm() * cable_bodyrate.squaredNorm() * cable_direction - drone_mass* ( (cable_direction * cable_direction.transpose()) * (payload_attitude.toRotationMatrix() * attach_point_centri_acc));

    return mav_attach_point_force;

}


// ComputeAttachPointTorque computes the torque applied by MAV at the attach point
Eigen::Vector3d UAVCable::ComputeAttachPointTorque(const Eigen::Vector3d &attach_point_post, const Eigen::Quaterniond &payload_attitude, Eigen::Vector3d &attach_point_force)
{

    Eigen::Vector3d mav_attach_point_torque(0,0,0);

    mav_attach_point_torque = drone_.TransVector3d2SkewSymMatrix(attach_point_post) * (payload_attitude.toRotationMatrix().transpose() * attach_point_force);

    return mav_attach_point_torque;
}




// ComputeControlInputs4MAV computes the control input for MAV based on the cable' status
// cable is taut  --> no tension force of cable
// cable is slack --> compute tension force of cable and caculate net force for MAV
void UAVCable::ComputeControlInputs4MAV(const Eigen::Vector3d &attach_point_acc)
{
    // 1 obtain cable's taut status
    bool cable_taut_status;

    cable_.GetCableTautStatus(cable_taut_status);


    // 2. call dynamic simulation based on status of cable' taut
    // define mav_net_input_force as the net input force for drone apart from gravity
    Eigen::Vector3d mav_net_input_force;

    if (cable_taut_status == true)
        {
            // cable is taut and there is a tension force
            Eigen::Vector3d tension_force;

            // compute tension force
            double mav_mass;
            drone_.GetMass(mav_mass);

            // to thrust force in world frame
            // note this = 0^R_i T_i where T_i is from control input
            Eigen::Quaterniond mav_attitude;

            drone_.GetAttitude(mav_attitude);

            Eigen::Matrix3d mav_rot_matrix = mav_attitude.toRotationMatrix();
            
            Eigen::Vector3d mav_thrust_force =  mav_rot_matrix * (Eigen::Vector3d::UnitZ() * mav_thrust_);

            Eigen::Vector3d cable_direction;
            
            cable_.GetCableDirection(cable_direction);

            // add
            cable_.ComputeCableTensionForce(mav_mass, mav_thrust_force, cable_direction, attach_point_acc);

            Eigen::Vector3d cable_tension_force;
            cable_.GetCableTensionForce(cable_tension_force);

            // compute net input force of drone
            // net input force = thrust force - cable tension force
            mav_net_input_force = mav_thrust_force - cable_tension_force;

            // input net input force for to drone
            drone_.InputForce(mav_net_input_force);

        }
    else
        {
          // cable is slack and there is no tension
          // mav only suffers from gravity and thrust force  
          drone_.InputThurst(mav_thrust_);
        }

        // mav rotation is independent
        drone_.InputTorque(mav_torque_);

}


// input control inputs from controller
void UAVCable::InputControllerInput(const double &mav_thrust, const Eigen::Vector3d &mav_torque)
{

    mav_thrust_ = mav_thrust;

    mav_torque_ = mav_torque;
}


// obtain parameters for payload dynamic equation
void UAVCable::GetMatrixMDiMCiMEi(Eigen::Matrix3d &m_C_i, Eigen::Matrix3d &m_D_i, Eigen::Matrix3d &m_E_i) const
{
    m_C_i = m_C_i_; 
    m_D_i = m_D_i_;
    m_E_i = m_E_i_;
}