#include "rotor_tm_sim/lib_uav_cable.hpp"

void UAVCable::CheckCollision(const Eigen::Vector3d &attachpoint_post, const Eigen::Vector3d &attachpoint_vel)
{

    // compute cable direction
    Eigen::Vector3d drone_vel;
    Eigen::Vector3d drone_post;

    drone_.getVel(drone_vel);
    drone_.getPosition(drone_post);

    cable_.ComputeCableDirection(attachpoint_post, drone_post);

    // check taut condition
    cable_.CheckTaut(attachpoint_post, drone_post, attachpoint_vel, drone_vel);
}


void UAVCable::UpdateVelCollidedUAVVel(const Eigen::Quaterniond &payload_attitude, const Eigen::Vector3d &attach_point_body_frame, const Eigen::Vector3d &payload_vel_collided, const Eigen::Vector3d &payload_bodyrate_collided)
{
    // 1. obtain drone vel
    Eigen::Vector3d drone_vel;
    drone_.getVel(drone_vel);

    // 2. obtain cable direction 
    Eigen::Vector3d xi;
    cable_.GetCableDirection(xi);

    //3. calculate drone' vel projected perpendicular to the cable direction
    Eigen::Vector3d drone_vel_proj_perpendicular_cable(0,0,0);
    drone_vel_proj_perpendicular_cable = CalVelProjPerpendicularCable(drone_vel, xi);

    // 4. compute updated drone'vel along the cable direction because of collision
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
    drone_.setVel(drone_vel_collided);
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



void UAVCable::ComputeControlInputs4MAV()
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
            Eigen::Vectir3d tension_force;

            // compute tension force
            double mav_mass;
            drone_.GetMass(mav_mass);

            // to thrust force in world frame
            // note this = 0^R_i T_i where T_i is from control input
            Eigen::Quaterniond mav_attitude;

            drone_.getAttitude(mav_attitude);

            Eigen::Matrix3d mav_rot_matrix = mav_attitude.toRotationMatrix();
            
            Eigen::Vector3d mav_thrust_force =  mav_rot_matrix * (Eigen::Vector3d::UnitZ() * mav_thrust_);

            Eigen::Vector3d cable_direction;
            
            cable_.GetCableDirection(cable_direction);

            cable_.ComputeCableTensionForce(mav_mass, mav_thrust_force, cable_direction, attach_point_acc);

            Eigen::Vector3d cable_tension_force;
            cable_.GetCableTensionForce(cable_tension_force);

            // compute net input force of drone
            // net input force = thrust force - cable tension force
            mav_net_input_force = mav_thrust_force - cable_tension_force;

            // input net input for to drone
            drone_.inputForce(mav_net_input_force)

        }
    else
        {
          // cable is slack and there is no tension
          // mav only suffers from gravity and thrust force  
          drone_.inputThurst(mav_thrust_)
        }

        // mav rotation is independent
        drone_.inputTorque(mav_torque)

}


void UAVCable::InputControllerInput(const double &mav_thrust, const Eigen::Vector3d &mav_torque)
{

    mav_thrust_ = mav_thrust;

    mav_torque_ = mav_torque;
}
