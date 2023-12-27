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

// check all MAVs if there are taut or not
// then modify var taut_ of cable based on the result
void  Cooperative::CheckCollisions4MAVCables()
{
    // compute post and vels of attach points of payload
    payload_.ComputeAttachPointsPostVel();    

    std::vector<Eigen::Vector3d> v_attach_points_posts;

    std::vector<Eigen::Vector3d> v_attach_points_vels;

    payload_.getAttachPointsPosts(v_attach_points_posts);

    payload_.getAttachPointsVels(v_attach_points_vels);

    // check all UAV-cables if they have collisions or not
    // it will update boolen var taut_ of every cable
    for (size_t i = 0; i < number_robots; i++)
    {
        // obtain ith uav-cable instance from vector
        UAVCable drone_cable = v_drone_cable_[i];

        // obtain the corresponding attach point position and vel
        Eigen::Vector3d attach_point_post = v_attach_points_posts[i];

        Eigen::Vector3d attach_point_vel = v_attach_points_vels[i];

        // check ith UAV if it has collision or not
        drone_cable.CheckCollision(attach_point_post, attach_point_vel);

        // 
    }
    
}

// update vels of MAVs and payload after collsion
void Cooperative::UpdateVelsCollidedUAVsPayload()
{
    
    // update MAVs' vels after collision
    for (size_t i = 0; i < number_robots; i++)
    {
        // obtain ith uav-cable instance from vector
        UAVCable drone_cable = v_drone_cable_[i];

        drone_cable.UpdateVelCollidedUAVVel(const Eigen::Quaterniond &payload_attitude, const Eigen::Vector3d &attach_point_body_frame, const Eigen::Vector3d &payload_vel_collided, const Eigen::Vector3d &payload_bodyrate_collided);

    };

    // update payload' vel after collision
    // UpdateVelCollided contains a loop to iterate all uavcable, to modify later
    payload_.UpdateVelCollided(v_drone_cable_);

    // to add updated positions of attach points

}



// compute interaction force and torques among MAVs and payload
void Cooperative::ComputeInteractWrenches()
{
    // 
    Eigen::Vector3d mavs_net_force;
    Eigen::Vector3d mavs_net_torque;
    Eigen::Matrix3d m_C = Eigen::MatrixXd::Zero(3, 3);
    Eigen::Matrix3d m_D = Eigen::MatrixXd::Zero(3, 3);
    Eigen::Matrix3d m_E = Eigen::MatrixXd::Zero(3, 3);

    double payload_mass;
    payload_.GetMass(payload_mass);

    Eigen::Matrix3d m_mass_matrix = Eigen::MatrixXd::Identity(3, 3) * payload_mass;

    Eigen::quaternion payload_attitude;
    
    payload_.getAttitude(payload_attitude);

    for (size_t i = 0; i < number_robots; i++)
    {
        // obtain ith MAV control input
        std::pair<double, Eigen::Vector3d> mav_control_input = v_controllers_inputs_[i];

        // 2.1 compute net force and torque applied by all MAVs to payload

        // (1) obtain ith uav-cable instance from vector
        UAVCable drone_cable = v_drone_cable_[i];

        // (2) compute force and torque applied by ith MAV to payload at its attach point position
        drone_cable.ComputeAttachPointWrenches();

        Eigen::Vector3d mav_attach_point_force{0,0,0};
        Eigen::Vector3d mav_attach_point_torque{0,0,0};

        drone_cable.GetAttachPointForce(mav_attach_point_force);
        drone_cable.GetAttachPointTorque(mav_attach_point_torque);

        // (3) allocate for all MAVs
        mavs_net_force = mavs_net_force + mav_attach_point_force;
        mavs_net_torque = mavs_net_torque + mav_attach_point_torque;

        // 2.2 compute vars needed by payload dynamic equation
 
        // (1) compute ith MAV's cable direction
        Eigen::Vector3d cable_direction{0,0,0};
        drone_cable.cable_.ComputeCableDirection();
        drone_cable.cable_.GetCableDirection(cable_direction);

        // (2) compute ith attach point post that is connected to ith MAV through cable
        Eigen::Vector3d attach_point_post;
        payload_.ComputeAttachPointAccs(i, attach_point_post);  

       // (3) compute matrix m_C_i, m_D_i, m_E_i for ith MAV
        Eigen::Matrix3d m_C_i = Eigen::MatrixXd::Zero(3, 3);
        Eigen::Matrix3d m_D_i = Eigen::MatrixXd::Zero(3, 3);
        Eigen::Matrix3d m_E_i = Eigen::MatrixXd::Zero(3, 3);

        drone_cable.ComputeMatrixMDiMCiMEi(cable_direction, payload_attitude, attach_point_post);

        drone_cable.GetMatrixMDiMCiMEi(m_C_i, m_D_i, m_E_i);

        // allocate m_C_i, m_D_i, m_C_i and m_E_i
        m_C = m_C + m_C_i;
        m_D = m_D + m_D_i;
        m_E = m_E + m_E_i;

        // (2) compute mass matrix for payload
        Eigen::Matrix3d m_mass_matrix_i = Eigen::MatrixXd::Zero(3, 3);
        double mav_mass{0};
        drone_cable.drone_.GetMass(mav_mass);

        m_mass_matrix_i = mav_mass * cable_direction * cable_direction.transpose();

        // allocate 
        m_mass_matrix = m_mass_matrix + m_mass_matrix_i;

        // 2.3 compute net input for ith MAV
        // (1) input controller's input for ith MAV
        drone_cable.InputControllerInput(mav_control_input.first, mav_control_input.second);

        // (2) comput input force and torque for ith MAV depending on if the cable is taut or slack
        drone_cable.ComputeControlInputs4MAV()

    };

    // 3 
    payload_.InputMassMatrix(m_mass_matrix);
    payload_.InputDronesNetForces(mavs_net_force, m_D);

    // input rotational dynamic model inputs: drones' net force to the payload and term m_D
    payload_.InputDronesNetTorques(mavs_net_torque, m_C, m_E);

}


void Cooperative::DoOneStepInt4Robots()
{
    payload_.doOneStepInt();

    for (auto drone_cable in v_drone_cable_)
    {
        drone_cable_.drone_.doOneStepInt();
    }
}





// Eigen::Vector3d Cooperative::CalVelProjPerpendicularCable(const Eigen::Vector3d drone_vel, const Eigen::Vector3d &cable_direction)
// {
//     // var of drone vel projected  along cable
//     Eigen::Vector3d drone_vel_projection_along_cable(0,0,0);    

//     // var of drone vel' projection that is perpendicular to cable
//     Eigen::Vector3d drone_vel_projection_perpendicular_cable(0,0,0);

//     // similar rule can be found at Eq 30
//     drone_vel_projection_along_cable = cable_direction * cable_direction.transpose() *  drone_vel;

//     // similar rule can be found at Eq 30
//     drone_vel_projection_perpendicular_cable = drone_vel - drone_vel_projection_along_cable;

//     return drone_vel_projection_perpendicular_cable;

// }
