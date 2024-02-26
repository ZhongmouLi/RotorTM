#include "rotor_tm_sim/lib_cooperative.hpp"



Cooperative::Cooperative(const std::vector<Eigen::Vector3d> &v_attach_point_post_bf, const double &mav_mass, const Eigen::Matrix3d &mav_inertia, const double cable_length, const double &payload_mass, const Eigen::Matrix3d &payload_inertia, const double &step_size):  payload_(v_attach_point_post_bf, payload_mass, payload_inertia, step_size)
{
    // obtain number of robots from number of attach points
    number_robots_ = v_attach_point_post_bf.size();

    // initilise with default value v_drone_cable_(v_attach_point_post_bf.size(), UAVCable(mav_mass, mav_inertia, cable_length, step_size))
    v_drone_cable_.reserve(number_robots_);
    
    // // initilise each member of v_drone_cable_
    for (size_t i = 0; i < number_robots_; i++) {
        // v_drone_cable_.at(i) = mav_cable;
        v_drone_cable_.emplace_back(UAVCable(mav_mass, mav_inertia, cable_length, step_size) );
    }

    // std::cout<< "[----------] Cooperative constuctor v_drone_cable_.size() is" << v_drone_cable_.size() <<std::endl;

    // initilise controller inputs with size being number of robots
    v_controllers_inputs_.reserve(number_robots_);
}

// Cooperative::Cooperative(const std::vector<Eigen::Vector3d> &v_attach_point_post_bf, const double &mass, const Eigen::Matrix3d &m_inertia, const double &step_size): RigidBody(mass, m_inertia, step_size), v_attach_point_posts_body_frame_(v_attach_point_post_bf)
// {
//     // reserve size of vectors v_attach_point_posts_ and v_attach_point_vels_
//     v_attach_point_posts_.reserve(v_attach_point_posts_body_frame_.size());

//     v_attach_point_vels_.reserve(v_attach_point_posts_body_frame_.size());
// }

// void Cooperative::CheckCollision()
// {
    

// }

// check all MAVs if there are taut or not
// then modify var taut_ of cable based on the result
// void  Cooperative::CheckCollisions4MAVCables()
// {
//     // compute post and vels of attach points of payload
//     payload_.ComputeAttachPointsPostVel();    

//     std::vector<Eigen::Vector3d> v_attach_points_posts;

//     std::vector<Eigen::Vector3d> v_attach_points_vels;

//     payload_.getAttachPointsPosts(v_attach_points_posts);

//     payload_.getAttachPointsVels(v_attach_points_vels);

//     // check all UAV-cables if they have collisions or not
//     // it will update boolen var taut_ of every cable
//     for (size_t i = 0; i < number_robots; i++)
//     {
//         // obtain ith uav-cable instance from vector
//         UAVCable drone_cable = v_drone_cable_[i];

//         // obtain the corresponding attach point position and vel
//         Eigen::Vector3d attach_point_post = v_attach_points_posts[i];

//         Eigen::Vector3d attach_point_vel = v_attach_points_vels[i];

//         // check ith UAV if it has collision or not
//         drone_cable.CheckCollision(attach_point_post, attach_point_vel);

//         // 
//     }
    
// }


void  Cooperative::SetPayloadInitPost()
{
    // payload begins at origin
    payload_.SetInitialPost(Eigen::Vector3d::Zero());

    // each mav begins from attach point's above
    for (size_t i = 0; i < number_robots_; i++)
    {
        // obtain ith uav-cable instance from vector
        UAVCable& mav_cable = v_drone_cable_.at(i);

        // obtain the corresponding attach point position and vel
        Eigen::Vector3d attach_point_post;
        payload_.GetOneAttachPointPost(i, attach_point_post);

        // std::cout<<i<<"th attach point post is "<< attach_point_post.transpose()<<std::endl;

        // set mav init post that is just above attach point with distance being cable length
        mav_cable.SetMAVInitPostCableTautWithPayloadPost(attach_point_post);

        Eigen::Vector3d mav_post;
        mav_cable.mav_.GetPosition(mav_post);

        // std::cout<<i<<"th mav post is "<< mav_post.transpose()<<std::endl;
    
    };
        
}

// update vels of MAVs and payload after collsion
void Cooperative::UpdateVelsCollidedUAVsPayload()
{
    
    // 1 update payload' vel after collision
    // UpdateVelCollided contains a loop to iterate all uavcable, to modify later
    // it compues vel and bodyrate of payload after collision
    payload_.UpdateVelCollided(v_drone_cable_);

    // 
    Eigen::Vector3d payload_vel_collided{0,0,0};
    payload_.GetVel(payload_vel_collided);

    Eigen::Vector3d payload_bodyrate_collided{0,0,0};
    payload_.GetBodyrate(payload_bodyrate_collided);

    // 2. update MAVs' vels after collision

    // 2.1 compute post and vels of attach points of payload
    payload_.ComputeAttachPointsKinematics();    

    // std::vector<Eigen::Vector3d> v_attach_points_posts;

    // std::vector<Eigen::Vector3d> v_attach_points_vels;

    // payload_.GetAttachPointsPosts(v_attach_points_posts);

    // payload_.GetAttachPointsVels(v_attach_points_vels);

    Eigen::Quaterniond payload_attitude;
    payload_.GetAttitude(payload_attitude);

    // 2.2 check each mav if it has collision with its attach point
    for (size_t i = 0; i < number_robots_; i++)
    {
        // obtain ith uav-cable instance from vector
        UAVCable& drone_cable = v_drone_cable_[i];

        // obtain the corresponding attach point position and vel in world frame
        Eigen::Vector3d attach_point_post;
        payload_.GetOneAttachPointPost(i, attach_point_post);

        Eigen::Vector3d attach_point_post_bodyframe;
        payload_.GetOneAttachPointPostBodyFrame(i,attach_point_post_bodyframe);

        Eigen::Vector3d attach_point_vel;
        payload_.GetOneAttachPointVel(i, attach_point_vel);

        // check ith UAV if it has collision or not
        if(drone_cable.IsCollided(attach_point_post, attach_point_vel))
        {
            std::cout<< std::to_string(i)<<"th MAV has collision!"<<std::endl;
            drone_cable.UpdateVelCollidedMAVVel(payload_attitude, attach_point_post_bodyframe, payload_vel_collided, payload_bodyrate_collided);
        }
        else{
            std::cout<< std::to_string(i)<<"th MAV has no collision."<<std::endl;
        }
    
    };

}



void Cooperative::InputControllerInput4MAVs(const Eigen::VectorXd v_mavs_thrust, const std::vector<Eigen::Vector3d> v_mavs_torque)
{

    if (number_robots_!= static_cast<size_t>(v_mavs_thrust.size()) || number_robots_!= static_cast<size_t>(v_mavs_torque.size())  )
    {
        std::cout<< "ERROR: dim of input < num of robot"<<std::endl;
        exit(0);
    }
    
     for (size_t i = 0; i < number_robots_; i++)
     {
        // v_drone_cable_.at(i).InputControllerInput(v_mavs_thrust[i], v_mavs_torque[i] );
        v_controllers_inputs_.emplace_back(v_mavs_thrust[i], v_mavs_torque.at(i)); 
        std::cout<<"mav "<<i<< " input thrust is " << v_mavs_thrust[i] << " input torque is " << v_mavs_torque.at(i).transpose() << std::endl;
     }
    

}

// compute interaction force and torques among MAVs and payload
void Cooperative::ComputeInteractWrenches()
{
    // 
    Eigen::Vector3d mavs_net_force{0,0,0};
    Eigen::Vector3d mavs_net_torque{0,0,0};
    Eigen::Matrix3d m_C = Eigen::MatrixXd::Zero(3, 3);
    Eigen::Matrix3d m_D = Eigen::MatrixXd::Zero(3, 3);
    Eigen::Matrix3d m_E = Eigen::MatrixXd::Zero(3, 3);

    double payload_mass;
    payload_.GetMass(payload_mass);

    Eigen::Matrix3d m_mass_matrix = Eigen::MatrixXd::Identity(3, 3) * payload_mass;

    // obtain payload post, vel, attitude, bodyrate
    Eigen::Quaterniond payload_attitude;
    payload_.GetAttitude(payload_attitude);

    Eigen::Vector3d payload_bodyrate{0,0,0};
    payload_.GetBodyrate(payload_bodyrate);

    // compute posts, vels and accs of attach points
    std::cout<< "------------------Cooperative compute ComputeAttachPointsKinematics----------------------"<<std::endl;
    payload_.ComputeAttachPointsKinematics();  

    std::cout<< "------------------Cooperative compute ComputeInteractWrenches----------------------"<<std::endl;
    for (size_t i = 0; i < number_robots_; i++)
    {
        // obtain ith MAV control input
        std::pair<double, Eigen::Vector3d>& mav_control_input = v_controllers_inputs_.at(i);

        std::cout<<"[----------] Cooperative: ComputeInteractWrenches 1" << static_cast<int>(i) << "th mav thrust input is " << mav_control_input.first<<std::endl;

        // 2.1 compute net force and torque applied by all MAVs to payload

        // (1) obtain ith uav-cable instance from vector
        UAVCable& drone_cable = v_drone_cable_.at(i);

        // (1) input controller's input for ith MAV

        std::cout<<"[----------] Cooperative: ComputeInteractWrenches 2" << std::endl;        
        drone_cable.InputControllerInput(mav_control_input.first, mav_control_input.second);

        // (2) compute ith attach point post and vel that are connected to ith MAV through cable
        Eigen::Vector3d attach_point_post;
        payload_.GetOneAttachPointPost(i, attach_point_post);  

        Eigen::Vector3d attach_point_vel;
        payload_.GetOneAttachPointVel(i, attach_point_vel); 

        // (2) compute force and torque applied by ith MAV to payload at its attach point position
        std::cout<<"[----------] Cooperative: ComputeInteractWrenches 3" << std::endl; 
        drone_cable.ComputeAttachPointWrenches(attach_point_post, attach_point_vel, payload_attitude, payload_bodyrate);

        Eigen::Vector3d mav_attach_point_force{0,0,0};
        Eigen::Vector3d mav_attach_point_torque{0,0,0};

        drone_cable.GetAttachPointForce(mav_attach_point_force);
        drone_cable.GetAttachPointTorque(mav_attach_point_torque);

        // (3) allocate for all MAVs
        mavs_net_force = mavs_net_force + mav_attach_point_force;
        mavs_net_torque = mavs_net_torque + mav_attach_point_torque;

        // 2.2 compute vars needed by payload dynamic equation
 
        // (1) compute ith MAV's cable direction
        Eigen::Vector3d mav_post{0,0,0};
        drone_cable.mav_.GetPosition(mav_post);
        std::cout<<"[----------] Cooperative: ComputeInteractWrenches 4" << std::endl; 
        drone_cable.cable_.ComputeCableDirection(attach_point_post, mav_post);

        Eigen::Vector3d cable_direction{0,0,0};
        drone_cable.cable_.GetCableDirection(cable_direction);

       // (2) compute matrix m_C_i, m_D_i, m_E_i for ith MAV
        Eigen::Matrix3d m_C_i = Eigen::MatrixXd::Zero(3, 3);
        Eigen::Matrix3d m_D_i = Eigen::MatrixXd::Zero(3, 3);
        Eigen::Matrix3d m_E_i = Eigen::MatrixXd::Zero(3, 3);

        std::cout<<"[----------] Cooperative: ComputeInteractWrenches 5" << std::endl; 
        drone_cable.ComputeMatrixMDiMCiMEi(cable_direction, payload_attitude, attach_point_post);

        drone_cable.GetMatrixMDiMCiMEi(m_C_i, m_D_i, m_E_i);

        // allocate m_C_i, m_D_i, m_C_i and m_E_i
        m_C = m_C + m_C_i;
        m_D = m_D + m_D_i;
        m_E = m_E + m_E_i;

        // (3) compute mass matrix for payload
        Eigen::Matrix3d m_mass_matrix_i = Eigen::MatrixXd::Zero(3, 3);
        double mav_mass{0};
        drone_cable.mav_.GetMass(mav_mass);

        m_mass_matrix_i = mav_mass * cable_direction * cable_direction.transpose();

        // allocate 
        m_mass_matrix = m_mass_matrix + m_mass_matrix_i;

        // 2.3 compute net input for ith MAV

        // (2) comput input force and torque for ith MAV depending on if the cable is taut or slack  
        Eigen::Vector3d attach_point_acc;
        std::cout<<"[----------] Cooperative: ComputeInteractWrenches 6" << std::endl; 
        payload_.GetOneAttachPointAcc(i, attach_point_acc);
        std::cout<<"[----------] Cooperative: ComputeInteractWrenches attach_point_acc is " << attach_point_acc.transpose() << std::endl; 


        std::cout<<"[----------] Cooperative: ComputeInteractWrenches 7" << std::endl; 
        drone_cable.ComputeControlInputs4MAV(attach_point_acc);

    };

    // 3 
    payload_.InputMassMatrix(m_mass_matrix);
    payload_.InputDronesNetForces(mavs_net_force, m_D);

    // input rotational dynamic model inputs: drones' net force to the payload and term m_D
    payload_.InputDronesNetTorques(mavs_net_torque, m_C, m_E);

    // compute acc and bodyrate_acc of payload
    // payload_.ComputeAccBodyRateAcc();

}


void Cooperative::DoOneStepInt4Robots()
{

    std::cout<< "[----------] Cooperative DoOneStepInt4Robots: fuck point 1" <<std::endl;
    payload_.DoOneStepInt();

    std::cout<< "[----------] Cooperative DoOneStepInt4Robots: fuck point 2" <<std::endl;
    
    for (auto &drone_cable: v_drone_cable_)
    {
        std::cout<< "[----------] Cooperative DoOneStepInt4Robots: fuck point 3" <<std::endl;
        drone_cable.mav_.DoOneStepInt();
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
