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


void  Cooperative::SetPayloadInitPost()
{
    // payload begins at origin
    payload_.SetInitialPost(Eigen::Vector3d::Zero());

    // each mav begins from attach point's above
    for (size_t i = 0; i < number_robots_; i++)
    {
        // obtain ith uav-cable instance from vector
        UAVCable& mav_cable = v_drone_cable_[i];

        // obtain the corresponding attach point position and vel
        Eigen::Vector3d attach_point_post;
        payload_.GetOneAttachPointPost(i, attach_point_post);

        // std::cout<<i<<"th attach point post is "<< attach_point_post.transpose()<<std::endl;

        // set mav init post that is just above attach point with distance being cable length
        mav_cable.SetMAVInitPostCableTautWithAttachPointPost(attach_point_post);

        Eigen::Vector3d mav_post;
        mav_cable.mav_.GetPosition(mav_post);

        std::cout<<i<<"th mav initial post is "<< mav_post.transpose()<<std::endl;
    
    };
        
}

void  Cooperative::SetPayloadInitPost(const Eigen::Vector3d &payload_init_post)
{
    // payload begins at origin
    payload_.SetInitialPost(payload_init_post);

    // each mav begins from attach point's above
    for (size_t i = 0; i < number_robots_; i++)
    {
        // obtain ith uav-cable instance from vector
        UAVCable& mav_cable = v_drone_cable_[i];

        // obtain the corresponding attach point position and vel
        Eigen::Vector3d attach_point_post;
        payload_.GetOneAttachPointPost(i, attach_point_post);

        // std::cout<<i<<"th attach point post is "<< attach_point_post.transpose()<<std::endl;

        // set mav init post that is just above attach point with distance being cable length
        mav_cable.SetMAVInitPostCableTautWithAttachPointPost(attach_point_post);

        Eigen::Vector3d mav_post;
        mav_cable.mav_.GetPosition(mav_post);

        std::cout<<i<<"th mav initial post is "<< mav_post.transpose()<<std::endl;
    
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

void Cooperative::SetPayloadInitialAccAndBodyrateACC()
{
    double mavs_initial_forces = 0;
    double mavs_masses = 0;
    double payload_mass;
    payload_.GetMass(payload_mass);

    for (size_t i = 0; i < number_robots_; i++)
    {
        // obtain ith MAV control input
        std::pair<double, Eigen::Vector3d>& mav_control_input = v_controllers_inputs_.at(i);

        mavs_initial_forces = mavs_initial_forces + mav_control_input.first;

        UAVCable& drone_cable = v_drone_cable_.at(i);

        double mav_mass;
        drone_cable.mav_.GetMass(mav_mass);
        mavs_masses = mavs_masses + mav_mass;
    }

    double net_initial_vertical_acc = (mavs_initial_forces - (mavs_masses + payload_mass) * gravity_)/((mavs_masses + payload_mass));

    Eigen::Vector3d payload_intial_acc{0,0,net_initial_vertical_acc};

    payload_.SetInitialAccBodyRateAcc(payload_intial_acc);
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
    if (!intial_payload_acc_set)
    {
        SetPayloadInitialAccAndBodyrateACC();
        intial_payload_acc_set = true;
        std::cout<<"[----------] Cooperative: set initial posts for payload once"<<std::endl;
    }
    
    // for test
    Eigen::Vector3d test_payload_acc{0,0,0};
    payload_.GetAcc(test_payload_acc);
    std::cout<<"[----------] Cooperative: payload_acc "<<test_payload_acc.transpose()<<std::endl;

    payload_.ComputeAttachPointsKinematics();  

    std::cout<< "------------------Cooperative compute ComputeInteractWrenches----------------------"<<std::endl;
    for (size_t i = 0; i < number_robots_; i++)
    {
        // 
        // std::cout<<"[----------] Cooperative: ComputeInteractWrenches " << static_cast<int>(i) << "the mav"<<std::endl;

        // obtain ith MAV control input
        // std::pair<double, Eigen::Vector3d> &mav_control_input = v_controllers_inputs_.at(i);

        // remove ith MAV input vector to save space for next input
        std::pair<double, Eigen::Vector3d> mav_control_input = v_controllers_inputs_.front();
        v_controllers_inputs_.erase(v_controllers_inputs_.begin());


        // std::cout<<"[----------] Cooperative: ComputeInteractWrenches " << static_cast<int>(i) << "th mav thrust input is " << mav_control_input.first<<std::endl;

        // 2.1 compute net force and torque applied by all MAVs to payload

        // (1) obtain ith uav-cable instance from vector
        UAVCable& drone_cable = v_drone_cable_.at(i);

        // (1) input controller's input for ith MAV

        // std::cout<<"[----------] Cooperative: ComputeInteractWrenches 2" << std::endl;        
        drone_cable.InputControllerInput(mav_control_input.first, mav_control_input.second);

        // (2) compute ith attach point post in body frame and world frame, and vel that are connected to ith MAV through cable
        Eigen::Vector3d attach_point_post;
        payload_.GetOneAttachPointPost(i, attach_point_post);  

        Eigen::Vector3d attach_point_vel;
        payload_.GetOneAttachPointVel(i, attach_point_vel); 

        Eigen::Vector3d attach_point_post_bf;
        payload_.GetOneAttachPointPostBodyFrame(i, attach_point_post_bf);

        // (2) compute force and torque applied by ith MAV to payload at its attach point position
        // std::cout<<"[----------] Cooperative: ComputeInteractWrenches 3" << std::endl; 
        drone_cable.ComputeAttachPointWrenches(attach_point_post_bf, attach_point_post, attach_point_vel, payload_attitude, payload_bodyrate);

        Eigen::Vector3d mav_attach_point_force{0,0,0};
        Eigen::Vector3d mav_attach_point_torque{0,0,0};

        drone_cable.GetAttachPointForce(mav_attach_point_force);
        drone_cable.GetAttachPointTorque(mav_attach_point_torque);

        std::cout<<i<<"th mav attach point torque is " << mav_attach_point_torque.transpose() << std::endl;
        // (3) allocate for all MAVs
        mavs_net_force = mavs_net_force + mav_attach_point_force;
        mavs_net_torque = mavs_net_torque + mav_attach_point_torque;

        // 2.2 compute vars needed by payload dynamic equation
 
        // (1) compute ith MAV's cable direction
        Eigen::Vector3d mav_post{0,0,0};
        drone_cable.mav_.GetPosition(mav_post);
        // std::cout<<"[----------] Cooperative: ComputeInteractWrenches 4" << std::endl; 
        drone_cable.cable_.ComputeCableDirection(attach_point_post, mav_post);

        Eigen::Vector3d cable_direction{0,0,0};
        drone_cable.cable_.GetCableDirection(cable_direction);

       // (2) compute matrix m_C_i, m_D_i, m_E_i for ith MAV
        Eigen::Matrix3d m_C_i = Eigen::MatrixXd::Zero(3, 3);
        Eigen::Matrix3d m_D_i = Eigen::MatrixXd::Zero(3, 3);
        Eigen::Matrix3d m_E_i = Eigen::MatrixXd::Zero(3, 3);

        // std::cout<<"[----------] Cooperative: ComputeInteractWrenches 5" << std::endl; 
        drone_cable.ComputeMatrixMDiMCiMEi(cable_direction, payload_attitude, attach_point_post_bf);

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
        // std::cout<<"[----------] Cooperative: ComputeInteractWrenches 6" << std::endl; 
        payload_.GetOneAttachPointAcc(i, attach_point_acc);
        // std::cout<<"[----------] Cooperative: ComputeInteractWrenches attach_point_acc is " << attach_point_acc.transpose() << std::endl; 


        // std::cout<<"[----------] Cooperative: ComputeInteractWrenches input for "<< i <<"th MAV" << std::endl; 
        drone_cable.ComputeControlInputs4MAV(attach_point_acc);

    };

    // 3 
    payload_.InputMassMatrix(m_mass_matrix);
    payload_.InputDronesNetForces(mavs_net_force, m_D);

    // input rotational dynamic model inputs: drones' net force to the payload and term m_D
    payload_.InputDronesNetTorques(mavs_net_torque, m_C, m_E);

    // compute acc and bodyrate_acc of payload
    payload_.ComputeAccBodyRateAcc();
    // for test
    Eigen::Vector3d test_payload_acc2{0,0,0};
    payload_.GetAcc(test_payload_acc2);
    std::cout<<"[----------] Cooperative: payload_acc "<<test_payload_acc2.transpose()<<std::endl;

    Eigen::Vector3d pd_post;
    payload_.GetPosition(pd_post);
    std::cout<<"[----------] payload post is "<< pd_post.transpose()<<std::endl;
}


void Cooperative::DoOneStepInt4Robots()
{

    Eigen::Vector3d pd_post_before;
    payload_.GetPosition(pd_post_before);
    // std::cout<<"[----------] DoOneStepInt4Robots post before is "<< pd_post_before.transpose()<<std::endl;

    // std::cout<< "[----------] Cooperative DoOneStepInt4Robots: fuck point 1" <<std::endl;
    payload_.DoPayloadOneStepInt();

    Eigen::Vector3d pd_post_after;
    payload_.GetPosition(pd_post_after);
    // std::cout<<"[----------] DoOneStepInt4Robots post after is "<< pd_post_after.transpose()<<std::endl;


    // std::cout<< "[----------] Cooperative DoOneStepInt4Robots: fuck point 2" <<std::endl;
    
    // for (auto &drone_cable: v_drone_cable_)
    // {
    //     // std::cout<< "[----------] Cooperative DoOneStepInt4Robots: fuck point 3" <<std::endl;
    //     drone_cable.mav_.DoOneStepInt();
    //     // Eigen::Vector3d mav_post;
    //     // drone_cable.mav_.GetPosition(mav_post);

    //     // std::cout<<"mav initial post is "<< mav_post.transpose()<<std::endl;
    // }

        Eigen::Vector3d mav_post_bf;
        v_drone_cable_[0].mav_.GetPosition(mav_post_bf);
        Eigen::Quaterniond mav_attitude_bf;
        v_drone_cable_[0].mav_.GetAttitude(mav_attitude_bf);


        // std::cout<<0<< "th "<<"mav post before is "<< mav_post_bf.transpose()<<std::endl;
        // std::cout<<0<< "th "<<"mav att before is "<< mav_attitude_bf<<std::endl;
        // std::cout <<v_drone_cable_.capacity() << number_robots_ <<std::endl;

     for (size_t i = 0; i < number_robots_; i++)
    {
        v_drone_cable_.at(i).mav_.DoOneStepInt();

        // if(i==0)
        {
        Eigen::Vector3d mav_post;
        v_drone_cable_[i].mav_.GetPosition(mav_post);
        Eigen::Quaterniond mav_attitude;
        v_drone_cable_[i].mav_.GetAttitude(mav_attitude);


        // std::cout<<i<< "th "<<"mav post after is "<< mav_post.transpose()<<std::endl;
        // std::cout<<i<< "th "<<"mav att after is "<< mav_attitude<<std::endl;

        Eigen::Vector3d pd_post;
        payload_.GetPosition(pd_post);
        // std::cout<<"payload post is "<< pd_post.transpose()<<std::endl;
        };
    };

    // clear control 

        Eigen::Vector3d test_payload_acc2{0,0,0};
    payload_.GetAcc(test_payload_acc2);
    std::cout<<"[----------] Cooperative: payload_acc last"<<test_payload_acc2.transpose()<<std::endl;
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
