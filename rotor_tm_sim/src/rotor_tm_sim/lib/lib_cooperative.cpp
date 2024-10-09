#include "rotor_tm_sim/lib_cooperative.hpp"


Cooperative::Cooperative(const std::shared_ptr<Payload>& ptr_payload_, const std::vector<std::shared_ptr<Joint>>&v_ptr_joints, const std::vector<std::shared_ptr<UAVCable>>& v_ptr_uavcables_):ptr_payload_(ptr_payload_), v_ptr_joints_(v_ptr_joints), v_ptr_uavcables_(v_ptr_uavcables_)
{
    if (v_ptr_joints.size() == v_ptr_uavcables_.size())
    {
        number_robots_ = v_ptr_joints.size();

        v_controllers_inputs_.resize(number_robots_);
    }
    else
    {
        std::cout << "error in input"<<std::endl;
    }
    
}

Cooperative::Cooperative(const std::shared_ptr<Payload>& ptr_payload, const std::shared_ptr<Joint> &ptr_joint, const std::shared_ptr<UAVCable>& ptr_uavcable):ptr_payload_(ptr_payload)
{
    v_ptr_joints_ = {ptr_joint};
    v_ptr_uavcables_ = {ptr_uavcable};
    number_robots_ =1;
    v_controllers_inputs_.resize(number_robots_);

}

void  Cooperative::SetPayloadInitPost()
{

    // set initial post for payload and mavs
    Eigen::Vector3d payload_init_post{0,0,0};
    // set initial post for payload
    ptr_payload_->SetInitialPost(payload_init_post);

    // set initial post for each mav-cable
    for (size_t i = 0; i < number_robots_; i++)
    {
        // get ith join post in body frame with the vector of shared pointer of joint
        Eigen::Vector3d attach_point_post_body_frame;
        attach_point_post_body_frame = v_ptr_joints_.at(i)->post_body_frame();

        // set initial post for each mav-cable
        v_ptr_uavcables_[i]->SetMAVInitPostCableTautWithAttachPointPost(payload_init_post + attach_point_post_body_frame);
    }
        
}

void  Cooperative::SetPayloadInitPost(const Eigen::Vector3d &payload_init_post)
{
    // payload begins at origin
    ptr_payload_->SetInitialPost(payload_init_post);

    ptr_payload_->SetJointInitPostBasedOnPayload();
    
    // set initial post for each mav-cable
    for (size_t i = 0; i < number_robots_; i++)
    {
        // get ith join post in body frame with the vector of shared pointer of joint
        Eigen::Vector3d attach_point_post_body_frame;
        attach_point_post_body_frame = v_ptr_joints_.at(i)->post_body_frame();

        // set initial post for each mav-cable
        v_ptr_uavcables_[i]->SetMAVInitPostCableTautWithAttachPointPost(payload_init_post + attach_point_post_body_frame);

        // set initial post for each joint
        
    }        
}


void Cooperative::SetPayloadInitPost(const Eigen::Vector3d &payload_init_post, const double& tilt_angle)
{
    // payload begins at origin
    ptr_payload_->SetInitialPost(payload_init_post);

    ptr_payload_->SetJointInitPostBasedOnPayload();
    
    // set initial post for each mav-cable
    for (size_t i = 0; i < number_robots_; i++)
    {
        // get ith join post in body frame with the vector of shared pointer of joint
        Eigen::Vector3d attach_point_post_body_frame;
        attach_point_post_body_frame = v_ptr_joints_.at(i)->post_body_frame();

        // set initial post for each mav-cable
        v_ptr_uavcables_[i]->SetMAVInitPostCableTautWithAttachPointPost(payload_init_post + attach_point_post_body_frame);

        // set initial post for each joint
        
    }        
}


void Cooperative::UpdateJointAndCableStatus()
{
    ptr_payload_->ComputeJointKinematics();
    
    for (size_t i = 0; i < number_robots_; i++)
    {
        v_ptr_uavcables_.at(i)->UpdateCableTautStatus();
    }
}




// void Cooperative::UpdateVelsCollidedUAVsPayload()
// {
//     std::vector<bool> inelastic_collision_flag(number_robots_, false);
//     for (size_t i = 0; i < number_robots_; i++)
//     {
//         v_ptr_uavcables_[i]->CheckInelasticCollision();
//         inelastic_collision_flag[i] = v_ptr_uavcables_[i]->inelasticCollisionStauts();
//     }

//     while (std::any_of(inelastic_collision_flag.begin(), inelastic_collision_flag.end(), [](bool flag) { return flag; }))
//     {
//         auto before_collision_flag = inelastic_collision_flag;
        
//         RigidbodyQuadInelasticCableCollision(inelastic_collision_flag);
        
//         for (size_t i = 0; i < number_robots_; i++)
//         {
//             v_ptr_uavcables_[i]->CheckInelasticCollision();
//             inelastic_collision_flag[i] = v_ptr_uavcables_[i]->inelasticCollisionStauts();
//         }

//         auto after_collision_flag = inelastic_collision_flag;

//         if (AnyNewCollisions(before_collision_flag, after_collision_flag))
//         {
//             UpdateCollisionFlags(inelastic_collision_flag, before_collision_flag, after_collision_flag);
//         }
//         else
//         {
//             UpdateRobotPositions();
//             break;
//         }
//     }
// }



// void Cooperative::RigidbodyQuadInelasticCableCollision(std::vector<bool>& collision_flags)
// {
//     ptr_payload_->UpdateVelCollided();

//     for (size_t i = 0; i < number_robots_; i++)
//     {
//         if (collision_flags[i])
//         {
//             v_ptr_uavcables_[i]->UpdateMAVVelCollided(
//                 ptr_payload_->pose().att,
//                 ptr_payload_->vels().linear_vel,
//                 ptr_payload_->vels().bodyrate
//             );
//         }
//     }
// }

// bool Cooperative::AnyNewCollisions(const std::vector<bool>& before, const std::vector<bool>& after)
// {
//     for (size_t i = 0; i < number_robots_; i++)
//     {
//         if (after[i] && !before[i])
//         {
//             return true;
//         }
//     }
//     return false;
// }

// void Cooperative::UpdateCollisionFlags(std::vector<bool>& current, const std::vector<bool>& before, const std::vector<bool>& after)
// {
//     for (size_t i = 0; i < number_robots_; i++)
//     {
//         current[i] = after[i] || before[i];
//     }
// }

void Cooperative::UpdateRobotPositions()
{
    Eigen::Vector3d payload_position = ptr_payload_->pose().post;
    Eigen::Matrix3d payload_rotation = ptr_payload_->pose().att.toRotationMatrix();

    for (size_t i = 0; i < number_robots_; i++)
    {
        Eigen::Vector3d attach_point = payload_position + payload_rotation * v_ptr_joints_[i]->post_body_frame();

        v_ptr_uavcables_[i]->UpdateCableTautStatus();
        Eigen::Vector3d cable_direction = v_ptr_uavcables_[i]->cable_.direction();
        
        Eigen::Vector3d new_position = attach_point - v_ptr_uavcables_[i]->cable_.length() * cable_direction;
        v_ptr_uavcables_[i]->mav_.SetPost(new_position);
    }
}

// update vels of MAVs and payload after collsion
void Cooperative::UpdateVelsCollidedUAVsPayload()
{
  // drone_cable.InputControllerInput(mav_control_input.first, mav_control_input.second);
        // ROS_INFO_STREAM("FUUUUCK point 21");
        // step 2 update kinematics of joints
        


        // step 3 check collision between mavs and payload at current iteration
        for (size_t i = 0; i < number_robots_; i++)
            {
                v_ptr_uavcables_[i]->CheckInelasticCollision();
            }

        // save collision status to v_flags_inelastic_collision_status
        std::vector<bool> v_flags_inelastic_collision_status(4,false);
        for (size_t i = 0; i < number_robots_; i++)
            {
                v_flags_inelastic_collision_status[i] = v_ptr_uavcables_[i]->inelasticCollisionStauts();
                //  std::cout<<std::string(2, ' ')<<"ith element of v_flags_inelastic_collision_status is"<<v_flags_inelastic_collision_status[i]<<std::endl;
            }


        /*Avoid any collision*/
        // for (size_t i = 0; i < number_robots_; i++)
        //     {
        //         v_flags_inelastic_collision_status[i] = false;
               
        //     }


        // step 3 distribute vels among collided mavs and payload until there is no collision
        while (std::any_of(v_flags_inelastic_collision_status.begin(), v_flags_inelastic_collision_status.end(), [](const bool& collision_flag) { return collision_flag; })) 
        {

            //  ROS_DEBUG_STREAM("FUUUUCK point 6");    
            // 3.1 define a vector of 4 mav-cable inelatic collision status
            std::vector<bool> v_mavs_inelastic_status_before_collision(4,false);
            //assign v_mavs_inelastic_collision with value of inelastic_collision_ of each mav-cable
            for (size_t i = 0; i < number_robots_; i++)
            {
                v_mavs_inelastic_status_before_collision[i] = v_ptr_uavcables_[i]->inelasticCollisionStauts();
            }

            // 3.2 check collision between mavs and payload
            // check each mav if it has collision with its attach point
            // update vels of payload because of collision
            ptr_payload_->UpdateVelCollided();

            // 3.3 update vel of mav if collision happend
            for (size_t i = 0; i < number_robots_; i++)
            {
                // obtain ith uav-cable instance from vector
                std::shared_ptr<UAVCable>& ptr_uavcable = v_ptr_uavcables_.at(i);

                // check if there is collision between mav and payload
                // UpdateMAVVelCollided(const Eigen::Quaterniond &payload_attitude, const Eigen::Vector3d &payload_vel_collided, const Eigen::Vector3d &payload_bodyrate_collided)
                ptr_uavcable->UpdateMAVVelCollided(ptr_payload_->pose().att, ptr_payload_->vels().linear_vel, ptr_payload_->vels().bodyrate);
            };

            // 3.4 save inelastic collision status of each mav-cable after collision
            std::vector<bool> v_mavs_inelastic_status_after_collision(4,false);
            //assign v_mavs_inelastic_status_after_collision with value of inelastic_collision_ of each mav-cable
            for (size_t i = 0; i < number_robots_; i++)
            {
                v_mavs_inelastic_status_after_collision[i] = v_ptr_uavcables_[i]->inelasticCollisionStauts();
            }

            // 3.5 check and find "new collision":
            // that was not collisioned before
            // but will get collision after  
            std::vector<bool> v_mavs_inelastic_status_diff(4,false);

            for (size_t i = 0; i < v_mavs_inelastic_status_diff.size(); i++) 
                {
                    v_mavs_inelastic_status_diff[i] = (v_mavs_inelastic_status_after_collision[i] && !v_mavs_inelastic_status_before_collision[i]);

                    // std::cout<<i<<"th element is"<<v_mavs_inelastic_status_diff[i]<<std::endl; 
                };

            bool condition_exist_new_collision = std::any_of(v_mavs_inelastic_status_diff.begin(),
                                     v_mavs_inelastic_status_diff.end(),
                                     [](bool val) { return val; });

            // 3.6
            if (condition_exist_new_collision)
            {
                for (size_t i = 0; i < v_flags_inelastic_collision_status.size(); i++) 
                    {
                        v_flags_inelastic_collision_status[i] = (v_mavs_inelastic_status_after_collision[i] ||v_mavs_inelastic_status_before_collision[i]);
                         
                    };
            }
            else
            {
                 std::cout<<std::string(2, ' ')<<"no new collision"<<std::endl;

                // update post and vels of joint
                ptr_payload_->ComputeJointKinematics();

                v_flags_inelastic_collision_status = v_mavs_inelastic_status_after_collision;

                for (size_t i = 0; i < number_robots_; i++)
                    {
                        // debug here
                        v_ptr_uavcables_[i]->CheckInelasticCollision();
                    }

            }
     
        }   

        UpdateRobotPositions();
}





void Cooperative::InputControllerInput4MAVs(const std::vector<double> v_mavs_thrust, const std::vector<Eigen::Vector3d> v_mavs_torque)
{

    
     for (size_t i = 0; i < number_robots_; i++)
     {

        // v_controllers_inputs_.emplace_back(v_mavs_thrust.at(i), v_mavs_torque.at(i));

        v_controllers_inputs_.at(i) = {v_mavs_thrust.at(i), v_mavs_torque.at(i)};
        // v_drone_cable_.at(i).InputControllerInput(v_mavs_thrust[i], v_mavs_torque[i] );
        // v_drone_cable_.at(i) = (v_mavs_thrust[i], v_mavs_torque[i] );

        // // v_controllers_inputs_.emplace_back(v_mavs_thrust[i], v_mavs_torque.at(i)); 
        // std::pair<double, Eigen::Vector3d> mav_input(v_mavs_thrust[i], v_mavs_torque.at(i));

        // v_controllers_inputs_.at(i) = mav_input;
        //  std::cout<<std::string(2, ' ')<<"mav "<<i<< " input thrust is " << v_mavs_thrust[i] << " input torque is " << v_mavs_torque.at(i).transpose() << std::endl;

     }
    
     for (size_t i = 0; i < number_robots_; i++)
     {

        //  std::cout<<std::string(2, ' ')<<"check" << std::endl;
        std::cout<<std::string(2, ' ')<<"mav "<<i<< " input thrust is " << v_controllers_inputs_.at(i).thrust << " input torque is " <<  v_controllers_inputs_.at(i).torque.transpose() << std::endl;

     }

    // set accs of mavs and payload for 1st iteration
    if (flag_first_iteration_ == true)
    {
        double total_mass = ptr_payload_->mass();
        double total_force = 0;

        for (size_t i = 0; i < number_robots_; i++)
        {

              total_mass = total_mass + v_ptr_uavcables_.at(i)->mav_.mass();

              total_force = total_force + v_mavs_thrust.at(i);
        };

        // compute intial accleration with total mass and total force
        Eigen::Vector3d total_acc = total_force * Eigen::Vector3d(0, 0, 1) /total_mass - ptr_payload_->gravity_ * Eigen::Vector3d(0, 0, 1);

        //  std::cout<<std::string(2, ' ')<<"total_mass is "<<total_mass<<std::endl;
        //  std::cout<<std::string(2, ' ')<<"total_force is "<<total_force<<std::endl;
        //  std::cout<<std::string(2, ' ')<<"total_acc is "<<total_acc.transpose()<<std::endl;

        // set accs of mavs
        for (size_t i = 0; i < number_robots_; i++)
        {

               v_ptr_uavcables_[i]->mav_.SetLinearAcc(total_acc);
        };

        // set acc of payload
        ptr_payload_->SetLinearAcc(total_acc);


        // change flag to be false then next iteration will never enter this block
        flag_first_iteration_ =  false;
    }
    

}



// compute interaction force and torques among MAVs and payload
void Cooperative::ComputeInteractWrenches()
{

     std::cout<<std::string(2, ' ')<<"Entre Cooperative::ComputeInteractWrenches() "<<std::endl;

    for (size_t i = 0; i < number_robots_; i++)
        {
            v_ptr_uavcables_.at(i)->InputControllerInput(v_controllers_inputs_.at(i).thrust, v_controllers_inputs_.at(i).torque);

            std::cout<<std::string(2, ' ')<<"mav "<<i<< " input thrust is " << v_controllers_inputs_.at(i).thrust << " input torque is " << v_controllers_inputs_.at(i).torque.transpose() << std::endl;
            
        }

        Eigen::Matrix3d sum_m_C_i = Eigen::Matrix3d::Zero();
        Eigen::Matrix3d sum_m_D_i = Eigen::Matrix3d::Zero();
        Eigen::Matrix3d sum_m_E_i = Eigen::Matrix3d::Zero();
        Eigen::Matrix3d m_mass_matrix = Eigen::Matrix3d::Identity() * ptr_payload_->mass();

        for (size_t i = 0; i < number_robots_; i++)
            {
               // compute MDi, MCi and MEi for each mav     
                v_ptr_uavcables_.at(i)->ComputeMatrixMDiMCiMEi(ptr_payload_->pose().att);

                // accumulate MDi, MCi and MEi
                sum_m_C_i = sum_m_C_i +  v_ptr_uavcables_[i]->m_C_i();
                sum_m_D_i = sum_m_D_i +  v_ptr_uavcables_[i]->m_D_i();
                sum_m_E_i = sum_m_E_i +  v_ptr_uavcables_[i]->m_E_i();

                // accumlate
                m_mass_matrix = m_mass_matrix + v_ptr_uavcables_[i]->m_mass_matrix();
            }        
        
        // assigen to interaction_parameters
        interaction_parameters_.m_C = sum_m_C_i;
        interaction_parameters_.m_D = sum_m_D_i;
        interaction_parameters_.m_E = sum_m_E_i;
        interaction_parameters_.m_mass_matrix = m_mass_matrix;

        ptr_payload_->InputPayloadInteractPara(interaction_parameters_);


        for (size_t i = 0; i < number_robots_; i++)
            {  
                // std::cout<< "Entre UAVCable ComputeInteractionWrenches "<<std::endl; 
                v_ptr_uavcables_.at(i)->ComputeInteractionWrenches(ptr_payload_->pose().att, ptr_payload_->vels().bodyrate);


                auto mav_wrench = v_ptr_uavcables_.at(i)->attach_point_wrench();

                std::cout<<std::string(2, ' ')<<i <<"th mav_wrench's force to payload is " << mav_wrench.force.transpose()<<std::endl;
                std::cout<<std::string(2, ' ')<<i <<"th mav_wrench's torque to payload is " << mav_wrench.torque.transpose()<<std::endl;
                net_mavs_wrench_to_payload_ = net_mavs_wrench_to_payload_ + mav_wrench;
            }           

        std::cout<<std::string(2, ' ')<< "net_mavs_wrench_to_payload_ force" << net_mavs_wrench_to_payload_.force.transpose()<<std::endl;

        std::cout<<std::string(2, ' ')<< "net_mavs_wrench_to_payload_ torque" << net_mavs_wrench_to_payload_.torque.transpose()<<std::endl;

        std::cout<<std::string(2, ' ')<<"Leave Cooperative::ComputeInteractWrenches()"<<std::endl;
}


void Cooperative::DoOneStepInt4Robots()
{

        std::cout<<std::string(2, ' ')<<"Entre Cooperative::DoOneStepInt4Robots()"<<std::endl;    
        std::cout<<std::string(2, ' ')<<"integration for payload"<<std::endl;
        ptr_payload_->InputDronesNetWrenches(net_mavs_wrench_to_payload_);


        ptr_payload_->DoOneStepInt();

        // std::cout << std::fixed << std::setprecision(6);

        std::cout<<std::string(2, ' ')<<"payload post is " << std::fixed << std::setprecision(5) <<  ptr_payload_->pose().post.transpose()<<std::endl;

        auto payload_euler = ptr_payload_->pose().att.toRotationMatrix().eulerAngles(2, 1, 0);
        // std::cout<<std::string(2, ' ')<<"payload Euler angles are " << std::fixed << std::setprecision(5) <<  ptr_payload_->pose().att.coeffs().transpose()<<std::endl;
        std::cout<<std::string(2, ' ')<<"payload Euler angles are " << std::fixed << std::setprecision(5) << "roll " << payload_euler[2]<< "pitch " <<payload_euler[1] << "yaw" << payload_euler[0] <<std::endl;

        std::cout<<std::string(2, ' ')<<"integration for mav"<<std::endl;
        for (size_t i = 0; i < number_robots_; i++)
        {
               // compute MDi, MCi and MEi for each mav     
                v_ptr_uavcables_.at(i)->ComputeNetWrenchApplied2MAV();
                v_ptr_uavcables_.at(i)->DoOneStepInt();
                std::cout<<std::string(2, ' ')<< i<<" th mav post is " << std::fixed << std::setprecision(5) <<  v_ptr_uavcables_.at(i)->mav_.pose().post.transpose()<<std::endl;
                // std::cout<<std::string(2, ' ')<<"mav att is " << std::fixed << std::setprecision(5) <<  v_ptr_uavcables_.at(i)->mav_.pose().att.coeffs().transpose()<<std::endl;
                auto mav_euler = v_ptr_uavcables_.at(i)->mav_.pose().att.toRotationMatrix().eulerAngles(2, 1, 0);
                std::cout<<std::string(2, ' ')<< i<<" th mav Euler angles are " << std::fixed << std::setprecision(5) << "roll " << mav_euler[2]<< "pitch " <<mav_euler[1] << "yaw" << mav_euler[0] <<std::endl;

                auto mav_bodyrate = v_ptr_uavcables_.at(i)->mav_.vels().bodyrate.transpose();
                std::cout<<std::string(2, ' ')<< i<<" th mavEuler bodyrate is " << mav_bodyrate <<std::endl;

                auto mav_angular_acc = v_ptr_uavcables_.at(i)->mav_.accs().angular_acc.transpose();
                std::cout<<std::string(2, ' ')<< i<<" th mavEuler angular acc is " << mav_angular_acc <<std::endl;                

        };


        // clear accumulated net wrenches for next iteration
        ClearNetWrenches2Payload();

    std::cout<<std::string(2, ' ')<<"Leave Cooperative::DoOneStepInt4Robots()"<<std::endl; 

    std::cout.unsetf(std::ios::fixed); // Reset fixed format
}  
    

void Cooperative::ClearNetWrenches2Payload()
{
    net_mavs_wrench_to_payload_.force = Eigen::Vector3d::Zero();
    net_mavs_wrench_to_payload_.torque = Eigen::Vector3d::Zero();

    interaction_parameters_.setZero();
}
     


