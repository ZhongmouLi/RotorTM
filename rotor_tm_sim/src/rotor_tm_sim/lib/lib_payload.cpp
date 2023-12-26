#include "rotor_tm_sim/lib_payload.hpp"


Payload::Payload(const std::vector<Eigen::Vector3d> &v_attach_point_post_bf, const double &mass, const Eigen::Matrix3d &m_inertia, const double &step_size): RigidBody(mass, m_inertia, step_size), v_attach_points_posts_body_frame_(v_attach_point_post_bf)
{
    // reserve size of vectors v_attach_points_posts_ and v_attach_points_vels_
    v_attach_points_posts_.reserve(v_attach_points_posts_body_frame_.size());

    v_attach_points_vels_.reserve(v_attach_points_posts_body_frame_.size());

    v_attach_point_accs_.reserve(v_attach_points_posts_body_frame_.size());
}




// void Payload::CalPost4AttachPoint(const )
// {
//     Eigen::Vector3d payload_post(0,0,0);
//     Eigen::Vector3d payload_vel(0,0,0);    
//     Eigen::Quaterniond payload_attitude(1,0,0,0);


//     // obtain post, vel and attitude of payload
//     this->getPosition(payload_post);

//     this->getVel(payload_vel);

//     this->getAttitude(payload_attitude);



// }




// void Payload::CalVel4AttachPoint();      




void Payload::ComputeAttachPointsPostVel()
{

    Eigen::Vector3d payload_position;

    Eigen::Quaterniond payload_attitude;
    // std::cout<<"fuck 1 "<<std::endl;
    getPosition(payload_position);

    getAttitude(payload_attitude);

    Eigen::Matrix3d m_payload_rotation = payload_attitude.toRotationMatrix();

    // std::cout<<"fuck 2 "<<std::endl;

    // std::for_each(v_attach_points_posts_body_frame_.begin(),
    //          v_attach_points_posts_body_frame_.end(),
    //          [](const Eigen::Vector3d& elem) {
 
    //              // printing one by one element
    //              // separated with space
    //              std::cout << "ele is "<<elem.transpose() << std::endl;
    //          });
    
    // compute attach point posts in world frame and store them in v_attach_points_posts_
    // formula: attach_point_world_frame = payload_position + (m_payload_rotation* attach_point_body_frame)
    // std::transform(v_attach_points_posts_body_frame_.begin(), v_attach_points_posts_body_frame_.end(), v_attach_points_posts_.begin(),[payload_position, m_payload_rotation](auto attach_point_body_frame) { //
    //                                                                             Eigen::Vector3d attach_point_world_frame;
    //                                                                             std::cout<<"fuck 2.5 "<<std::endl;
    //                                                                             attach_point_world_frame = payload_position + (m_payload_rotation* attach_point_body_frame);
                                                                                
    //                                                                             std::cout<<"fuck 2.7 "<<std::endl;
    //                                                                             std::cout << "ele is "<<attach_point_world_frame.transpose() << std::endl;


    //                                                                             return attach_point_world_frame ;
    //                                                                             // v_attach_points_posts_.push_back(attach_point_world_frame);

    //                                                                             });

    /*works*/
    // std::for_each(v_attach_points_posts_body_frame_.begin(),
    //          v_attach_points_posts_body_frame_.end(),
    //          [payload_position, m_payload_rotation, this](const Eigen::Vector3d& attach_point_body_frame) {

    //          // update post   
    //          Eigen::Vector3d attach_point_world_frame;
    //          attach_point_world_frame = payload_position + (m_payload_rotation* attach_point_body_frame);

    //          v_attach_points_posts_.push_back(attach_point_world_frame);

    //          });


    // compute vels of attach points
    Eigen::Vector3d payload_vel(0,0,0);    
    Eigen::Vector3d payload_bodyrate(0,0,0);  

    getBodyrate(payload_bodyrate);
    getVel(payload_vel);

    std::for_each(v_attach_points_posts_body_frame_.begin(),
             v_attach_points_posts_body_frame_.end(),
             [payload_position, m_payload_rotation, payload_bodyrate, payload_vel, this](const Eigen::Vector3d& attach_point_body_frame) {

             // update post   
             Eigen::Vector3d attach_point_post_world_frame;
             attach_point_post_world_frame = payload_position + (m_payload_rotation* attach_point_body_frame);
             v_attach_points_posts_.push_back(attach_point_post_world_frame);
            
            
            
            // update vels
            // pl_vel + pl_rot @ utilslib.vec2asym(x[10:13]) @ rho_vec_list

            Eigen::Vector3d attach_point_vel_world_frame;

            Eigen::Matrix3d m_skew_payload_bodyrate = TransVector3d2SkewSymMatrix(payload_bodyrate);
            
            attach_point_vel_world_frame = payload_vel + m_payload_rotation * m_skew_payload_bodyrate * attach_point_body_frame;
            // v_attach_points_vels_.push_back(attach_point_vel_world_frame);
            // 
             });
}


void Payload::GetOneAttachPointPost(size_t i, Eigen::Vector3d attach_point_post) const
{

    std::try 
    {
        if (i <= attach_point_post.size()-1) 
        {
        attach_point_post = v_attach_points_posts_[i];
        } 
        else {
        std::throw (i);
        }
    }
    
    std::catch (size_t myNum) 
    {
    std::cout << "index is beyong the size of vector attach_point_post";
    cout << "index is: " << myNum;  
    }

}




void Payload::GetAttachPointVel(size_t i, Eigen::Vector3d attach_point_vel) const
{

    std::try 
    {
        if (i <= v_attach_points_vels_.size()-1) 
        {
        attach_point_vel = v_attach_points_vels_[i];
        } 
        else {
        std::throw (i);
        }
    }
    
    std::catch (size_t myNum) 
    {
    std::cout << "index is beyong the size of vector attach_point_vel";
    cout << "index is: " << myNum;  
    }

}



// // getAttachPointPost obtain vector of attach points vel
// void Payload::getAttachPointsVels(std::vector<Eigen::Vector3d> &v_attach_points_vels) const
// {
//     v_attach_points_vels = v_attach_points_vels_;
// };   


// void Payload::ComputeVarJ()
// {
//     // compute Ji
//     Eigen::MatrixXd Ji = ComputeMatrixJi(cable_direction, payload_attitude, attach_point_body_frame);

//     //accumlate Ji * mi to J that is shown in Eq 45 
//     J = J + Ji * drone_mass;
// }


// void Payload::ComputeVarb()
// {
//     // compute bi for drones whose cables are taut
//     Eigen::VectorXd bi = ComputeVectorbi(drone_mass, drone_vel, cable_direction, payload_attitude, attach_point_body_frame);

//     // allocate bi for b
//     b = b + bi;    
// }


// UpdateVelCollided computs updated vel of payload after collision
void Payload::UpdateVelCollided(const std::vector<UAVCable> &v_drone_cable)
{
    // 1. define matrix J and b in J [xL+, omegaL+] = b
    Eigen::MatrixXd J = Eigen::MatrixXd::Identity(6,6);
    Eigen::VectorXd b = Eigen::MatrixXd::Identity(6,1);

    // 2. initialisation of J and b
    // 2.1 obtain payload mass and inerita
    double payload_mass;
    GetMass(payload_mass);

    Eigen::Matrix3d payload_inertia;
    GetInertia(payload_inertia);    

    Eigen::Quaterniond payload_attitude;
    getAttitude(payload_attitude);

    Eigen::Matrix3d m_payload_rotation = payload_attitude.toRotationMatrix();

    Eigen::Vector3d payload_vel;
    getVel(payload_vel);

    Eigen::Vector3d payload_bodyrate;
    getBodyrate(payload_bodyrate);

    // 2.2 initialisation of J
    // . J[1:3, 1:3] = mL * I3
    J.block<3,3>(0,0) = Eigen::MatrixXd::Identity(3,3) * payload_mass;
    // J[4:6, 4:6] = IL
    J.block<3,3>(3,3) = payload_inertia;

    // 2.3 initialisation of b
    // left variable of b in Eq 42 that depend on payload
    b.head<3>() = payload_vel * payload_mass;
    b.tail<3>() = payload_inertia * payload_bodyrate;

    // 3 compute all Ji in Eq45 and Eq46 for every drone that is collided
    // check the numner of UAVCable equals to the number of attachment points
    if (v_drone_cable.size() != v_attach_points_posts_body_frame_.size())
    {
        std::cout<<"number of cable-drone != number of attach point"<<std::endl;
        return;
    }
    
    // get number of robots
    size_t number_robot = v_drone_cable.size();

    // iterate for each drone, cable, attach point
    for (size_t i = 0; i < number_robot; ++i) {

        // (1) obtain UAVCable ptr, attach point post in world frame and body frame, and its vel in world frame
        // where UAV are connect to attach point through cable    
        UAVCable UAVCable = v_drone_cable[i];

        Eigen::Vector3d attach_point_body_frame = v_attach_points_posts_body_frame_[i];

        Eigen::Vector3d attachpoint_post = v_attach_points_posts_[i];
        Eigen::Vector3d attachpoint_vel  = v_attach_points_vels_[i];

        // (1) check if the cable of UAVCable is taut
        bool flag_cable_taut;
        UAVCable.CheckCollision(attachpoint_post, attachpoint_vel);


        if (flag_cable_taut == true) // the cable is taut 
        {
            // 1) compute cable direction, drone position, and drone vel
            Eigen::Vector3d cable_direction;
            Eigen::Vector3d drone_post;
            Eigen::Vector3d drone_vel;

            UAVCable.cable_.ComputeCableDirection(attachpoint_post, drone_post);
            UAVCable.cable_.GetCableDirection(cable_direction);

            UAVCable.drone_.getPosition(drone_post);
            UAVCable.drone_.getVel(drone_vel);


            // 2) obtain drone mass
            double drone_mass;
            UAVCable.drone_.GetMass(drone_mass);

            // 3) compute Ji for drones whose cables are taut
            Eigen::MatrixXd Ji = ComputeMatrixJi(cable_direction, payload_attitude, attach_point_body_frame);

            // 4) accumlate Ji * mi to J that is shown in Eq 45 
            J = J + Ji * drone_mass;

            // 5) compute bi for drones whose cables are taut
            Eigen::VectorXd bi = ComputeVectorbi(drone_mass, drone_vel, cable_direction, payload_attitude, attach_point_body_frame);
            b = b + bi;
        }

        // payload_vel_bodyrate_collised =[vel; bodyrate]
        Eigen::VectorX payload_vel_bodyrate_collised{0,0,0,0,0,0};

        // solve J [vel; bodyrate] = b in Eq.42
        payload_vel_bodyrate_collised = J.householderQr().solve(b);

        SetVel(payload_vel_bodyrate_collised.head(3));

        SetBodyrate(payload_vel_bodyrate_collised.tail(3));
    }




// void Payload::UpdateVelCollided(const std::vector<std::shared_ptr<UAVCable>> v_drone_cable_ptr)
// {
//     // 1. define matrix J and b in J [xL+, omegaL+] = b
//     Eigen::MatrixXd J = Eigen::MatrixXd::Identity(6,6);
//     Eigen::VectorXd b = Eigen::MatrixXd::Identity(6,1);

//     // 2. initialisation of J and b
//     // 2.1 obtain payload mass and inerita
//     double payload_mass;
//     GetMass(payload_mass);

//     Eigen::Matrix3d payload_inertia;
//     GetInertia(payload_inertia);    

//     Eigen::Quaterniond payload_attitude;
//     getAttitude(payload_attitude);

//     Eigen::Matrix3d m_payload_rotation = payload_attitude.toRotationMatrix();

//     Eigen::Vector3d payload_vel;
//     getVel(payload_vel);

//     Eigen::Vector3d payload_bodyrate;
//     getBodyrate(payload_bodyrate);

//     // 2.2 initialisation of J
//     // . J[1:3, 1:3] = mL * I3
//     J.block<3,3>(0,0) = Eigen::MatrixXd::Identity(3,3) * payload_mass;
//     // J[4:6, 4:6] = IL
//     J.block<3,3>(3,3) = payload_inertia;

//     // 2.3 initialisation of b
//     // left variable of b in Eq 42 that depend on payload
//     b.head<3>() = payload_vel * payload_mass;
//     b.tail<3>() = payload_inertia * payload_bodyrate;

//     // 3 compute all Ji in Eq45 and Eq46 for every drone that is collided
//     // check the numner of UAVCable equals to the number of attachment points
//     if (v_drone_cable_ptr.size() != v_attach_points_posts_body_frame_.size())
//     {
//         std::cout<<"number of cable-drone != number of attach point"<<std::endl;
//         return;
//     }
    
//     // get number of robots
//     size_t number_robot = v_drone_cable_ptr.size();

//     // iterate for each drone, cable, attach point
//     for (size_t i = 0; i < number_robot; ++i) {

//         // (1) obtain UAVCable ptr, attach point post in world frame and body frame, and its vel in world frame
//         // where UAV are connect to attach point through cable    
//         std::shared_ptr<UAVCable> ptr_UAVCable = v_drone_cable_ptr[i];

//         Eigen::Vector3d attach_point_body_frame = v_attach_points_posts_body_frame_[i];

//         Eigen::Vector3d attachpoint_post = v_attach_points_posts_[i];
//         Eigen::Vector3d attachpoint_vel  = v_attach_points_vels_[i];

//         // (1) check if the cable of UAVCable is taut
//         bool flag_cable_taut;
//         ptr_UAVCable->CheckCollision(attachpoint_post, attachpoint_vel);


//         if (flag_cable_taut == true) // the cable is taut 
//         {
//             // 1) compute cable direction, drone position, and drone vel
//             Eigen::Vector3d cable_direction;
//             Eigen::Vector3d drone_post;
//             Eigen::Vector3d drone_vel;

//             ptr_UAVCable->cable_.ComputeCableDirection(attachpoint_post, drone_post);
//             ptr_UAVCable->cable_.GetCableDirection(cable_direction);

//             ptr_UAVCable->drone_.getPosition(drone_post);
//             ptr_UAVCable->drone_.getVel(drone_vel);


//             // 2) obtain drone mass
//             double drone_mass;
//             ptr_UAVCable->drone_.GetMass(drone_mass);

//             // 3) compute Ji for drones whose cables are taut
//             Eigen::MatrixXd Ji = ComputeMatrixJi(cable_direction, payload_attitude, attach_point_body_frame);

//             // 4) accumlate Ji * mi to J that is shown in Eq 45 
//             J = J + Ji * drone_mass;

//             // 5) compute bi for drones whose cables are taut
//             Eigen::VectorXd bi = ComputeVectorbi(drone_mass, drone_vel, cable_direction, payload_attitude, attach_point_body_frame);
//             b = b + bi;
//         }

//         // payload_vel_bodyrate_collised =[vel; bodyrate]
//         Eigen::VectorX payload_vel_bodyrate_collised{0,0,0,0,0,0};

//         // solve J [vel; bodyrate] = b in Eq.42
//         payload_vel_bodyrate_collised = J.householderQr().solve(b);

//         SetVel(payload_vel_bodyrate_collised.head(3));

//         SetBodyrate(payload_vel_bodyrate_collised.tail(3));
//     }


    // group UAVCable and attach appoint together in pair

    // UAV_Cable_attachpoint.first = std::shared_ptr<UAVCable>
    // UAV_Cable_attachpoint.second = <Eigen::Vector3d> 
    // std::vector<//
    //         std::pair<//
    //         std::shared_ptr<UAVCable>, Eigen::Vector3d//
    //         >//
    // > pair_UAV_Cable_attachpoint;

    // // Fill the combined vector UAV_Cable_attachpoint
    // for (size_t i = 0; i < v_drone_cable_ptr.size(); ++i) {
    //     pair_UAV_Cable_attachpoint.push_back(std::make_pair(v_drone_cable_ptr[i], v_attach_points_posts_body_frame_[i]));
    // }

    
    // // Iterate over the combined vector
    // for (const auto& UAV_Cable_attachpoint : pair_UAV_Cable_attachpoint) 
    // {
    //     // (1) obtain UAVCable ptr and attach point
    //     // where UAV are connect to attach point through cable    
    //     std::shared_ptr<UAVCable> ptr_UAVCable = UAV_Cable_attachpoint.first;
    //     Eigen::Vector3d attach_point = UAV_Cable_attachpoint.second;

    //     // (1) check if the cable of UAVCable is taut
    //     bool flag_cable_taut;
    //     ptr_UAVCable->CheckCollision(attachpoint_post, attachpoint_vel);


    //     if (flag_cable_taut == true) // the cable is taut 
    //     {
    //         // 1) compute cable direction xi
    //         Eigen::Vector3d xi;
    //         ptr_UAVCable->cable_.ComputeCableDirection();
    //         ptr_UAVCable->cable_.GetCableDirection(xi);

    //         // 2) obtain drone mass
    //         double drone_mass;
    //         ptr_UAVCable->drone_.GetMass(drone_mass);

    //         // 3) compute Ji for drones whose cables are taut
    //         Eigen::MatrixXd Ji = ComputeMatrixJi(xi, payload_attitude, attach_point);

    //         // 4) accumlate Ji * mi to J that is shown in Eq 45 
    //         J = J + Ji * drone_mass;
    //     }
    // }


    // for (const auto& drone_cable_ptr : v_drone_cable_ptr) {
    //     // drone_cable_ptr is a pointer points to an instance of UAVCable
    //     // (1) check if the cable of UAVCable is taut
    //     bool flag_cable_taut;
    //     drone_cable_ptr->cable_.CheckTaut(flag_cable_taut);


    //     if (flag_cable_taut == true) // the cable is taut 
    //     {
    //         // 1) compute cable direction xi
    //         Eigen::Vector3d xi;
    //         drone_cable_ptr->cable_.ComputeCableDirection();
    //         drone_cable_ptr->cable_.GetCableDirection(xi);

    //         // 2) obtain drone mass
    //         double drone_mass;
    //         drone_cable_ptr->drone_.GetMass(drone_mass);

    //         // 3) compute Ji for drones whose cables are taut
    //         Eigen::MatrixXd Ji = ComputeMatrixJi(xi, payload_attitude, attach_point_body_frame);

    //         // 4) accumlate Ji * mi to J that is shown in Eq 45 
    //         J = J + Ji * drone_mass;
    //     }
        
    // }



}


Eigen::MatrixXd Payload::ComputeMatrixJi(const Eigen::Vector3d &cable_direction, const Eigen::Quaterniond &payload_attitude, const Eigen::Vector3d &attach_point_body_frame)
{
    // define matrix Ji in Eq45    
    Eigen::MatrixXd Ji = Eigen::MatrixXd::Identity(6,6);

    // compute ai in Eq 43
    // xi = cable_direction
    Eigen::Matrix3d ai = cable_direction *cable_direction.transpose();
    
    // compute bi in Eq 43 and 44
    Eigen::Matrix3d hat_pho = TransVector3d2SkewSymMatrix(attach_point_body_frame);
    Eigen::Matrix3d payload_R = payload_attitude.toRotationMatrix(); // convert a quaternion to a 3x3 rotation matrix

    Eigen::Matrix3d bi = hat_pho * payload_R.transpose();

    // compute bi_t in Eq 44
    Eigen::Matrix3d bi_t = - payload_R * hat_pho;


    // construct matrix Ji in Eq 46
    // Ji[1:3,1:3] = ai
    Ji.block<3,3>(0,0) = ai;

    // Ji[1:3,4:6] = ai * bi_t
    Ji.block<3,3>(0,3) = ai * bi_t;

    // Ji[4:6,1:3] = bi * ai
    Ji.block<3,3>(3,0) = bi * ai;


    // Ji[4:6,4:6] = bi * ai
    Ji.block<3,3>(3,3) = bi * ai * bi_t;


    return Ji;
}

Eigen::VectorXd Payload::ComputeVectorbi(const double &drone_mass, const Eigen::Vector3d &drone_vel, const Eigen::Vector3d &cable_direction, const Eigen::Quaterniond &payload_attitude, const Eigen::Vector3d &attach_point_body_frame)
{
    Eigen::VectorXd bi = Eigen::MatrixXd::Identity(6,1);

    Eigen::Vector3d bi_up = Eigen::MatrixXd::Identity(3,1);
    Eigen::Vector3d bi_down = Eigen::MatrixXd::Identity(3,1);

    // compute first three elements in bi in eq 42
    bi_up = drone_mass * cable_direction *cable_direction.transpose()*drone_vel;

    Eigen::Matrix3d payload_R = payload_attitude.toRotationMatrix(); // convert a quaternion to a 3x3 rotation matrix
    Eigen::Matrix3d hat_pho = TransVector3d2SkewSymMatrix(attach_point_body_frame);

    // compute last three elements in bi in eq 42
    bi_down = drone_mass * hat_pho * payload_R.transpose() * cable_direction *cable_direction.transpose()*drone_vel;

    bi.head<3>() = bi_up;
    bi.tail<3>() = bi_down;

}

Payload::ComputeAttachPointAccs()
{
      attach_centrifugal_accel = np.matmul(pl_omg_asym, np.matmul(pl_omg_asym, self.rho_vec_list))
      self.attach_accel = self.pl_accel + np.array([0,0,self.pl_params.grav]) + \
                          np.matmul(pl_rot, np.matmul(utilslib.vec2asym(self.pl_ang_accel), self.rho_vec_list)).T + \
                          np.matmul(pl_rot, attach_centrifugal_accel).T

                     
    // 1. obtain number of attach points
    size_t number_attach_point = v_attach_point_accs_.size();


    // 2. 

    // obtain payload acc
    Eigen::Vector3d payload_acc;
    GetAcc(payload_acc)

    // obtain payload bodyrate acc
    Eigen::Vector3d payload_bodyrate_acc;
    GetBodyRateAcc(payload_bodyrate_acc)    
    Eigen::Matrix3d m_skewsym_payload_bodyrate_acc = TransVector3d2SkewSymMatrix(payload_bodyrate_acc);
    
    // compute skew sym matrix of payload body rate
    Eigen::Vector3d payload_bodyrate;
    getBodyrate(payload_bodyrate);

    Eigen::Matrix3d m_payload_bodyrate_skewsym = TransVector3d2SkewSymMatrix(payload_bodyrate);

    // obtain payload attitude
    Eigen::Quaterniond payload_attitude;
    getAttitude(payload_attitude);

    Eigen::Matrix3d m_payload_rotation = payload_attitude.toRotationMatrix();


    // iterate for each drone, cable, attach point
    for (size_t i = 0; i < number_attach_point; ++i) {

        // 3.1 obtain current attach point
        Eigen::Vector3d attach_point_body_frame = v_attach_points_posts_body_frame_[i];

        Eigen::Vector3d attach_point_post = v_attach_points_posts_[i];
        Eigen::Vector3d attachpoint_vel  = v_attach_points_vels_[i];

        // 3.2 compute centri acc of attach point
        Eigen::Vector3d attach_point_centri_acc =  m_payload_bodyrate_skewsym * (m_payload_bodyrate_skewsym * attach_point_body_frame);

        // 3.3 compute acc of attach point
        Eigen::Vector3d attach_point_acc = payload_acc + (Eigen::Vector3d::UnitZ() * gravity_) + (m_payload_rotation * (m_skewsym_payload_bodyrate_acc * attach_point_body_frame)).transpose() + (m_payload_rotation * attach_point_centri_acc).transpose();

    }
}


void Payload::InputMassMatrix(const Eigen::Matrix3d &m_mass_matrix)
{
    m_mass_matrix_ = mass_matrix;
}

void Payload::InputDronesNetForces(const Eigen::Vector3d &drones_net_force, const Eigen::Matrix3d &m_D)
{
    drones_net_forces_ = drones_net_force;

    m_D_ = m_D;
    
}


void Payload::InputDronesNetTorques(const Eigen::Vector3d &drones_net_torque, const Eigen::Matrix3d &m_C, const Eigen::Matrix3d &m_E)
{
    drones_net_torque_ = drones_net_torque;

    m_C_ = m_C;

    m_E_ = m_E;
    
}



Eigen::Vector3d Payload::ComputeTransDynamics(const Eigen::Vector3d &drones_net_forces, const Eigen::Matrix3d &mass_matrix, const Eigen::Matrix3d &m_D,  const Eigen::Vector3d &payload_bodyrate_acc)
{
    Eigen::Vector3d payload_acc(0,0,0);
    

    payload_acc = mass_matrix.householderQr().solve(drones_net_forces + m_D * payload_bodyrate_acc) - gravity_;

    return payload_acc;
}


Eigen::Vector3d Payload::ComputeRotDynamics(const Eigen::Vector3d &drones_net_forces, const Eigen::Vector3d &drones_net_torques, const Eigen::Matrix3d &m_mass_matrix, const Eigen::Vector3d &payload_bodyrate, const Eigen::Matrix3d &m_C, const Eigen::Matrix3d &m_D, const Eigen::Matrix3d &m_E)
{

    // compute effective inertia and torque for the payload
    // such that
    // effective_inertia * payload_bodyrate_acc = effective_torque
    // this relation can be obtained from Eq17-18

    // Input
    // m_C in R{3X3}, m_C = Sum_k m_k * m_skew(attach_point_post_k) * (payload_attitude_rotation_matrix)^T * cable_direction_k * cable_direction_k^T 
    // m_D in R{3X3}, m_D = Sum_k m_k * cable_direction_k * cable_direction_k^T  * payload_attitude_rotation_matrix * m_skew(attach_point_post_k)
    // m_E in R{3X3}, m_E = Sum_k m_k * m_skew(attach_point_post_k) * (payload_attitude_rotation_matrix)^T * cable_direction_k * cable_direction_k^T  * payload_attitude_rotation_matrix * m_skew(attach_point_post_k)

    // setp 1. compute effective torque for the payload
    // such that 
    Eigen::Vector3d torque_effective(0,0,0);

    Eigen::Matrix3d payload_interia:
    GetInertia(payload_interia);

    Eigen::Matrix3d inv_m_mass_matrix = m_mass_matrix.inverse();

    torque_effective = drones_net_torques - m_C * inv_m_mass_matrix *  drones_net_forces - TransVector3d2SkewSymMatrix(payload_bodyrate) * payload_interia * payload_bodyrate;

    // step 2. compute effective inertia
    Eigen::Matrix3d interia_effective;

    interia_effective = payload_interia + m_C * (inv_m_mass_matrix * m_D) - m_E;

    // step 3 compute bodyrate acc
    // bodyrate_acc = inv(effective_inertia) * effective_torque

    Eigen::Vector3d bodyrate_acc;

    bodyrate_acc =  interia_effective.householderQr().solve(torque_effective);

    return bodyrate_acc;
}


void Payload::operator() (const object_state &x , object_state &dxdt, const double time)
{

    static bool is_recursing = false;
    if (is_recursing) return;  // Prevent recursion
    is_recursing = true;
    
    // std::cout << "state of payload" << x.transpose()<<std::endl; 
    // x =  [x,     y,      z,      dx,     dy,     dz,     phi,    theta,      psi,    p,      q,      r]
    // dx = [dx,    dy,     dz,     ddx,    ddy,    ddz,    dphi,   dtheta,     dpsi,   dp,     dq,     dr]

    // For instance
    // std::cout<< "Euler angle is "<< x.segment<3>(6).transpose()<<std::endl;
    // std::cout<< "Bodyrate is "<< x.tail(3).transpose()<<std::endl;
    // std::cout<< "position is "<< x.head(3).transpose()<<std::endl;
    // std::cout<< "vel is "<< x.segment<3>(3).transpose()<<std::endl;

    // obtain bodyrate [p,      q,      r]
    Eigen::Vector3d payload_bodyrate;
    payload_bodyrate = x.tail(3);    

    // obtain bodyrate acc [ dp,     dq,     dr]
    Eigen::Vector3d payload_bodyrate_acc;
    payload_bodyrate_acc = dx.tail(3);       

    // 1. translation in world frame
    // P = [x,y,z,dx, dy, dz]
    // dP = [dx, dy, dz, ddx, ddy, ddz]
    dxdt.head(3) = x.segment<3>(3);

    // [ddx ddy ddz] = 
    dxdt.segment<3>(3) = ComputeTransDynamics(drones_net_force_, m_mass_matrix_, m_D_, payload_bodyrate_acc);

    // 2. rotation in body frame
    Eigen::Matrix3d matrix_pdr2dEuler;
    matrix_pdr2dEuler = matirxBodyrate2EulerRate(x(6), x(7), x(8));

    // compute [dphi,   dtheta,     dpsi]^T =  matrix_pdr2dEuler * bodyrate
    dxdt.segment<3>(6) = matrix_pdr2dEuler * payload_bodyrate;

    // compute dp, dq ,dr
    dxdt.tail(3) =ComputeRotDynamics(drones_net_force_, drones_net_torque_, m_mass_matrix_, payload_bodyrate, m_C_, m_D_, m_E_);

    is_recursing = false;
}



void Payload::doOneStepInt()
{

    this->stepper_.do_step(*this, object_state, current_step_, step_size_);

    // acculate simulation steps
    current_step_ = current_step_ + step_size_;

    // // assign states to position and velocity
    // assignPtMasState(ptmas_state_);

};