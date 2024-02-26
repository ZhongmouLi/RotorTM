#include "rotor_tm_sim/lib_payload.hpp"


Payload::Payload(const std::vector<Eigen::Vector3d> &v_attach_point_post_bf, const double &mass, const Eigen::Matrix3d &m_inertia, const double &step_size): RigidBody(mass, m_inertia, step_size), v_attach_points_posts_body_frame_(v_attach_point_post_bf), v_attach_points_posts_(v_attach_point_post_bf.begin(), v_attach_point_post_bf.end()), v_attach_points_vels_(v_attach_point_post_bf.size(), Eigen::Vector3d::Zero()), v_attach_points_accs_(v_attach_point_post_bf.size(), Eigen::Vector3d::Zero())
{
    
    // obtain numnber of attach points that equals to number of robots
    num_robot_ = v_attach_points_posts_body_frame_.size();

    // std::cout<< "v_attach_points_posts_body_frame_.size()" << v_attach_points_posts_body_frame_.size() <<std::endl; 

    // std::cout<< "num_robot_" << num_robot_ <<std::endl;     

    // // reserve size of vectors v_attach_points_posts_ and v_attach_points_vels_
    // v_attach_points_posts_.reserve(num_robot_);

    // std::cout<< "v_attach_points_posts_.size()" << v_attach_points_posts_.size() <<std::endl; 


    // v_attach_points_vels_.reserve(num_robot_);

    // v_attach_points_accs_.reserve(num_robot_);
}




void Payload::ComputeAttachPointsKinematics()
{

    // obtain payload pose: post and attitude
    Eigen::Vector3d payload_position;
    Eigen::Quaterniond payload_attitude;

    // std::cout<<"fuck 1 "<<std::endl;
    GetPosition(payload_position);
    GetAttitude(payload_attitude);
    Eigen::Matrix3d m_payload_rotation = payload_attitude.toRotationMatrix();

    // obtain payload vel and bodyrate
    Eigen::Vector3d payload_vel{0,0,0};    
    Eigen::Vector3d payload_bodyrate{0,0,0};  
    GetVel(payload_vel);
    GetBodyrate(payload_bodyrate);
    Eigen::Matrix3d m_payload_bodyrate_skewsym = TransVector3d2SkewSymMatrix(payload_bodyrate);


    // obtain payload acc
    Eigen::Vector3d payload_acc;
    GetAcc(payload_acc);

    // obtain payload bodyrate acc
    Eigen::Vector3d payload_bodyrate_acc;
    GetBodyRateAcc(payload_bodyrate_acc);    
    Eigen::Matrix3d m_skewsym_payload_bodyrate_acc = TransVector3d2SkewSymMatrix(payload_bodyrate_acc);
    
    std::cout<<"[----------] Payload: ComputeAttachPointsKinematics fuck inside 1"<<std::endl;
    // compute post, vel and acc of each attach point
    for (size_t i = 0; i < num_robot_; i++)
    {
        // 1. obtain attach points' posts in body frame
        Eigen::Vector3d attach_point_body_frame = v_attach_points_posts_body_frame_.at(i);

        std::cout<<"[----------] Payload: ComputeAttachPointsKinematics fuck inside 2"<<std::endl;

        // 2. comnpute attach point's posts in world frame
        // 
        Eigen::Vector3d attach_point_post_world_frame;
        attach_point_post_world_frame = payload_position + (m_payload_rotation* attach_point_body_frame);

        std::cout<<"[----------] Payload: ComputeAttachPointsKinematics fuck inside 3"<<std::endl;

        std::cout<< "v_attach_points_posts_ is "<< v_attach_points_posts_.size()<<std::endl;

        v_attach_points_posts_.at(i)= attach_point_post_world_frame;
        std::cout<<"[----------] Payload: ComputeAttachPointsKinematics fuck inside 4"<<std::endl;

        // 3. compute attach point's vels in world frame
        Eigen::Vector3d attach_point_vel_world_frame;

        Eigen::Matrix3d m_skew_payload_bodyrate = TransVector3d2SkewSymMatrix(payload_bodyrate);
            
        attach_point_vel_world_frame = payload_vel + m_payload_rotation * m_skew_payload_bodyrate * attach_point_body_frame;
        
        std::cout<<"[----------] Payload: ComputeAttachPointsKinematics fuck inside 5"<<std::endl;
        v_attach_points_vels_.at(i) = attach_point_vel_world_frame;        

        // 4 compute centri acc of attach point
        Eigen::Vector3d attach_point_centri_acc =  m_payload_bodyrate_skewsym * (m_payload_bodyrate_skewsym * attach_point_body_frame);
        std::cout<< "attach_point_centri_acc  is " <<attach_point_centri_acc.transpose()<<std::endl;
        // 3.3 compute acc of attach point
        //  self.attach_accel = self.pl_accel + np.array([0,0,self.pl_params.grav]) + np.matmul(pl_rot, np.matmul(utilslib.vec2asym(self.pl_ang_accel), self.rho_vec_list)).T + np.matmul(pl_rot, attach_centrifugal_accel).T

        Eigen::Vector3d attach_point_acc = payload_acc + (Eigen::Vector3d::UnitZ() * gravity_) + (m_payload_rotation * (m_skewsym_payload_bodyrate_acc * attach_point_body_frame)) + (m_payload_rotation * attach_point_centri_acc);

        // Eigen::Vector3d attach_point_acc = payload_acc + (Eigen::Vector3d::UnitZ() * gravity_) + (m_payload_rotation * (m_skewsym_payload_bodyrate_acc * attach_point_body_frame)).transpose();
        std::cout<<"[----------] Payload: ComputeAttachPointsKinematics fuck inside 6"<<std::endl;
        v_attach_points_accs_.at(i) = attach_point_acc;   
        std::cout<< "attach_point_acc  is " <<attach_point_acc.transpose()<<std::endl;
        std::cout<< "v_attach_points_accs_  is " <<v_attach_points_accs_.at(i) <<std::endl;            
    };
    

// void Payload::ComputeAttachPointAccs()
// {

//     //   attach_centrifugal_accel = np.matmul(pl_omg_asym, np.matmul(pl_omg_asym, self.rho_vec_list))
//     //   self.attach_accel = self.pl_accel + np.array([0,0,self.pl_params.grav]) + np.matmul(pl_rot, np.matmul(utilslib.vec2asym(self.pl_ang_accel), self.rho_vec_list)).T + np.matmul(pl_rot, attach_centrifugal_accel).T

                     
//     // 1. number of attach points is num_robot_
//     // size_t number_attach_point = v_attach_points_posts_body_frame_.size();


//     // 2. 

//     // obtain payload acc
//     Eigen::Vector3d payload_acc;
//     GetAcc(payload_acc);

//     // obtain payload bodyrate acc
//     Eigen::Vector3d payload_bodyrate_acc;
//     GetBodyRateAcc(payload_bodyrate_acc);    
//     Eigen::Matrix3d m_skewsym_payload_bodyrate_acc = TransVector3d2SkewSymMatrix(payload_bodyrate_acc);
    
//     // compute skew sym matrix of payload body rate
//     Eigen::Vector3d payload_bodyrate;
//     GetBodyrate(payload_bodyrate);

//     Eigen::Matrix3d m_payload_bodyrate_skewsym = TransVector3d2SkewSymMatrix(payload_bodyrate);

//     // obtain payload attitude
//     Eigen::Quaterniond payload_attitude;
//     GetAttitude(payload_attitude);

//     Eigen::Matrix3d m_payload_rotation = payload_attitude.toRotationMatrix();


//     // iterate for each drone, cable, attach point
//     for (size_t i = 0; i < num_robot_; i++) {

//         // 3.1 obtain current attach point
//         Eigen::Vector3d attach_point_body_frame = v_attach_points_posts_body_frame_[i];

//         Eigen::Vector3d attach_point_post = v_attach_points_posts_[i];
//         Eigen::Vector3d attachpoint_vel  = v_attach_points_vels_[i];

//         // 3.2 compute centri acc of attach point
//         Eigen::Vector3d attach_point_centri_acc =  m_payload_bodyrate_skewsym * (m_payload_bodyrate_skewsym * attach_point_body_frame);

//         // 3.3 compute acc of attach point
//         //  self.attach_accel = self.pl_accel + np.array([0,0,self.pl_params.grav]) + np.matmul(pl_rot, np.matmul(utilslib.vec2asym(self.pl_ang_accel), self.rho_vec_list)).T + np.matmul(pl_rot, attach_centrifugal_accel).T

//         Eigen::Vector3d attach_point_acc = payload_acc + (Eigen::Vector3d::UnitZ() * gravity_) + (m_payload_rotation * (m_skewsym_payload_bodyrate_acc * attach_point_body_frame)) + (m_payload_rotation * attach_point_centri_acc);

//         // Eigen::Vector3d attach_point_acc = payload_acc + (Eigen::Vector3d::UnitZ() * gravity_) + (m_payload_rotation * (m_skewsym_payload_bodyrate_acc * attach_point_body_frame)).transpose();
        
//         v_attach_points_accs_[i] = attach_point_acc;

//     }
// }


    // std::for_each(v_attach_points_posts_body_frame_.begin(),
    //          v_attach_points_posts_body_frame_.end(),
    //          [payload_position, m_payload_rotation, payload_bodyrate, payload_vel, this](const Eigen::Vector3d& attach_point_body_frame) {

    //          // update post   
    //          Eigen::Vector3d attach_point_post_world_frame;
    //          attach_point_post_world_frame = payload_position + (m_payload_rotation* attach_point_body_frame);
    //          v_attach_points_posts_.push_back(attach_point_post_world_frame);
            
            
            
    //         // update vels
    //         // pl_vel + pl_rot @ utilslib.vec2asym(x[10:13]) @ rho_vec_list

    //         Eigen::Vector3d attach_point_vel_world_frame;

    //         Eigen::Matrix3d m_skew_payload_bodyrate = TransVector3d2SkewSymMatrix(payload_bodyrate);
            
    //         attach_point_vel_world_frame = payload_vel + m_payload_rotation * m_skew_payload_bodyrate * attach_point_body_frame;
    //         v_attach_points_vels_.push_back(attach_point_vel_world_frame);
    //         // 
    //          });


}


void Payload::GetOneAttachPointPost(const size_t &i, Eigen::Vector3d &attach_point_post) const
{

    // std::cout<< "[----------]" <<"input i is "<< i << " num_robot_ is "<< num_robot_<<std::endl;
    try 
    {
        if (i < num_robot_ ) 
        {
            attach_point_post = v_attach_points_posts_[i];
            // std::cout<<"[----------]" << i << "th attach point post is" << attach_point_post.transpose()<<std::endl;
        } 
        else {
        throw i;
        }
    }
    
    catch (size_t myNum) 
    {
    std::cout << "index is beyong the size of vector attach_point_post";
    std::cout << "index is: " << myNum;  
    }

}

void Payload::GetOneAttachPointPostBodyFrame(const size_t &i, Eigen::Vector3d &attach_point_post_bodyframe) const
{

    attach_point_post_bodyframe = v_attach_points_posts_body_frame_[i];
} 




void Payload::GetOneAttachPointVel(const size_t &i, Eigen::Vector3d &attach_point_vel) const
{

    try 
    {
        if (i < num_robot_  ) 
        {
            attach_point_vel = v_attach_points_vels_[i];
            std::cout<<"[----------]" << i << "th attach point vel is" << attach_point_vel.transpose()<<std::endl;
        } 
        else {
        throw (i);
        }
    }
    
    catch (size_t myNum) 
    {
    std::cout << "index is beyong the size of vector attach_point_vel";
    std::cout << "index is: " << myNum;  
    }

}


void Payload::GetOneAttachPointAcc(const size_t &i, Eigen::Vector3d &attach_point_acc) const
{
    attach_point_acc =v_attach_points_accs_.at(i);

}



// UpdateVelCollided computs updated vel of payload after collision
void Payload::UpdateVelCollided(const std::vector<UAVCable> &v_mav_cable)
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
    GetAttitude(payload_attitude);

    // Eigen::Matrix3d m_payload_rotation = payload_attitude.toRotationMatrix();

    Eigen::Vector3d payload_vel;
    GetVel(payload_vel);

    Eigen::Vector3d payload_bodyrate;
    GetBodyrate(payload_bodyrate);

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
    if (v_mav_cable.size() != v_attach_points_posts_body_frame_.size())
    {
        std::cout<<"number of cable-drone != number of attach point"<<std::endl;
        return;
    }
    
    // get number of robots
    size_t number_robot = v_mav_cable.size();

    // iterate for each drone, cable, attach point
    for (size_t i = 0; i < number_robot; ++i) {

        // (1) obtain UAVCable ptr, attach point post in world frame and body frame, and its vel in world frame
        // where UAV are connect to attach point through cable    
        UAVCable UAVCable = v_mav_cable[i];

        Eigen::Vector3d attach_point_body_frame = v_attach_points_posts_body_frame_[i];

        Eigen::Vector3d attachpoint_post = v_attach_points_posts_[i];
        Eigen::Vector3d attachpoint_vel  = v_attach_points_vels_[i];

        // (1) check if the cable of UAVCable is taut
        // bool flag_cable_taut = false;
        // UAVCable.CheckCollision(attachpoint_post, attachpoint_vel);

        if (UAVCable.IsCollided(attachpoint_post, attachpoint_vel)) // the cable is taut 
        {
            // 1) compute cable direction, drone position, and drone vel
            Eigen::Vector3d cable_direction;
            Eigen::Vector3d mav_post;
            Eigen::Vector3d mav_vel;

            UAVCable.cable_.ComputeCableDirection(attachpoint_post, mav_post);
            UAVCable.cable_.GetCableDirection(cable_direction);

            UAVCable.mav_.GetPosition(mav_post);
            UAVCable.mav_.GetVel(mav_vel);


            // 2) obtain drone mass
            double mav_mass;
            UAVCable.mav_.GetMass(mav_mass);

            // 3) compute Ji for drones whose cables are taut
            Eigen::MatrixXd Ji = ComputeMatrixJi(cable_direction, payload_attitude, attach_point_body_frame);

            // 4) accumlate Ji * mi to J that is shown in Eq 45 
            J = J + Ji * mav_mass;

            // 5) compute bi for drones whose cables are taut
            Eigen::VectorXd bi = ComputeVectorbi(mav_mass, mav_vel, cable_direction, payload_attitude, attach_point_body_frame);
            b = b + bi;
        }

        // payload_vel_bodyrate_collised =[vel; bodyrate]
        Eigen::VectorXd payload_vel_bodyrate_collised = Eigen::MatrixXd::Zero(6, 1);

        // solve J [vel; bodyrate] = b in Eq.42
        payload_vel_bodyrate_collised = J.householderQr().solve(b);

        SetVel(payload_vel_bodyrate_collised.head(3));

        SetBodyrate(payload_vel_bodyrate_collised.tail(3));
    }




// void Payload::UpdateVelCollided(const std::vector<std::shared_ptr<UAVCable>> v_mav_cable_ptr)
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
//     GetAttitude(payload_attitude);

//     Eigen::Matrix3d m_payload_rotation = payload_attitude.toRotationMatrix();

//     Eigen::Vector3d payload_vel;
//     GetVel(payload_vel);

//     Eigen::Vector3d payload_bodyrate;
//     GetBodyrate(payload_bodyrate);

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
//     if (v_mav_cable_ptr.size() != v_attach_points_posts_body_frame_.size())
//     {
//         std::cout<<"number of cable-drone != number of attach point"<<std::endl;
//         return;
//     }
    
//     // get number of robots
//     size_t number_robot = v_mav_cable_ptr.size();

//     // iterate for each drone, cable, attach point
//     for (size_t i = 0; i < number_robot; ++i) {

//         // (1) obtain UAVCable ptr, attach point post in world frame and body frame, and its vel in world frame
//         // where UAV are connect to attach point through cable    
//         std::shared_ptr<UAVCable> ptr_UAVCable = v_mav_cable_ptr[i];

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
//             Eigen::Vector3d mav_post;
//             Eigen::Vector3d mav_vel;

//             ptr_UAVCable->cable_.ComputeCableDirection(attachpoint_post, mav_post);
//             ptr_UAVCable->cable_.GetCableDirection(cable_direction);

//             ptr_UAVCable->mav_.GetPosition(mav_post);
//             ptr_UAVCable->mav_.GetVel(mav_vel);


//             // 2) obtain drone mass
//             double mav_mass;
//             ptr_UAVCable->mav_.GetMass(mav_mass);

//             // 3) compute Ji for drones whose cables are taut
//             Eigen::MatrixXd Ji = ComputeMatrixJi(cable_direction, payload_attitude, attach_point_body_frame);

//             // 4) accumlate Ji * mi to J that is shown in Eq 45 
//             J = J + Ji * mav_mass;

//             // 5) compute bi for drones whose cables are taut
//             Eigen::VectorXd bi = ComputeVectorbi(mav_mass, mav_vel, cable_direction, payload_attitude, attach_point_body_frame);
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
    // for (size_t i = 0; i < v_mav_cable_ptr.size(); ++i) {
    //     pair_UAV_Cable_attachpoint.push_back(std::make_pair(v_mav_cable_ptr[i], v_attach_points_posts_body_frame_[i]));
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
    //         double mav_mass;
    //         ptr_UAVCable->mav_.GetMass(mav_mass);

    //         // 3) compute Ji for drones whose cables are taut
    //         Eigen::MatrixXd Ji = ComputeMatrixJi(xi, payload_attitude, attach_point);

    //         // 4) accumlate Ji * mi to J that is shown in Eq 45 
    //         J = J + Ji * mav_mass;
    //     }
    // }


    // for (const auto& mav_cable_ptr : v_mav_cable_ptr) {
    //     // mav_cable_ptr is a pointer points to an instance of UAVCable
    //     // (1) check if the cable of UAVCable is taut
    //     bool flag_cable_taut;
    //     mav_cable_ptr->cable_.CheckTaut(flag_cable_taut);


    //     if (flag_cable_taut == true) // the cable is taut 
    //     {
    //         // 1) compute cable direction xi
    //         Eigen::Vector3d xi;
    //         mav_cable_ptr->cable_.ComputeCableDirection();
    //         mav_cable_ptr->cable_.GetCableDirection(xi);

    //         // 2) obtain drone mass
    //         double mav_mass;
    //         mav_cable_ptr->mav_.GetMass(mav_mass);

    //         // 3) compute Ji for drones whose cables are taut
    //         Eigen::MatrixXd Ji = ComputeMatrixJi(xi, payload_attitude, attach_point_body_frame);

    //         // 4) accumlate Ji * mi to J that is shown in Eq 45 
    //         J = J + Ji * mav_mass;
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

Eigen::VectorXd Payload::ComputeVectorbi(const double &mav_mass, const Eigen::Vector3d &mav_vel, const Eigen::Vector3d &cable_direction, const Eigen::Quaterniond &payload_attitude, const Eigen::Vector3d &attach_point_body_frame)
{
    Eigen::VectorXd bi = Eigen::MatrixXd::Identity(6,1);

    Eigen::Vector3d bi_up = Eigen::MatrixXd::Identity(3,1);
    Eigen::Vector3d bi_down = Eigen::MatrixXd::Identity(3,1);

    // compute first three elements in bi in eq 42
    bi_up = mav_mass * cable_direction *cable_direction.transpose()*mav_vel;

    Eigen::Matrix3d payload_R = payload_attitude.toRotationMatrix(); // convert a quaternion to a 3x3 rotation matrix
    Eigen::Matrix3d hat_pho = TransVector3d2SkewSymMatrix(attach_point_body_frame);

    // compute last three elements in bi in eq 42
    bi_down = mav_mass * hat_pho * payload_R.transpose() * cable_direction *cable_direction.transpose()*mav_vel;

    bi.head<3>() = bi_up;
    bi.tail<3>() = bi_down;

    return bi;

}


void Payload::InputMassMatrix(const Eigen::Matrix3d &m_mass_matrix)
{
    m_mass_matrix_ = m_mass_matrix;
}

void Payload::InputDronesNetForces(const Eigen::Vector3d &drones_net_force, const Eigen::Matrix3d &m_D)
{
    drones_net_force_ = drones_net_force;

    m_D_ = m_D;

    std::cout<<"[----------] Payload::InputDronesNetForces mavs_net_force is "<< drones_net_force_.transpose()<<std::endl;   
}

void Payload::ComputeAccBodyRateAcc()
{
    Eigen::Vector3d payload_bodyrate{0,0,0};

    GetBodyrate(payload_bodyrate);

    // bodyrate acc
    auto payload_bodyrate_acc = ComputeRotDynamics(drones_net_force_, drones_net_torque_, m_mass_matrix_, payload_bodyrate, m_C_, m_D_, m_E_);

    // linear acc
    auto payload_acc = m_mass_matrix_.householderQr().solve(drones_net_force_ + m_D_ * payload_bodyrate_acc) - Eigen::Vector3d::UnitZ() * gravity_;

    SetAcc(payload_acc);

    SetBodyrateAcc(payload_bodyrate_acc);
};

void Payload::InputDronesNetTorques(const Eigen::Vector3d &drones_net_torque, const Eigen::Matrix3d &m_C, const Eigen::Matrix3d &m_E)
{
    drones_net_torque_ = drones_net_torque;

    m_C_ = m_C;

    m_E_ = m_E;
    
    std::cout<<"[----------] Payload::InputDronesNetForces mavs_net_torque is "<< drones_net_torque_.transpose()<<std::endl;       
}



Eigen::Vector3d Payload::ComputeTransDynamics(const Eigen::Vector3d &drones_net_forces, const Eigen::Matrix3d &mass_matrix, const Eigen::Matrix3d &m_D,  const Eigen::Vector3d &payload_bodyrate_acc)
{
    Eigen::Vector3d payload_acc(0,0,0);
    

    payload_acc = mass_matrix.householderQr().solve(drones_net_forces + m_D * payload_bodyrate_acc) - Eigen::Vector3d::UnitZ() * gravity_;

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

    Eigen::Matrix3d payload_interia;
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
    payload_bodyrate_acc = dxdt.tail(3);       

    // 1. translation in world frame
    // P = [x,y,z,dx, dy, dz]
    // dP = [dx, dy, dz, ddx, ddy, ddz]
    dxdt.head(3) = x.segment<3>(3);

    // [ddx ddy ddz] = 
    dxdt.segment<3>(3) = ComputeTransDynamics(drones_net_force_, m_mass_matrix_, m_D_, payload_bodyrate_acc);

    // 2. rotation in body frame
    Eigen::Matrix3d matrix_pdr2dEuler;
    matrix_pdr2dEuler = matirxBodyrate2EulerRate(x(6), x(7));

    // compute [dphi,   dtheta,     dpsi]^T =  matrix_pdr2dEuler * bodyrate
    dxdt.segment<3>(6) = matrix_pdr2dEuler * payload_bodyrate;

    // compute dp, dq ,dr
    dxdt.tail(3) =ComputeRotDynamics(drones_net_force_, drones_net_torque_, m_mass_matrix_, payload_bodyrate, m_C_, m_D_, m_E_);

    is_recursing = false;
}



// void Payload::doOneStepInt()
// {

//     auto state = RigidBody::state();

//     auto step_size = RigidBody::step_size();


//     this->stepper_.do_step(*this, state_, current_step_, RigidBody::step_size());

//     // acculate simulation steps
//     current_step_ = current_step_ +  RigidBody::step_size();

//     // // assign states to position and velocity
//     // assignPtMasState(ptmas_state_);

// }


Eigen::Matrix3d Payload::matirxBodyrate2EulerRate(const double &phi, const double &theta)
{
    Eigen::Matrix3d m_Bodyrate2EulerRate;

        m_Bodyrate2EulerRate(0,0) = 1;
        m_Bodyrate2EulerRate(0,1) = sin(phi)*tan(theta);
        m_Bodyrate2EulerRate(0,2) = cos(phi)*tan(theta);

        m_Bodyrate2EulerRate(1,0) = 0;
        m_Bodyrate2EulerRate(1,1) = cos(phi);
        m_Bodyrate2EulerRate(1,2) = -sin(phi);

        m_Bodyrate2EulerRate(2,0) = 0;
        m_Bodyrate2EulerRate(2,1) = sin(phi)/cos(theta);
        m_Bodyrate2EulerRate(2,2) = cos(phi)/cos(theta);

    return m_Bodyrate2EulerRate;
} 

