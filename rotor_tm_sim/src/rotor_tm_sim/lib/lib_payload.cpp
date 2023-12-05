#include "rotor_tm_sim/lib_payload.hpp"


Payload::Payload(const std::vector<Eigen::Vector3d> &v_attach_point_post_bf, const double &mass, const Eigen::Matrix3d &m_inertia, const double &step_size): RigidBody(mass, m_inertia, step_size), v_attach_point_posts_body_frame_(v_attach_point_post_bf)
{
    // reserve size of vectors v_attach_point_posts_ and v_attach_point_vels_
    v_attach_point_posts_.reserve(v_attach_point_posts_body_frame_.size());

    v_attach_point_vels_.reserve(v_attach_point_posts_body_frame_.size());
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




void Payload::UpdateMotion4AttachPoint()
{

    Eigen::Vector3d payload_position;

    Eigen::Quaterniond payload_attitude;
    // std::cout<<"fuck 1 "<<std::endl;
    getPosition(payload_position);

    getAttitude(payload_attitude);

    Eigen::Matrix3d payload_rotation_matrix = payload_attitude.toRotationMatrix();

    // std::cout<<"fuck 2 "<<std::endl;

    // std::for_each(v_attach_point_posts_body_frame_.begin(),
    //          v_attach_point_posts_body_frame_.end(),
    //          [](const Eigen::Vector3d& elem) {
 
    //              // printing one by one element
    //              // separated with space
    //              std::cout << "ele is "<<elem.transpose() << std::endl;
    //          });
    
    // compute attach point posts in world frame and store them in v_attach_point_posts_
    // formula: attach_point_world_frame = payload_position + (payload_rotation_matrix* attach_point_body_frame)
    // std::transform(v_attach_point_posts_body_frame_.begin(), v_attach_point_posts_body_frame_.end(), v_attach_point_posts_.begin(),[payload_position, payload_rotation_matrix](auto attach_point_body_frame) { //
    //                                                                             Eigen::Vector3d attach_point_world_frame;
    //                                                                             std::cout<<"fuck 2.5 "<<std::endl;
    //                                                                             attach_point_world_frame = payload_position + (payload_rotation_matrix* attach_point_body_frame);
                                                                                
    //                                                                             std::cout<<"fuck 2.7 "<<std::endl;
    //                                                                             std::cout << "ele is "<<attach_point_world_frame.transpose() << std::endl;


    //                                                                             return attach_point_world_frame ;
    //                                                                             // v_attach_point_posts_.push_back(attach_point_world_frame);

    //                                                                             });

    /*works*/
    // std::for_each(v_attach_point_posts_body_frame_.begin(),
    //          v_attach_point_posts_body_frame_.end(),
    //          [payload_position, payload_rotation_matrix, this](const Eigen::Vector3d& attach_point_body_frame) {

    //          // update post   
    //          Eigen::Vector3d attach_point_world_frame;
    //          attach_point_world_frame = payload_position + (payload_rotation_matrix* attach_point_body_frame);

    //          v_attach_point_posts_.push_back(attach_point_world_frame);

    //          });


    // compute vels of attach points
    Eigen::Vector3d payload_vel(0,0,0);    
    Eigen::Vector3d payload_bodyrate(0,0,0);  

    getBodyrate(payload_bodyrate);
    getVel(payload_vel);

    std::for_each(v_attach_point_posts_body_frame_.begin(),
             v_attach_point_posts_body_frame_.end(),
             [payload_position, payload_rotation_matrix, payload_bodyrate, payload_vel, this](const Eigen::Vector3d& attach_point_body_frame) {

             // update post   
             Eigen::Vector3d attach_point_post_world_frame;
             attach_point_post_world_frame = payload_position + (payload_rotation_matrix* attach_point_body_frame);
             v_attach_point_posts_.push_back(attach_point_post_world_frame);
            
            
            
            // update vels
            // pl_vel + pl_rot @ utilslib.vec2asym(x[10:13]) @ rho_vec_list

            Eigen::Vector3d attach_point_vel_world_frame;

            Eigen::Matrix3d m_skew_payload_bodyrate = TransVector3d2SkewSymMatrix(payload_bodyrate);
            
            attach_point_vel_world_frame = payload_vel + payload_rotation_matrix * m_skew_payload_bodyrate * attach_point_body_frame;
            // v_attach_point_vels_.push_back(attach_point_vel_world_frame);
            // 
             });




    
}

void Payload::getAttachPointPost(std::vector<Eigen::Vector3d> &v_attach_point_post) const
{

    // std::cout<<"fuck getAttach point1"<<std::endl;
    // std::for_each(v_attach_point_posts_.begin(),
    //          v_attach_point_posts_.end(),
    //          [](const Eigen::Vector3d& elem) {
 
    //              // printing one by one element
    //              // separated with space
    //              std::cout << "ele is "<<elem.transpose() << std::endl;
    //          });

    v_attach_point_post = v_attach_point_posts_;

    //  std::cout<<"fuck getAttach point 2"<<std::endl;
    // std::for_each(v_attach_point_post.begin(),
    //          v_attach_point_post.end(),
    //          [](const Eigen::Vector3d& elem) {
 
    //              // printing one by one element
    //              // separated with space
    //              std::cout << "ele is "<<elem.transpose() << std::endl;
    //          });

}


void Payload::UpdateVelCollided(const std::vector<std::shared_ptr<UAVCable>> v_drone_cable_ptr)
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

    Eigen::Matrix3d payload_rotation_matrix = payload_attitude.toRotationMatrix();

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
    if (v_drone_cable_ptr.size() != v_attach_point_posts_body_frame_.size())
    {
        std::cout<<"number of cable-drone != number of attach point"<<std::endl;
        return;
    }
    
    // get number of robots
    size_t number_robot = v_drone_cable_ptr.size();

    // iterate for each drone, cable, attach point
    for (size_t i = 0; i < number_robot; ++i) {

        // (1) obtain UAVCable ptr, attach point post in world frame and body frame, and its vel in world frame
        // where UAV are connect to attach point through cable    
        std::shared_ptr<UAVCable> ptr_UAVCable = v_drone_cable_ptr[i];

        Eigen::Vector3d attach_point = v_attach_point_posts_body_frame_[i];

        Eigen::Vector3d attachpoint_post = v_attach_point_posts_[i];
        Eigen::Vector3d attachpoint_vel  = v_attach_point_vels_[i];

        // (1) check if the cable of UAVCable is taut
        bool flag_cable_taut;
        ptr_UAVCable->CheckCollision(attachpoint_post, attachpoint_vel);


        if (flag_cable_taut == true) // the cable is taut 
        {
            // 1) compute cable direction xi
            Eigen::Vector3d xi;
            Eigen::Vector3d drone_post;

            ptr_UAVCable->drone_.getPosition(drone_post);
            ptr_UAVCable->cable_.ComputeCableDirection(attachpoint_post, drone_post);
            ptr_UAVCable->cable_.GetCableDirection(xi);

            // 2) obtain drone mass
            double drone_mass;
            ptr_UAVCable->drone_.GetMass(drone_mass);

            // 3) compute Ji for drones whose cables are taut
            Eigen::MatrixXd Ji = ComputeMatrixJi(xi, payload_attitude, attach_point);

            // 4) accumlate Ji * mi to J that is shown in Eq 45 
            J = J + Ji * drone_mass;
        }
    }




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
    //     pair_UAV_Cable_attachpoint.push_back(std::make_pair(v_drone_cable_ptr[i], v_attach_point_posts_body_frame_[i]));
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