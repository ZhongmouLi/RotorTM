#include "rotor_tm_sim/lib_payload.hpp"


Payload::Payload(const MassProperty &mass_property, const double &step_size): RigidBody(mass_property, step_size)
{
    

}



Payload::Payload(const MassProperty &mass_property, std::shared_ptr<Joint> v_ptr_joint, const double &step_size): RigidBody(mass_property, step_size)
{
    num_robot_ =1;
    v_ptr_joints_ = {v_ptr_joint};
};


Payload::Payload(const MassProperty &mass_property, std::vector<std::shared_ptr<Joint>> v_ptr_joints, const double &step_size): RigidBody(mass_property, step_size), v_ptr_joints_(v_ptr_joints)
{
    num_robot_ = v_ptr_joints_.size();

};


void Payload::SetJointInitPostBasedOnPayload()
{
    // obtain payload pose: post and attitude
    Eigen::Vector3d payload_position = pose().post;
    Eigen::Quaterniond payload_attitude = pose().att;

    Eigen::Matrix3d m_payload_rotation = payload_attitude.toRotationMatrix();

    // compute post, vel and acc of each attach point
    for (size_t i = 0; i < num_robot_; i++)    
    {
        Eigen::Vector3d joint_post_bodyframe = v_ptr_joints_.at(i)->post_body_frame();

        Pose joint_pose;
        joint_pose.post = payload_position + m_payload_rotation * joint_post_bodyframe;
        joint_pose.att = payload_attitude;
        v_ptr_joints_.at(i)->SetPose(joint_pose);
    }
}

void Payload::AddJointsLinkedWithUAVCable(const std::vector<std::shared_ptr<Joint>>& v_ptr_joints)
{

    // copy shared pointers of v_ptr_joints to v_ptr_joints_
    num_robot_ = v_ptr_joints.size();
    v_ptr_joints_.reserve(num_robot_);

    for (size_t i = 0; i < num_robot_; i++)
    {
        v_ptr_joints_.push_back(v_ptr_joints.at(i));
    }
    

}

void Payload::ComputeJointKinematics()
{

    std::cout<<std::string(4, ' ') <<"Enter Payload::ComputeJointKinematics()"<<std::endl;
    // obtain payload pose: post and attitude
    Eigen::Vector3d payload_position = pose().post;
    Eigen::Quaterniond payload_attitude = pose().att;


    Eigen::Matrix3d m_payload_rotation = payload_attitude.toRotationMatrix();


    // obtain payload vel and bodyrate
    Eigen::Vector3d payload_vel = vels().linear_vel;    
    Eigen::Vector3d payload_bodyrate = vels().bodyrate;  

    Eigen::Matrix3d m_skewsym_payload_bodyrate = TransVector3d2SkewSymMatrix(payload_bodyrate);

    // obtain payload acc
    Eigen::Vector3d payload_acc = accs().linear_acc;


    // obtain payload bodyrate acc
    Eigen::Vector3d payload_angular_acc = accs().angular_acc;  
    Eigen::Matrix3d m_skewsym_payload_bodyrate_acc = TransVector3d2SkewSymMatrix(payload_angular_acc);
    
    // std::cout<<std::string(4, ' ')<<"[----------] Payload: ComputeAttachPointsKinematics fuck inside 1"<<std::endl;
    // compute post, vel and acc of each attach point

    // for (auto it = v_ptr_joints_.begin(); it != v_ptr_joints_.end(); ++it) 
    for (size_t i = 0; i < num_robot_; i++)    
    {
    // Access the member of the unique_ptr<Joint> using *it
    // Example: (*it)->someMethod();

        Eigen::Vector3d joint_post_bodyframe = v_ptr_joints_.at(i)->post_body_frame();

        Pose joint_pose;
        joint_pose.post = payload_position + m_payload_rotation * joint_post_bodyframe;
        joint_pose.att = payload_attitude;
        v_ptr_joints_.at(i)->SetPose(joint_pose);

        // vels
        Vels joint_vels;
        joint_vels.linear_vel = payload_vel +  m_payload_rotation * m_skewsym_payload_bodyrate * joint_post_bodyframe;
        joint_vels.bodyrate = payload_bodyrate;
        v_ptr_joints_.at(i)->SetVels(joint_vels);

        // accs
        Accs joint_accs;

        Eigen::Vector3d joint_centri_acc =  m_skewsym_payload_bodyrate * (m_skewsym_payload_bodyrate * joint_post_bodyframe);

        //
        //  self.attach_accel = self.pl_accel + np.array([0,0,self.pl_params.grav]) + np.matmul(pl_rot, np.matmul(utilslib.vec2asym(self.pl_ang_accel), self.rho_vec_list)).T + np.matmul(pl_rot, attach_centrifugal_accel).T

        joint_accs.linear_acc = payload_acc + (Eigen::Vector3d::UnitZ() * gravity_) + (m_payload_rotation * (m_skewsym_payload_bodyrate_acc * joint_post_bodyframe)) + (m_payload_rotation * joint_centri_acc);      
          
        joint_accs.angular_acc = payload_angular_acc;
        v_ptr_joints_.at(i)->SetAccs(joint_accs);


        std::cout<<std::string(4, ' ')<<"joint " << i <<std::endl;
        std::cout<<std::string(4, ' ') <<"payload_position is "<< payload_position.transpose()<<std::endl;
        std::cout<<std::string(4, ' ') <<"m_payload_rotation is "<< m_payload_rotation.transpose().reshaped().transpose()<<std::endl;
        std::cout<<std::string(4, ' ') <<"joint_post_bodyframe is "<< joint_post_bodyframe.transpose()<<std::endl;
        std::cout<<std::string(4, ' ') <<"joint_pose.post is "<< joint_pose.post.transpose()<<std::endl;
        
        
        
        // std::cout<<std::string(4, ' ') <<"-------------Problem---------------"<<std::endl;

        std::cout<<std::string(4, ' ') <<"payload_acc is "<< payload_acc.transpose()<<std::endl;
        std::cout<<std::string(4, ' ') <<"payload_angular_acc is "<< payload_angular_acc.transpose()<<std::endl;
        std::cout<<std::string(4, ' ') <<"m_skewsym_payload_bodyrate_acc is "<< m_skewsym_payload_bodyrate_acc.transpose().reshaped().transpose()<<std::endl;


        auto term1 = m_payload_rotation * (m_skewsym_payload_bodyrate_acc * joint_post_bodyframe);
        std::cout<<std::string(4, ' ') <<"term1 is "<< term1.transpose()<<std::endl;

        auto term2 = m_payload_rotation * joint_centri_acc;
        std::cout<<std::string(4, ' ') <<"term2 is "<< term2.transpose()<<std::endl;
        std::cout<<std::string(4, ' ') <<"joint_centri_acc is "<< joint_centri_acc.transpose()<<std::endl;
        // std::cout<<std::string(4, ' ') <<"m_payload_rotation is "<< m_payload_rotation<<std::endl;
        // std::cout<<std::string(4, ' ') <<"m_skewsym_payload_bodyrate_acc is "<< m_skewsym_payload_bodyrate_acc<<std::endl;
        // std::cout<<std::string(4, ' ') <<"joint_post_bodyframe is "<< joint_post_bodyframe.transpose()<<std::endl;



        
        std::cout<<std::string(4, ' ') <<"joint_accs.linear_acc is "<< joint_accs.linear_acc.transpose()<<std::endl;
        std::cout<<std::string(4, ' ') <<"joint_accs.angular_acc is "<< joint_accs.angular_acc.transpose()<<std::endl;

        // std::cout<<std::string(4, ' ') <<"-------------Problem---------------"<<std::endl;
        std::cout<<std::string(4, ' ') <<"Leave Payload::ComputeJointKinematics()"<<std::endl;

    }

    

}






/*----------------------------Collision--------------------------------------*/
// UpdateVelCollided computs updated vel of payload after collision
//write a unit test for this function
void Payload::UpdateVelCollided()
{
    // 1. define matrix J and b in J [xL+, omegaL+] = b
    Eigen::MatrixXd J = Eigen::MatrixXd::Identity(6,6);
    Eigen::VectorXd b = Eigen::MatrixXd::Identity(6,1);

    // 2. initialisation of J and b
    // . J[1:3, 1:3] = mL * I3
    J.block<3,3>(0,0) = Eigen::MatrixXd::Identity(3,3) * mass();
    // J[4:6, 4:6] = IL
    // J.block<3,3>(3,3) = payload_inertia;
    J.block<3,3>(3,3) = inertia();

    // 2.3 initialisation of b
    // left variable of b in Eq 42 that depend on payload
    // b.head<3>() = payload_vel * payload_mass;
    // b.tail<3>() = payload_inertia * payload_bodyrate;
    b.head<3>() = mass() * vels().linear_vel;
    b.tail<3>() = inertia() * vels().bodyrate;


    // 3 compute all Ji in Eq45 and Eq46 for every drone that is collided
    // iterate for each drone, cable, attach point
    // take only drones whose cables are taut
    for (size_t i = 0; i < num_robot_; ++i) {

        // (1) obtain UAVCable ptr, attach point post in world frame and body frame, and its vel in world frame
        // where Quadrotor are connect to attach point through cable  

        auto& ptr_joint = v_ptr_joints_[i];

        // UAVCable UAVCable = v_mav_cable[i];
        const auto& ptr_UAVCable = ptr_joint->ptr_UAVCable();

        // Eigen::Vector3d attach_point_body_frame = v_attach_points_posts_body_frame_[i];

        // Eigen::Vector3d attachpoint_post = v_attach_points_posts_[i];
        // Eigen::Vector3d attachpoint_vel  = v_attach_points_vels_[i];

        // (1) check if the cable of UAVCable is taut
        // bool flag_cable_taut = false;
        // UAVCable.CheckCollision(attachpoint_post, attachpoint_vel);

        if (ptr_UAVCable->inelasticCollisionStauts() == true) // cable is taut
        {

            // 3) compute Ji for drones whose cables are taut
            // Eigen::MatrixXd Ji = ComputeMatrixJi(cable_direction, payload_attitude, attach_point_body_frame);
            // Eigen::MatrixXd Ji = ComputeMatrixJi(ptr_UAVCable->cable_.direction(), pose().att, ptr_joint->post_body_frame());

            // 3) compute Ji with cable, mav and joint as arguments
            Eigen::MatrixXd Ji = ComputeMatrixJi(ptr_UAVCable->cable_, ptr_joint);
            // Eigen::MatrixXd Ji = ComputeMatrixJi(ptr_UAVCable->cable_.direction(), pose().att, ptr_joint->post_body_frame());


            // 4) accumlate Ji * mi to J that is shown in Eq 45 
            // J = J + Ji * mav_mass;
            J = J + Ji * ptr_UAVCable->mav_.mass();

            // 5) compute bi for drones whose cables are taut
            Eigen::VectorXd bi = ComputeVectorbi(ptr_UAVCable->mav_, ptr_UAVCable->cable_, ptr_joint);

            b = b + bi;
        }

        // payload_vel_bodyrate_collised =[vel; bodyrate]
        Eigen::VectorXd payload_vel_bodyrate_collised = Eigen::MatrixXd::Zero(6, 1);

        // solve J [vel; bodyrate] = b in Eq.42
        // payload_vel_bodyrate_collised = J.householderQr().solve(b);
        payload_vel_bodyrate_collised = J.colPivHouseholderQr().solve(b);
        // payload_vel_bodyrate_collised = J.inverse() * b;


        SetLinearVel(payload_vel_bodyrate_collised.head(3));

        SetBodyrate(payload_vel_bodyrate_collised.tail(3));
    }






}

// add explain of function ComputeMatrixJi  
// ComputeMatrixJi computes matrix Ji in Eq45
// Ji = ai * ai^T + bi * bi^T
Eigen::MatrixXd Payload::ComputeMatrixJi(const Cable &cable, const std::shared_ptr<Joint> &ptr_joint)
{
    // define matrix Ji in Eq45    
    Eigen::MatrixXd Ji = Eigen::MatrixXd::Identity(6,6);

    // compute ai in Eq 43
    // xi = cable_direction
    Eigen::Matrix3d ai = cable.direction() *cable.direction().transpose();
    
    // compute bi in Eq 43 and 44
    Eigen::Matrix3d hat_pho = TransVector3d2SkewSymMatrix(ptr_joint->post_body_frame());
    Eigen::Matrix3d payload_R = pose().att.toRotationMatrix(); // convert a quaternion to a 3x3 rotation matrix

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
};

// add explaination of function ComputeVectorbi
// ComputeVectorbi computes vector bi in Eq 42
Eigen::VectorXd Payload::ComputeVectorbi(const Quadrotor &mav, const Cable &cable, const std::shared_ptr<Joint> &ptr_joint)
{
    Eigen::VectorXd bi = Eigen::MatrixXd::Identity(6,1);

    Eigen::Vector3d bi_up = Eigen::MatrixXd::Identity(3,1);
    Eigen::Vector3d bi_down = Eigen::MatrixXd::Identity(3,1);

    // compute first three elements in bi in eq 42
    bi_up = mav.mass() * cable.direction() *cable.direction().transpose()*mav.vels().linear_vel;

    Eigen::Matrix3d payload_R = pose().att.toRotationMatrix(); // convert a quaternion to a 3x3 rotation matrix

    Eigen::Matrix3d hat_pho = TransVector3d2SkewSymMatrix(ptr_joint->post_body_frame());

    // compute last three elements in bi in eq 42
    bi_down = mav.mass() * hat_pho * payload_R.transpose() * cable.direction() *cable.direction().transpose()*mav.vels().linear_vel;

    bi.head<3>() = bi_up;
    bi.tail<3>() = bi_down;

    return bi;

}



void Payload::SetInitialAccBodyRateAcc(const Eigen::Vector3d &payload_initial_acc)
{
    if(!intial_acc_set_)
    {
        SetLinearAcc(payload_initial_acc);
        SetAngularAcc(Eigen::Vector3d::Zero());
        intial_acc_set_ =  true;
    }
}



void Payload::InputDronesNetWrenches(const Wrench &mavs_net_wrench)
{
    mavs_net_wrench_ = mavs_net_wrench;
}

void Payload::InputPayloadInteractPara(const CooperIntertPara &cooper_interact_para)
{

    cooper_interact_para_ = cooper_interact_para;
};




void Payload::operator() (const object_state &x , object_state &dxdt, const double time [[maybe_unused]])
{

    // static bool is_recursing = false;
    // if (is_recursing) return;  // Prevent recursion
    // is_recursing = true;
    
    // std::cout<<std::string(4, ' ') << "state of payload" << x.transpose()<<std::endl; 
    // x =  [x,     y,      z,      dx,     dy,     dz,     phi,    theta,      psi,    p,      q,      r]
    // dxdt = [dx,    dy,     dz,     ddx,    ddy,    ddz,    dphi,   dtheta,     dpsi,   dp,     dq,     dr]


    // obtain bodyrate [p,      q,      r]
    // Eigen::Vector3d payload_bodyrate;
    // payload_bodyrate = x.tail(3);    
    Eigen::Vector3d payload_bodyrate(x.at(10), x.at(11), x.at(12));

    // obtain bodyrate acc [ dp,     dq,     dr]
    // Eigen::Vector3d payload_angular_acc;
    // payload_angular_acc = dxdt.tail(3);    
    Eigen::Vector3d payload_angular_acc(dxdt.at(10), dxdt.at(11), dxdt.at(12));   

    // 1. translation in world frame
    // P = [x,y,z,dx, dy, dz]
    // dP = [dx, dy, dz, ddx, ddy, ddz]
    // dxdt.head(3) = x.segment<3>(3);
    // dxdt.at(0) = x.at(3);
    // dxdt.at(1) = x.at(4);
    // dxdt.at(2) = x.at(5);

    // // [ddx ddy ddz] = 
    // // dxdt.segment<3>(3) = ComputeTransDynamics(drones_net_force_, m_mass_matrix_, m_D_, payload_angular_acc);

    // // dxdt.segment<3>(3) =ComputeTransDynamics();
    // dxdt.segment<3>(3) =ComputeTransDynamics(payload_angular_acc);
    dxdt.at(0) = x.at(3);
    dxdt.at(1) = x.at(4);
    dxdt.at(2) = x.at(5);

    auto ddx = ComputeTransDynamics(payload_angular_acc);
    dxdt.at(3) = ddx[0];
    dxdt.at(4) = ddx[1];
    dxdt.at(5) = ddx[2];



    // map bodyrate to quaternion derivative
    // current att in quaternion
    Eigen::Quaterniond qn(x.at(6), x.at(7), x.at(8), x.at(9));
    qn.normalize();

    // convert bodyrate into quaternion
    // define bodyrate
    // Eigen::Vector3d bodyrate;
    // bodyrate = x.tail(3);    
    Eigen::Vector3d bodyrate(x.at(10), x.at(11), x.at(12));
    // Eigen::Quaterniond omega(0, bodyrate[0], bodyrate[1], bodyrate[2]);

    // // compute quaternion derivative
    // Eigen::Quaterniond dqn = 0.5 * ( omega * qn);
    // dqn.normalize();
    auto dqn = ComputeQuaternionDerivative(qn, bodyrate);

    // dxdt[6] = dqn(0);
    // dxdt[7] = dqn(1);
    // dxdt[8] = dqn(2);
    // dxdt[9] = dqn(3);

    dxdt.at(6) = dqn(0);
    dxdt.at(7) = dqn(1);
    dxdt.at(8) = dqn(2);
    dxdt.at(9) = dqn(3);

    
    // compute dp, dq ,dr
    // dxdt.tail(3) =ComputeRotDynamics(bodyrate);
    auto dpqr = ComputeRotDynamics(bodyrate);
    
    dxdt.at(10) = dpqr[0];
    dxdt.at(11) = dpqr[1];
    dxdt.at(12) = dpqr[2];    



    double qw = state_.at(6), qx = state_.at(7), qy = state_.at(8), qz = state_.at(9);
    double norm = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
    state_.at(6) /= norm;
    state_.at(7) /= norm;
    state_.at(8) /= norm;
    state_.at(9) /= norm;
    // NormalizeQuaternion(dxdt, 6);  // Assuming quaternion starts at index 6

    // std::cout<<std::string(4, ' ')<<"fuck payload post" << x.head(3).transpose() <<std::endl;
    // std::cout<<std::string(4, ' ')<<"fuck payload acc" <<  dxdt.segment<3>(3).transpose() <<std::endl;

    // is_recursing = false;

    // current_step_ = current_step_ + 
    // save payload linear acc and angular acc
    Eigen::Vector3d linear_acc(dxdt.at(3), dxdt.at(4), dxdt.at(5));
    SetLinearAcc(linear_acc);
    Eigen::Vector3d angular_acc(dxdt.at(10), dxdt.at(11), dxdt.at(12));
    SetAngularAcc(angular_acc);

    // std::cout<<"fuck hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh"<<std::endl;
    std::cout<<"    "<< "payload inte is called"<<std::endl;

}



Eigen::Vector3d Payload::ComputeTransDynamics(const Eigen::Vector3d &payload_angular_acc)
{
    Eigen::Vector3d payload_acc(0,0,0);
    

    // payload_acc = mass_matrix.householderQr().solve(drones_net_forces + m_D * payload_angular_acc) - Eigen::Vector3d::UnitZ() * gravity_;

    payload_acc = cooper_interact_para_.m_mass_matrix.colPivHouseholderQr().solve(mavs_net_wrench_.force + cooper_interact_para_.m_D * accs().angular_acc) - Eigen::Vector3d::UnitZ() * gravity_;

    // payload_acc = cooper_interact_para_.m_mass_matrix.inverse() * (mavs_net_wrench_.force + cooper_interact_para_.m_D * payload_angular_acc)  - Eigen::Vector3d::UnitZ() * gravity_;
    
    // SetAcc(payload_acc);

    return payload_acc;
}

// // Eigen::Vector3d Payload::ComputeRotDynamics(const Eigen::Vector3d &drones_net_forces, const Eigen::Vector3d &drones_net_torques, const Eigen::Matrix3d &m_mass_matrix, const Eigen::Vector3d &payload_bodyrate, const Eigen::Matrix3d &m_C, const Eigen::Matrix3d &m_D, const Eigen::Matrix3d &m_E)

Eigen::Vector3d Payload::ComputeRotDynamics(const Eigen::Vector3d &payload_bodyrate)
{

    // setp 1. compute effective torque for the payload
    // such that 
    Eigen::Vector3d torque_effective{0,0,0};

    // Eigen::Matrix3d payload_interia;
    // GetInertia(payload_interia);

    // Eigen::Matrix3d inv_m_mass_matrix = cooper_interact_para_.m_mass_matrix.inverse();

    // torque_effective = mavs_net_wrench_.torque - cooper_interact_para_.m_C * inv_m_mass_matrix *  mavs_net_wrench_.force - TransVector3d2SkewSymMatrix(payload_bodyrate) * inertia() * payload_bodyrate;
    torque_effective = mavs_net_wrench_.torque - cooper_interact_para_.m_C * cooper_interact_para_.m_mass_matrix.colPivHouseholderQr().solve(  mavs_net_wrench_.force) - TransVector3d2SkewSymMatrix(payload_bodyrate) * inertia() * payload_bodyrate;

    // step 2. compute effective inertia
    Eigen::Matrix3d interia_effective;

    // effective_inertia = self.pl_params.I + np.matmul(C, np.matmul(invML, D)) - E
    
    // interia_effective = inertia() + cooper_interact_para_.m_C * (inv_m_mass_matrix * cooper_interact_para_.m_D) - cooper_interact_para_.m_E;
    interia_effective = inertia() + cooper_interact_para_.m_C * (cooper_interact_para_.m_mass_matrix.colPivHouseholderQr().solve( cooper_interact_para_.m_D)) - cooper_interact_para_.m_E;


    // step 3 compute bodyrate acc

    Eigen::Vector3d bodyrate_acc;

    bodyrate_acc =  interia_effective.colPivHouseholderQr().solve(torque_effective);
    //  bodyrate_acc =  interia_effective.inverse() * torque_effective;


    // return bodyrate_acc;
    // SetBodyrateAcc(bodyrate_acc);

    return bodyrate_acc;
}






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

