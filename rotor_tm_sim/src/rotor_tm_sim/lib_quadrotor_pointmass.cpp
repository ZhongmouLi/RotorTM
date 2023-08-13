#include "rotor_tm_sim/lib_quadrotor_pointmass.hpp"



rotorTMQuadrotorPointMass::rotorTMQuadrotorPointMass(const double &UAV_mass, const Eigen::Matrix3d &m_inertia, const double &pd_mass, const double &cable_length, const double &step_size): mav_mass_(UAV_mass), payload_mass_(pd_mass), cable_length_(cable_length), step_size_(step_size)
{
    quadrotor = std::make_shared<QuadrotorDynamicSimulator>(UAV_mass, m_inertia, step_size);

    pm_payload = std::make_shared<PointMassDynamicSimulator>(pd_mass, step_size);
};




void rotorTMQuadrotorPointMass::inputMAVThrust(const double &mav_thrust)
{
        // obtain mav attitude
        Eigen::Quaterniond mav_attitude;
        quadrotor->getAttitude(mav_attitude); 

        // compute mav thrust force expressed in world frame
        mav_thrust_force_ = mav_thrust * (mav_attitude.toRotationMatrix()*Eigen::Vector3d::UnitZ());
};



std::pair<Eigen::Vector3d, Eigen::Vector3d> rotorTMQuadrotorPointMass::updateVel4CableCollision(const Eigen::Vector3d &mav_position, const Eigen::Vector3d &payload_position, const Eigen::Vector3d &mav_vel, const Eigen::Vector3d &payload_vel)
{
    // 1. get position and vel of mav and payload
    // Eigen::Vector3d mav_position, payload_position;
    // Eigen::Vector3d mav_vel, payload_vel;


    // quadrotor->getPosition(mav_position);
    // pm_payload->getPosition(payload_position);

    // quadrotor->getVel(mav_vel);
    // pm_payload->getVel(payload_vel);    

    // 1. define reuslts in pair
    // vels_collision is a pair containing vels of quadrotor and payload
    // quadrotor's vel = vels_collision.first
    // payloadd's vel = vels_collision.second

    std::pair<Eigen::Vector3d, Eigen::Vector3d> vels_collision;

    double distance = (mav_position - payload_position).norm();

    // 2. comput cable direction
    Eigen::Vector3d cable_dir = (mav_position - payload_position)/distance;

    double cable_dir_projmat = cable_dir.dot(cable_dir);

    // prejection of mav and payload's vel
    Eigen::Vector3d payload_vel_pre = cable_dir_projmat * payload_vel;

    Eigen::Vector3d mav_vel_pre = cable_dir_projmat * mav_vel;

    // 3. comput new vels
    Eigen::Vector3d v = (payload_mass_ * payload_vel_pre + mav_mass_ * mav_vel)/ (payload_mass_ + mav_mass_);

    // compute new vels after collision
    // payload_vel = v + payload_vel - payload_vel_pre;
    // mav_vel = v + mav_vel - mav_vel_pre;

    vels_collision.first = v + mav_vel - mav_vel_pre;
    vels_collision.second = v + payload_vel - payload_vel_pre;

    // update vels of quadrotor and payload
    // quadrotor->setVel(mav_vel);
    // pm_payload->setVel(payload_vel);

    return vels_collision;

};


void rotorTMQuadrotorPointMass::setInitPost(const Eigen::Vector3d &payload_init_position)
{
    // std::cout<< "payload int post" <<  payload_init_position.transpose()<< std::endl;
    pm_payload->setInitialPost(payload_init_position);

    Eigen::Vector3d mav_initial_post = payload_init_position + cable_length_ * Eigen::Vector3d::UnitZ();

    // std::cout<< "cable vector" <<  cable_length_ * Eigen::Vector3d::UnitZ().transpose()<< std::endl;


    //std::cout<< "mav int post from payload" <<  mav_initial_post.transpose()<< std::endl;

    quadrotor->setInitialPost(mav_initial_post);

}; 


bool rotorTMQuadrotorPointMass::isSlack(const Eigen::Vector3d &mav_position, const Eigen::Vector3d &payload_position)
{
    //   if (np.linalg.norm(robot_pos - attach_pos) > (cable_length - 1e-3)):
    //     return np.array([0.0])
    //   else:
    //     return np.array([1.0])

    double distance = (mav_position - payload_position).norm();

    if (distance > (cable_length_ - 1e-3))
        return false;
    else
        return true;


}

Eigen::Vector3d rotorTMQuadrotorPointMass::computeTensionForce(const Eigen::Vector3d &mav_position, const Eigen::Vector3d &payload_position, const Eigen::Vector3d &mav_vel, const Eigen::Vector3d &payload_vel)
{

        // 1. get position and vel of mav and payload
        // Eigen::Vector3d mav_position, payload_position;
        // Eigen::Vector3d mav_vel, payload_vel;

        // quadrotor->getPosition(mav_position);
        // pm_payload->getPosition(payload_position);

        // step 1.2 compute xi and xi_dot
        Eigen::Vector3d xi = (payload_position - mav_position)/cable_length_;
        Eigen::Vector3d xi_dot = (payload_vel - mav_vel)/cable_length_;

        // step 1.3 compute xi_omega
        Eigen::Vector3d xi_omega = xi.cross(xi_dot);


        // step 2 compute thrust force expressed in world frame, centrifugal force and tension vector

        // step 2.1 thrust force
        // this is done in inputMAVThrust

        // step 2.2 centrifugal force
        // note transpose is not necessary for Eigen vector
        double centrif = xi_omega.dot(xi_omega) * payload_mass_ * cable_length_;

   
        // step 2.3 tension vector
        // note that use -xi.dot(thrust_force) instead of -xi.transpose * thrust_force, as the second means matrix manipulation for a 1X3 vector and a 3X1 vector and it is the same to apply` dot product to two vectors in Eigen
        Eigen::Vector3d tension_force = payload_mass_ * (-xi.dot(mav_thrust_force_) + centrif) * xi / (mav_mass_ + payload_mass_);

        return tension_force;
    
}

void rotorTMQuadrotorPointMass::doOneStepint()
{
    // step 1. get quadrotor and payload's position and vel
    Eigen::Vector3d mav_position, payload_position;
    Eigen::Vector3d mav_vel, payload_vel;


    quadrotor->getPosition(mav_position);
    pm_payload->getPosition(payload_position);

    quadrotor->getVel(mav_vel);
    pm_payload->getVel(payload_vel);   

    // step 2 check collision
    // if yes, vels of mav and point mass will be modified
                    //     cable_norm_vel = np.transpose(pl_pos - robot_pos) @ (pl_vel - robot_vel)/np.linalg.norm(pl_pos - robot_pos)
                    // ## if collision, compute new velocities and assign to state
                    // if cable_norm_vel > 1e-3 and not self.cable_is_slack:
                    //     print("Colliding")
                    //     v1, v2 = self.ptmass_inelastic_cable_collision(x[0:6], x[13:19], self.pl_params.mass, self.uav_params[0].mass)
                    //     x[3:6] = v1
                    //     x[16:19] = v2
    double distance = (mav_position - payload_position).norm();
    double cable_norm_vel = (payload_position-mav_position).dot(payload_vel-mav_vel)/distance;

    if(cable_norm_vel>1e-3 && !cable_is_slack_)
    {
        std::pair<Eigen::Vector3d, Eigen::Vector3d> vels_collision = updateVel4CableCollision(mav_position, payload_position, mav_vel, payload_vel);
        quadrotor->setVel(vels_collision.first);
        pm_payload->setVel(vels_collision.second);
    }

    // step 3 decide inputs for mav and point mass based on cable's status
    if (cable_is_slack_ == true)
        {
            // step 2.1 cable is slack
            // mav and payload are two independent systems
            // payload suffer only gravity
            pm_payload->inputForce(Eigen::Vector3d::Zero());

            quadrotor->inputForce(mav_thrust_force_);
        }
    else
        {
            // step 2.2 cable is taut
            // mav and payload is a whole system
            // they are coupled on translations that is represented by the interaction force, i.e. tension force
            // note mav's attitude is independent

            tension_force_ = computeTensionForce(mav_position, payload_position, mav_vel, payload_vel);

            quadrotor->inputForce(mav_thrust_force_+tension_force_);        

            pm_payload->inputForce(-tension_force_);
        }

    // input for quadrotor's attitude that is independent no mater cable is slack or taut
    quadrotor->inputTorque(mav_torque_);


    // step 3 call one step integration
    pm_payload->doOneStepInt();
    quadrotor->doOneStepInt();

    // update current step
    current_step_ = current_step_ + step_size_;

    // step 4 compute cable's status and update cable_is_slack_
    cable_is_slack_ = isSlack(mav_position, payload_position);

    // self.cable_is_slack = self.isslack(x[0:3], x[13:16], self.pl_params.cable_length)    
    //                     pl_pos = x[0:3]
    //                 pl_vel = x[3:6]
    //                 robot_pos = x[13:16]
    //                 robot_vel = x[16:19]
};