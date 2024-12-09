#include "rotor_tm_sim/base/lib_cable.hpp"
#include <iostream>



Cable::Cable(const double &length):length_(length)
{

};

void Cable::SetDirection(const Eigen::Vector3d &xi)
{
    xi_ = xi;
};

void Cable::ComputeCableDirection(const Eigen::Vector3d &attachpoint_post, const Eigen::Vector3d &robot_post)
{

    // xi = (attach_pos - robot_pos) / np.linalg.norm(attach_pos - robot_pos, 2, 0)
    double distance = (attachpoint_post - robot_post).norm();

    xi_ = (attachpoint_post - robot_post)/distance;

    //  std::cout<<std::string(8, ' ')<< "attachpoint_post is " << attachpoint_post.transpose() <<std::endl;
    //  std::cout<<std::string(8, ' ')<< "robot_post is " << robot_post.transpose() <<std::endl;
    //  std::cout<<std::string(8, ' ')<< "distance is " << distance <<std::endl;

};


void Cable::CheckTaut(const Eigen::Vector3d &attachpoint_post, const Eigen::Vector3d &robot_post)
{

    // 1. compute the distance between attach point and robot
    double distance = (attachpoint_post - robot_post).norm();

    // flag = (np.linalg.norm(robot_pos - attach_pos, 2, 0) > (cable_length - 1e-3))
    bool flag_distance = (  std::abs(distance) > (length_ - k_threshold) ); 

    // 3. final taut is true if relative vel is positive and distance = cable length
    // i.e. both of flag_relative_vel and flag_distance must be true

    taut_ = flag_distance;

    //  std::cout<<std::string(8, ' ')<<"flag_relative_vel is " << flag_relative_vel<<std::endl;
    //  std::cout<<std::string(8, ' ')<<"flag_distance is " << flag_distance<<std::endl;    
}


// void Cable::CheckCollision(const Eigen::Vector3d &attachpoint_post, const Eigen::Vector3d &robot_post, const Eigen::Vector3d &attachpoint_vel, const Eigen::Vector3d &robot_vel)
// {

//     // // 1. compute the projection of relative vel of robot to attach point on cable  whose direction is represented by xi
//     // double vel_robot2attachpoint_projected_xi_direction;

//     // Eigen::Vector3d vel_robot2attachpoint = attachpoint_vel - robot_vel;

//     // vel_robot2attachpoint_projected_xi_direction = xi_.dot(vel_robot2attachpoint);

//     // //  std::cout<<std::string(8, ' ')<<"relative vel is " << vel_robot2attachpoint_projected_xi_direction<<std::endl;
    
//     // bool flag_relative_vel = (vel_robot2attachpoint_projected_xi_direction >= 0);

//     // // 2. check taut condition
//     // CheckTaut(attachpoint_post, robot_post);

//     // // 3. final taut is true if relative vel is positive and distance = cable length
//     // // i.e. both of flag_relative_vel and flag_distance must be true

//     // inelastic_collision_ = flag_relative_vel && taut_;

//     //  std::cout<<std::string(8, ' ')<<"flag_relative_vel is " << flag_relative_vel<<std::endl;
//     //  std::cout<<std::string(8, ' ')<<"flag_distance is " << flag_distance<<std::endl;    
// }

void Cable::ComputeCableTensionForce(const double &mav_mass, const Eigen::Vector3d &mav_thrust_force, const Eigen::Vector3d &attach_point_acc)
{

    // if (taut_ == false)
    // {
    //     std::cout<<std::string(8, ' ')<<"WARN: the cable is slack and there is no tension"<<std::endl;

    //    return;
    // }
    
    // python code to compute tension force for each cable
    // tension = self.uav_params[uav_idx].mass*cable_len*np.sum(cbl_omg**2) - np.matmul(xi, qd_u - self.uav_params[uav_idx].mass * self.attach_accel[uav_idx,:])
    
    // tension_vector[:,uav_idx] = tension * xi

    //for a special case one quadrotor, one cable and one paylaod, on a vertical direction
    // m_q = 1kg, m_p = 0.5kg
    // u|| = 2*9.8N = 19.6N
    // a_whole = 3.2667 m/s^2 // acc for the whole system (quadrotor + cable+ payload)
    //
    // xi = [0 0 -1]^T
    // Newton's law for payload
    // -tension_force =  m_p * (9.8+a_whole) = 6.4807N ===> tension_force =  -6.5334N

    // tension_force = - [0 0 -1] * (u|| - m_d * ( 9.8+a_whole) ) = -6.5334


    // 1. compute tension
    if (taut_ == true)
    {
        // double tension =0;
        // tension = self.uav_params[uav_idx].mass*cable_len*np.sum(cbl_omg**2) - \
        //                 np.matmul(xi, qd_u - self.uav_params[uav_idx].mass * self.attach_accel[uav_idx,:])
        //       tension_vector[:,uav_idx] = tension * xi
        tension_ = mav_mass * length_ *  body_rate_.squaredNorm() - xi_.dot(mav_thrust_force - mav_mass * attach_point_acc);

      
        // 2. comupute tension force
        tension_force_ = tension_ * xi_;

        // std::cout << std::scientific << std::setprecision(20);
        // std::cout<< "mav_mass is "<< mav_mass << std::endl;
        // std::cout<< "length_ is "<< length_ << std::endl;
        // std::cout << " body_rate_.squaredNorm(): " <<  body_rate_.squaredNorm() << std::endl;
        // std::cout << "mav_thrust_force: " << mav_thrust_force.transpose() << std::endl;       
        // std::cout << "attach_point_acc: " << attach_point_acc.transpose() << std::endl;
        // std::cout << "xi_: " << xi_.transpose() << std::endl;
        // correct
        // mav_mass is 2.50000000000000000000e-01
        // length_ is 5.00000000000000000000e-01
        // body_rate_.squaredNorm(): 4.45057385184420950530e-01
        // mav_thrust_force: 2.42918119593411557799e-01 1.54301568306383835028e-01 3.06756500072247906274e+00
        // attach_point_acc: 1.83630674805819893436e+00 3.60526111173580021685e-01 9.56372385915080158725e+00
        // xi_: -1.51739359394178235929e-01 -5.19497254434936953094e-02 -9.87054402166866351465e-01


// uav 0's qd_u is  [2.4291811959341156e-1 0.15430156830638384 3.067565000722479  ]

        // wrong one
        // mav_mass is 2.50000000000000000000e-01
        // length_ is 5.00000000000000000000e-01
        // body_rate_.squaredNorm(): 4.45057385184420950530e-01
        // mav_thrust_force: 2.42918146976459220499e-01 1.54301563801073182702e-01 3.06756499878065813647e+00
        // attach_point_acc: 1.83630674805819893436e+00 3.60526111173580021685e-01 9.56372385915080158725e+00
        // xi_: -1.51739359394178235929e-01 -5.19497254434936953094e-02 -9.87054402166866351465e-01



    }
    else
    {
        std::cout<<std::string(8, ' ')<<"WARN: the cable is slack and there is no tension"<<std::endl;
        tension_force_ = Eigen::Vector3d::Zero();
    }
    

}


void Cable::ComputeCableBodyrate(const Eigen::Vector3d &robot_vel, const Eigen::Vector3d &attachpoint_vel)
{
    // obtain unit vector of relative vel of attach point to mav
    Eigen::Vector3d relative_vel_unit = (attachpoint_vel - robot_vel)/length_;

    // 
    body_rate_ = xi_.cross(relative_vel_unit);

    // std::cout << std::scientific << std::setprecision(20);
    // std::cout<< "attachpoint_vel is "<< attachpoint_vel.transpose() << std::endl;
    // std::cout<< "robot_vel is "<< robot_vel.transpose() << std::endl;
    // std::cout << "relative_vel_unit: " << relative_vel_unit.transpose() << std::endl;
    // std::cout << "xi_: " << xi_.transpose() << std::endl;
    // std::cout << "body_rate_: " << body_rate_.transpose() << std::endl;

// relative_vel_unit:  1.22030604771391798513e-01 -6.55694343972218929295e-01  1.52031421334690719505e-02
// xi_: -1.51739359394178235929e-01 -5.19497254434936953094e-02 -9.87054402166866351465e-01
// body_rate_: -6.47995787753406271570e-01 -1.18143930590576040629e-01  1.05834096126307744545e-01

// fuck wrong
// attachpoint_vel is  1.10202110000000005896e-01 -5.13619009999999986960e-01 -6.80346299999999987174e-02
// robot_vel is  4.91868099999999974781e-02 -1.85771829999999998906e-01 -7.56362000000000006539e-02
// relative_vel_unit:  1.22030600000000016836e-01 -6.55694360000000031619e-01  1.52031400000000038730e-02
// xi_: -1.51739359394178235929e-01 -5.19497254434936953094e-02 -9.87054402166866351465e-01
// body_rate_: -6.47995803462865005429e-01 -1.18143926204684016068e-01  1.05834098310480489991e-01


}

