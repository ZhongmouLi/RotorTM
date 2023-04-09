#ifndef ROS_QUADROTORDYNQMIC_SIMULATOR_H
#define ROS_QUADROTORDYNQMIC_SIMULATOR_H

#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <memory>
#include "rotor_tm_sim/lib_quadrotor_dynamic_simulator.hpp"
// #include "rotor_tm_msgs/RPMCommand.hpp"
// #include "rotor_tm_msgs/FMCommand.hpp"
#include <rotor_tm_msgs/FMCommand.h>

class ROSQuadrotorDynamicSimulator 
{
    private:
        // QuadrotorDynamicSimulator quadrotor_;
        std::unique_ptr<QuadrotorDynamicSimulator> quadrotor_;

        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;
        ros::Timer simulatorloop_time_;
        ros::Time last_request_, reference_request_now_;

        ros::Subscriber fm_command_sub;
        void fmCmdCallback(const rotor_tm_msgs::FMCommand &msg);

        ROSQuadrotorDynamicSimulator();
    public:
    // class methods and properties
        
        ROSQuadrotorDynamicSimulator(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private,const double &mass, const Eigen::Matrix3d &m_inertia, const double &step_size);
        ~ROSQuadrotorDynamicSimulator();
};


#endif