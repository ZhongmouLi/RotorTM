#include "rotor_tm_sim/lib_ros_simulator.hpp"


ROSQuadrotorDynamicSimulator::ROSQuadrotorDynamicSimulator(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private,const double &mass, const Eigen::Matrix3d &m_inertia, const double &step_size): nh_(nh), nh_private_(nh_private), quadrotor_(std::make_unique<QuadrotorDynamicSimulator>(mass, m_inertia, step_size)){
  std::cout<<"initilisation"<<std::endl;

  fm_command_sub =
      nh_.subscribe("reference/fm_cmd", 1, &ROSQuadrotorDynamicSimulator::fmCmdCallback, this, ros::TransportHints().tcpNoDelay());
}


ROSQuadrotorDynamicSimulator::~ROSQuadrotorDynamicSimulator() {
  // Destructor
}

void ROSQuadrotorDynamicSimulator::fmCmdCallback(const rotor_tm_msgs::FMCommand &msg)
{

}
