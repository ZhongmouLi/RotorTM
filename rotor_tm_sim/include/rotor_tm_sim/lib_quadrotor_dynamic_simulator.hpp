#include <iostream>
#include <boost/numeric/odeint.hpp>
#include <Eigen/Dense>
// #include <boost/numeric/odeint/external/eigen/eigen_algebra.hpp>
#include <boost/numeric/odeint/external/eigen/eigen.hpp>
#include <cmath>
#include <vector>

// #include <functional>
// namespace pl = std::placeholders;

using namespace boost::numeric::odeint;


typedef Eigen::Matrix<double, 12, 1> quadrotor_state;


class QuadrotorDynamicSimulator
{
    private:

        double mass_;
        double step_size_;
        Eigen::Matrix3d m_inertia_;
        const double gravity = 9.8;

        double current_step_ = 0;

        Eigen::Vector3d post_;
        Eigen::Vector3d vel_;
        Eigen::Vector3d bodyrate_;
        Eigen::Quaterniond attitude_;

        Eigen::Vector3d thrust_;
        Eigen::Vector3d torque_;

        // rotational dynamic
        // compute d_bodyrate
        Eigen::Vector3d quadRotDynac(const Eigen::Vector3d &torque, const Eigen::Matrix3d &Inertia, const Eigen::Vector3d &bodyrate);

        // translation dyanmic
        Eigen::Vector3d quadTransDynac(const Eigen::Vector3d &Thurst, const double &mass, const double &gravity);

        // bodyrate to d_Euler
        Eigen::Vector3d quadBodyrate2Eulerrate(const Eigen::Vector3d &bodyrate, const Eigen::Matrix3d &trans_matrix);

        // matrix transforming bodyrate to d_Euler
        Eigen::Matrix3d quadTransMatrix(const double &phi, const double &theta, const double &psi);

        // dynamic model of quadrotor
        void rhs(const quadrotor_state &x , quadrotor_state &dxdt, const double time);

        quadrotor_state done_state_;

        runge_kutta4<quadrotor_state> stepper_;

        inline double deg2rad(double deg) {return deg * M_PI / 180.0;};

        QuadrotorDynamicSimulator();

        void assignDroneState(const quadrotor_state &done_state);

    public:

        // constructor
        QuadrotorDynamicSimulator(const double &mass, const Eigen::Matrix3d &m_inertia, const double &step_size);

        // do one step integration
        void doOneStepInt();
        void operator()(const quadrotor_state &x , quadrotor_state &dxdt, const double time); // Declare the function call operator

        // get
        void getPosition(Eigen::Vector3d &mav_position);

        void getVel(Eigen::Vector3d &mav_vel);

        void getBodyrate(Eigen::Vector3d &mav_bodyrate);

        void getAttitude(Eigen::Quaterniond &mav_attitude);

        void inputThurstForce(const Eigen::Vector3d &mav_thrust);

        void inputTorque(const Eigen::Vector3d &mav_torque);

        void getCurrentTimeStep(double &current_time);
};

