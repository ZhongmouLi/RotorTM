#ifndef CABLE_SIMULATOR_H
#define CABLE_SIMULATOR_H

#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <vector>


using namespace boost::numeric::odeint;


typedef Eigen::Matrix<double, 12, 1> object_state;


class Cable
{
    private:
        double cable_length_;
        
        Eigen::Vector3d tension_force_;

        Eigen::Vector3d unit_vector_;

        bool slack_status_; 
        

    public:

        void setTensionForce();
        void setUnitVect();
        void getTensionForce();
        void getUnitVect();
        void getSlackStatus();
        void goSlack();
        void goTaut();


};
#endif
