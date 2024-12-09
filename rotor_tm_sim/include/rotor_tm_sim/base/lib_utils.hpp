#ifndef ROTORTM_UTILS_H
#define ROTORTM_UTILS_H

#include <Eigen/Dense>

namespace Utils 
{
    Eigen::Matrix3d TransVector3d2SkewSymMatrix(Eigen::Vector3d vector);
}


#endif