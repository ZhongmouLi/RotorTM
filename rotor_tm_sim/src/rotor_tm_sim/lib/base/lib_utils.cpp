#include "rotor_tm_sim/base/lib_utils.hpp"

namespace Utils 
{


    Eigen::Matrix3d TransVector3d2SkewSymMatrix(Eigen::Vector3d vector)
    {
        Eigen::Matrix3d m_skewsym;

        m_skewsym << 0, -vector(2), vector(1),
            vector(2), 0, -vector(0),
            -vector(1), vector(0), 0;

        return m_skewsym;
    };

}