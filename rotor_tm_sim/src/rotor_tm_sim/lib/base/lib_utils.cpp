#include "rotor_tm_sim/base/lib_utils.hpp"

namespace Utils 
{


    Eigen::Matrix3d TransVector3d2SkewSymMatrix(const Eigen::Vector3d &vector)
    {
        Eigen::Matrix3d m_skewsym;

        m_skewsym << 0, -vector(2), vector(1),
            vector(2), 0, -vector(0),
            -vector(1), vector(0), 0;

        return m_skewsym;
    };

    std::string FormatDouble4DBG(const double &value, const int &precision)
    {
        std::ostringstream strs;
        strs << std::scientific<<std::setprecision(precision) << value;
        std::string str = strs.str();
        return str;
    };


    double CompensatedDot(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
        double sum = 0.0;
        double c = 0.0;  // compensation term
        for(int i = 0; i < 3; ++i) {
            double prod = a(i) * b(i);
            double y = prod - c;
            double t = sum + y;
            c = (t - sum) - y;  // update compensation
            sum = t;
        }
        return sum;
    }

    // Compensated matrix-vector multiplication
    Eigen::Vector3d CompensatedMatVecMul(const Eigen::Matrix3d& A, const Eigen::Vector3d& b) {
        Eigen::Vector3d result;
        for(int i = 0; i < 3; ++i) {
            result(i) = CompensatedDot(A.row(i), b);
        }
        return result;
}


}