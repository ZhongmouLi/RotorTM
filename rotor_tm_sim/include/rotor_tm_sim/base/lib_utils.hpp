#ifndef ROTORTM_UTILS_H
#define ROTORTM_UTILS_H

#include <Eigen/Dense>
#include <iostream>
#include <iomanip>
#include <sstream>

namespace Utils 
{

    const double gravity = 9.81;
    
    Eigen::Matrix3d TransVector3d2SkewSymMatrix(const Eigen::Vector3d &vector);

    std::string FormatDouble4DBG(const double &value, const int &precision);


    double CompensatedDot(const Eigen::Vector3d& a, const Eigen::Vector3d& b);

    Eigen::Vector3d CompensatedMatVecMul(const Eigen::Matrix3d& A, const Eigen::Vector3d& b);


    template<typename Derived>
    std::string EigenVectorToString(const Eigen::MatrixBase<Derived>& v, int precision = 10) {
        std::ostringstream oss;
        oss << std::setprecision(precision);
        
        for (int i = 0; i < v.size(); ++i) {
            if (i > 0) oss << " ";
            if (v.cols() == 1) {
                oss << v(i, 0);
            } else {
                oss << v(0, i);
            }
        }
        
        return oss.str();
    }

    // Helper function to convert Eigen matrix to a string on a single line
    template<typename Derived>
    std::string EigenMatrixToString(const Eigen::MatrixBase<Derived>& m, int precision = 10) {
        std::ostringstream oss;
        oss << std::setprecision(precision);
        
        for (int i = 0; i < m.rows(); ++i) {
            for (int j = 0; j < m.cols(); ++j) {
                if (i > 0 || j > 0) oss << " ";
                oss << m(i, j);
            }
        }
        
        return oss.str();
    }

    template<typename T, size_t N>
    std::string arrayToString(const std::array<T, N>& arr, int precision = 6) {
        std::ostringstream oss;
        oss << std::setprecision(precision);
        
        for (size_t i = 0; i < N; ++i) {
            if (i > 0) oss << " ";
            oss << arr[i];
        }
        
        return oss.str();
    }
}


#endif