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


   // Template function definition (must be in header)
    template <typename Derived>
    std::string FortmatEigen4DBG(const Eigen::MatrixBase<Derived>& mat, const int &precision) {
        // Log the input matrix
        // std::cout << "Input matrix:\n" << mat << std::endl;
        
        std::ostringstream strs;
        strs << std::scientific<<std::setprecision(precision);
        
        strs << "[";
        for (int ii = 0; ii < mat.rows(); ++ii) {
            if (ii > 0) {
                strs << "; ";
            }
            for (int jj = 0; jj < mat.cols(); ++jj) {
                if (jj > 0) {
                    strs << ", ";
                }
                strs << mat(ii, jj);
            }
        }
        strs << "]";
        
        // Log the formatted string
        std::string formatted_str = strs.str();
        // std::cout << "Formatted string: " << formatted_str << std::endl;
        return formatted_str;
    };
}


#endif