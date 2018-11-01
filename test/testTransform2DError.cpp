#include <iostream>

#include <odom_extrinsic_calibrate/odom_calibrate_solver.h>
#include <odom_extrinsic_calibrate/NumbDifferentiator.hpp>

int main() {

    // Simulate data
    double theta = 0.3;
    Eigen::Matrix2d R;
    R << cos(theta), -sin(theta), sin(theta), cos(theta);

    Eigen::Vector2d trans(0,1.2);
    Eigen::Vector2d x(0, 0.3);
    Eigen::Vector2d y = R*x + trans;

    /*
     * Zero
     */

    double* parameters[2] = {&theta, trans.data()};
    Eigen::Vector2d residuals;
    odom_calib::Transform2DError transform2DError(x,y);
    transform2DError.Evaluate(parameters,residuals.data(),NULL);
    std::cout<< "residuals: " << residuals.transpose() << std::endl;

    /**
     * Jacobians
     */

    NumbDifferentiator<odom_calib::Transform2DError,2> numbDifferentiator(&transform2DError);

    Eigen::Matrix<double,2,1> jacobian0;
    Eigen::Matrix<double,2,1> num_jacobian0;
    Eigen::Matrix<double,2,2,Eigen::RowMajor> jacobian1;
    Eigen::Matrix<double,2,2,Eigen::RowMajor> num_jacobian1;

    double* jacobians[2] = {jacobian0.data(), jacobian1.data()};
    transform2DError.Evaluate(parameters,residuals.data(), jacobians);

    numbDifferentiator.df_r_1D_x<2>(parameters,0,num_jacobian0.data());
    numbDifferentiator.df_r_xi<2,2>(parameters,1,num_jacobian1.data());
    std::cout<< "jacobian0: " << jacobian0<< std::endl;
    std::cout<< "num_jacobian0: " << num_jacobian0<< std::endl;

    std::cout<< "jacobian1: " << jacobian1<< std::endl;
    std::cout<< "num_jacobian1: " << num_jacobian1<< std::endl;

    return 1;
}