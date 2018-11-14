#include <iostream>
#include <ceres/ceres.h>
#include <odom_extrinsic_calibrate/NumbDifferentiator.hpp>
#include <odom_extrinsic_calibrate/utils.h>
#include <odom_extrinsic_calibrate/odom_calibrate_solver.h>
#include <odom_extrinsic_calibrate/odom_integrator.h>
#include <odom_extrinsic_calibrate/OdomExtrinsicCalibrateError.h>
#include <odom_extrinsic_calibrate/local_parameterization_se2.hpp>

using namespace   odom_calib;
int main() {
    std::cout<< "test odom calibrate error ... " << std::endl;

    /**
     *  Simulate Data
     */

    Eigen::Vector2d delta_ticks(3000, -2000);
    Eigen::Vector2d odom_parameter(0.1,0.1);
    OdomIntergrator temp;
    Eigen::Vector3d pose0(0.2,0.3, 0.00);
    Eigen::Vector3d pose1 = temp.computeOdom(delta_ticks,
                                             odom_parameter, pose0);

    Sophus::SE2d  T_WO0(pose0[2], pose0.head(2));
    Sophus::SE2d  T_WO1(pose1[2], pose1.head(2));
    Sophus::SE2d  T_O0O1 = T_WO0.inverse() * T_WO1;

    std::cout<< "T_WO0: \n" << T_WO0.matrix() << std::endl;
    std::cout<< "T_WO1: \n" << T_WO1.matrix() << std::endl;
    std::cout<< "T_O0O1: \n" << T_O0O1.matrix() << std::endl;

    Sophus::SE2d T_OC( 0.001, Eigen::Vector2d(0.13, 0.13));
    std::cout<< "T_OC:  \n" << T_OC.matrix() << std::endl;
    Sophus::SE2d T_WC0 = T_WO0 * T_OC;
    Sophus::SE2d T_WC1 = T_WO1 * T_OC;
    Sophus::SE2d T_C0C1 = T_WC0.inverse()*T_WC1;

    std::cout<< "T_WC0: " << T_WC0.matrix() << std::endl;
    std::cout<< "T_WC1: " << T_WC1.matrix() << std::endl;
    std::cout<< "T_C0C1: " << T_C0C1.matrix() << std::endl;

    /**
     * Zero Test.
     */
    OdomExtrinsicCalibrateError odomExtrinsicCalibrateError(T_WC0,T_WC1,
            Eigen::Vector2d::Zero(), delta_ticks);

    Eigen::Vector3d residuals;
    double* parameters [2] = {odom_parameter.data(), T_OC.data()};
    odomExtrinsicCalibrateError.Evaluate(parameters, residuals.data(), NULL);
    std::cout << "residuals: " << residuals.transpose() << std::endl;

//
    /**
     *  Jacobian check
     *  Compare NumericDiffCostFunction jacobian and one from NumbDifferentiator.
     */
    Eigen::Vector2d noised_parameter(0.21, 0.198);
    SE2d delta(0.001, Eigen::Vector2d(0.01, 0.00));
    SE2d noised_T_OC = T_OC*delta;

    double* noised_parameters[2] = {noised_parameter.data(), noised_T_OC.data()};

    Eigen::Matrix<double,3,2,  Eigen::RowMajor> jacobian0;
    Eigen::Matrix<double,3,4,  Eigen::RowMajor> jacobian1;

    Eigen::Matrix<double,3,2,  Eigen::RowMajor> min_jacobian0;
    Eigen::Matrix<double,3,3,  Eigen::RowMajor> min_jacobian1;
    double* jacobians[2] = {jacobian0.data(), jacobian1.data()};
    double* min_jacobians[2] = {min_jacobian0.data(), min_jacobian1.data()};
    odomExtrinsicCalibrateError.EvaluateWithMinimalJacobians(noised_parameters,residuals.data(),
            jacobians, min_jacobians);
    NumbDifferentiator<OdomExtrinsicCalibrateError, 2> numbDifferentiator(&odomExtrinsicCalibrateError);

    Eigen::Matrix<double, 3,2, Eigen::RowMajor> num_jacobian0;
    numbDifferentiator.df_r_xi<3,2>(noised_parameters, 0, num_jacobian0.data());

    std::cout<< "num_jacobian0: \n" << num_jacobian0 << std::endl;
    std::cout<< "jacobian0: \n" << jacobian0 << std::endl;


    Eigen::Matrix<double, 3,3, Eigen::RowMajor> num_min_jacobian1;
    numbDifferentiator.df_r_xi<3,4,3,test::LocalParameterizationSE2>(noised_parameters, 1, num_min_jacobian1.data());

    std::cout<< "num_min_jacobian1: \n" << num_min_jacobian1 << std::endl;
    std::cout<< "min_jacobian1: \n" << min_jacobian1 << std::endl;
//

    return 0;
}