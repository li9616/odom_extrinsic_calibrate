#include <iostream>
#include <ceres/ceres.h>
#include <odom_extrinsic_calibrate/OdomSensorExtrinsicError.h>
#include <odom_extrinsic_calibrate/NumbDifferentiator.hpp>
#include <odom_extrinsic_calibrate/utils.h>
#include <odom_extrinsic_calibrate/odom_calibrate_solver.h>
#include <odom_extrinsic_calibrate/odom_integrator.h>
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

//    Sophus::SE2d est_T_C0C1 = T_OC.inverse() * T_O0O1 *T_OC;
//    Eigen::Vector3d error = (est_T_C0C1 * T_C0C1.inverse()).log();
//    std::cout<< "error: " << error.transpose() << std::endl;



    /**
     * Zero Test.
     */
    auto functor = std::make_shared<OdomSensorExtrinsicFunctor>(T_C0C1,T_O0O1);
    OdomSensorExtrinsicError odomSensorExtrinsicError(functor);

    Eigen::Vector3d residuals;

    double* parameters [1] = {T_OC.data()};
    odomSensorExtrinsicError.Evaluate(parameters, residuals.data(), NULL);
    std::cout << "residuals: " << residuals.transpose() << std::endl;

//
    /**
     *  Jacobian check
     *  Compare NumericDiffCostFunction  jacobian and one from NumbDifferentiator.
     */
    Eigen::Vector3d noised_parameter(0.21, 0.198, 0.2);
    Sophus::SE2d delta(noised_parameter[2], noised_parameter.head(2));
    Sophus::SE2d noised_T_OC = T_OC*delta;
    std::cout<<"noised_T_OC: "<<noised_T_OC.matrix() << std::endl;
    double* noised_parameters[1] = {noised_T_OC.data()};

    Eigen::Matrix<double,3,4,  Eigen::RowMajor> jacobian0;
    Eigen::Matrix<double,3,3,  Eigen::RowMajor> min_jacobian0;
    double* jacobians[1] = {jacobian0.data()};
    double* min_jacobians[1] = {min_jacobian0.data()};
    odomSensorExtrinsicError.EvaluateWithMinimalJacobians(noised_parameters,residuals.data(),
            jacobians, min_jacobians);
    NumbDifferentiator<OdomSensorExtrinsicError, 1> numbDifferentiator(&odomSensorExtrinsicError);

    Eigen::Matrix<double, 3,3, Eigen::RowMajor> num_min_jacobian0;
    numbDifferentiator.df_r_xi<3,4,3,Sophus::test::LocalParameterizationSE2>(noised_parameters,
            0, num_min_jacobian0.data());

    std::cout<< "num_min_jacobian0: \n" << num_min_jacobian0 << std::endl;
    std::cout<< "min_jacobian0: \n" << min_jacobian0 << std::endl;

//
    /**
     *  Opt.
     *
     */
    Sophus::SE2d init_T_OC = noised_T_OC;

    // Build the problem.
    ceres::Problem problem;

    Sophus::test::LocalParameterizationSE2* se2PoseLocalParameterization = new Sophus::test::LocalParameterizationSE2();

    // Specify local update rule for our parameter
    problem.AddParameterBlock(noised_T_OC.data(), 4, se2PoseLocalParameterization);

    // Create and add cost functions. Derivatives will be evaluated via
    // automatic differentiation
    auto functor1 = std::make_shared<OdomSensorExtrinsicFunctor>(T_C0C1,T_O0O1);
    ceres::CostFunction* cost_function1 =
            new OdomSensorExtrinsicError(functor1);
    problem.AddResidualBlock(cost_function1, NULL, noised_T_OC.data());

    // Set solver options (precision / method)
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = 40;
    options.minimizer_progress_to_stdout = true;

    // Solve
    ceres::Solver::Summary summary;
    Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << std::endl;

    std::cout<< "Noised parameter after Opt. \n " << noised_T_OC.matrix() << std::endl;
    std::cout<< "Noised parameter before Opt. \n " << init_T_OC.matrix() << std::endl;
    std::cout<< "True parameter. \n " << T_OC.matrix() << std::endl;
    return 0;
}