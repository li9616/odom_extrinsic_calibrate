#include <iostream>
#include <ceres/ceres.h>
#include <odom_extrinsic_calibrate/OdomCalibrateError.h>
#include <odom_extrinsic_calibrate/NumbDifferentiator.hpp>
#include <odom_extrinsic_calibrate/utils.h>
#include <odom_extrinsic_calibrate/odom_calibrate_solver.h>
#include <odom_extrinsic_calibrate/odom_integrator.h>
using namespace   odom_calib;

int main() {
    std::cout<< "test odom calibrate error ... " << std::endl;

    /**
     *  Simulate Data
     */

    Eigen::Vector2d delta_ticks(3000, 2000);
    Eigen::Vector2d odom_parameter(0.1,0.1);
    OdomIntergrator temp;
    Eigen::Vector3d pose0(0.2,0.3, 0.03);
    Eigen::Vector3d pose1 = temp.computeOdom(delta_ticks,
            odom_parameter, pose0);
    Sophus::SE2d  T_WO0 = vector2SE2(pose0);
    Sophus::SE2d  T_WO1 = vector2SE2(pose1);

    std::cout<< "pose0:         " << pose0.transpose() << std::endl;
    std::cout<< "pose1:         " << pose1.transpose() << std::endl;
    std::cout<< "pose1 - pose0: " << (pose1 - pose0).transpose() << std::endl;

    std::cout<< "sopuhs pose0:         " << SE22vector(T_WO0).transpose() << std::endl;
    std::cout<< "sopuhs pose1:         " << SE22vector(T_WO1).transpose() << std::endl;
    std::cout<< "sopuhs pose1 - pose0: " << SE22vector(T_WO0.inverse()*T_WO1).transpose() << std::endl;

    Sophus::SE2d T_OC;
    std::cout<< T_OC.matrix() << std::endl;

    Sophus::SE2d T_WC0 = T_WO0 * T_OC;
    Sophus::SE2d T_WC1 = T_WO1 * T_OC;


    /**
     * Zero Test.
     */
    OdomCalibrateError2 odomCalibrateError2(T_WC0,T_WC1,Eigen::Vector2d::Zero(), delta_ticks, T_OC);

    Eigen::Vector3d residuals;
    double* parameters [1] = {odom_parameter.data()};
    odomCalibrateError2.Evaluate(parameters, residuals.data(), NULL);
    std::cout << "residuals: " << residuals.transpose() << std::endl;


    /**
     *  Jacobian check
     *  Compare NumericDiffCostFunction  jacobian and one from NumbDifferentiator.
     */
    Eigen::Vector2d noised_parameter(0.21, 0.198);
    double* noised_parameters[1] = {noised_parameter.data()};

    Eigen::Matrix<double,3,2,  Eigen::RowMajor> jacobian0;
    double* jacobians[1] = {jacobian0.data()};
    odomCalibrateError2.Evaluate(noised_parameters,residuals.data(), jacobians);
    NumbDifferentiator<OdomCalibrateError2, 1> numbDifferentiator(&odomCalibrateError2);

    Eigen::Matrix<double, 3,2, Eigen::RowMajor> num_jacobian0;
    numbDifferentiator.df_r_xi<3,2>(noised_parameters, 0, num_jacobian0.data());

    std::cout<< "num_jacobian0: \n" << num_jacobian0 << std::endl;
    std::cout<< "jacobian0: \n" << jacobian0 << std::endl;


    /**
     *  Opt.
     *
     */
    // Build the problem.
    ceres::Problem problem;

    // Specify local update rule for our parameter
    problem.AddParameterBlock(noised_parameter.data(), 2);

    // Create and add cost functions. Derivatives will be evaluated via
    // automatic differentiation
    ceres::CostFunction* cost_function1 =
                    new OdomCalibrateError2(T_WC0,T_WC1,Eigen::Vector2d::Zero(), delta_ticks, T_OC);
    problem.AddResidualBlock(cost_function1, NULL, noised_parameter.data());

    // Set solver options (precision / method)
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = 10;
    options.minimizer_progress_to_stdout = true;

    // Solve
    ceres::Solver::Summary summary;
    Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << std::endl;

    std::cout<< "Noised parameter after Opt. " << noised_parameter.transpose() << std::endl;
    return 0;
}