#include "odom_extrinsic_calibrate/OdomExtrinsicCalibrateError.h"
#include "odom_extrinsic_calibrate/local_parameterization_se2.hpp"

using namespace Sophus;
namespace odom_calib {
    bool OdomExtrinsicCalibrateError::Evaluate(double const *const *parameters,
                                       double *residuals,
                                       double **jacobians) const {
        return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, NULL);
    }

    bool OdomExtrinsicCalibrateError::EvaluateWithMinimalJacobians(double const *const *parameters,
                                                           double *residuals,
                                                           double **jacobians,
                                                           double **jacobiansMinimal) const {

        Eigen::Map<const Eigen::Vector2d> odom_paramter(parameters[0]);
        Eigen::Map<const SE2d> est_T_OC(parameters[1]);
        if (!jacobians) {
            return ceres::internal::VariadicEvaluate<
                    OdomExtrinsicCalibrateFunctor, double, 2, 4, 0, 0, 0,0,0,0,0,0>
            ::Call(*odomExtrinsicCalibrateFunctor_, parameters, residuals);
        }


        Eigen::Matrix<double,3,4,Eigen::RowMajor> sub_jacobian0;
        Eigen::Matrix<double,3,4,Eigen::RowMajor> sub_jacobian1;
        double* sub_jacobians[2] = {sub_jacobian0.data(), sub_jacobian1.data()};
        bool success =  ceres::internal::AutoDiff<OdomExtrinsicCalibrateFunctor, double,
                2, 4>::Differentiate(
                *odomExtrinsicCalibrateFunctor_,
                parameters,
                3,
                residuals,
                jacobians);

        if (success && jacobians != NULL) {
            if (jacobians[0] != NULL) {
                Eigen::Map<Eigen::Matrix<double, 3, 2, Eigen::RowMajor>> jacobian0(jacobians[0]);
                if (jacobiansMinimal != NULL && jacobiansMinimal[0] != NULL) {
                    Eigen::Map<Eigen::Matrix<double, 3, 2, Eigen::RowMajor>> mini_jacobian0(jacobiansMinimal[0]);
                    mini_jacobian0 = jacobian0;
                }
            }

            if (jacobians[1] != NULL) {
                if (jacobiansMinimal != NULL && jacobiansMinimal[1] != NULL) {
                    Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> jacobian1(jacobians[1]);
                    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> mini_jacobian1(jacobiansMinimal[1]);
                    test::LocalParameterizationSE2 localParameterizationSE2;
                    SE2d T_OC = est_T_OC;
                    Eigen::Matrix<double,4,3,Eigen::RowMajor> plusJacobian;
                    localParameterizationSE2.ComputeJacobian(T_OC.data(),plusJacobian.data());
                    mini_jacobian1 = jacobian1 * plusJacobian;
                }
            }
        }

        return true;
    }
}

