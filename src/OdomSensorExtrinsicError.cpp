#include "odom_extrinsic_calibrate/OdomSensorExtrinsicError.h"
#include "odom_extrinsic_calibrate/local_parameterization_se2.hpp"

namespace  odom_calib {

    bool OdomSensorExtrinsicError::Evaluate(double const *const *parameters,
                                            double *residuals,
                                            double **jacobians) const {
        return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, NULL);
    }

    bool OdomSensorExtrinsicError::EvaluateWithMinimalJacobians(double const *const *parameters,
                                                                double *residuals,
                                                                double **jacobians,
                                                                double **jacobiansMinimal) const {

        Eigen::Map<const Sophus::SE2d> est_T_OC(parameters[0]);



        if (!jacobians) {
            return ceres::internal::VariadicEvaluate<
                    OdomSensorExtrinsicFunctor, double, 4, 0, 0, 0, 0,0,0,0,0,0>
            ::Call(*odomSensorExtrinsicFunctor_, parameters, residuals);
        }
        bool success =  ceres::internal::AutoDiff<OdomSensorExtrinsicFunctor, double,
                4>::Differentiate(
                *odomSensorExtrinsicFunctor_,
                parameters,
                SizedCostFunction<3,
                        4>::num_residuals(),
                residuals,
                jacobians);

        if (success && jacobians != NULL) {
            if (jacobians[0] != NULL) {
                if (jacobiansMinimal != NULL && jacobiansMinimal[0] != NULL) {
                    Eigen::Map <Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> mini_jacobian0(jacobiansMinimal[0]);
                    Eigen::Map <Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> jacobian0(jacobians[0]);
                    Eigen::Matrix<double,4,3,Eigen::RowMajor> plusJacobian;
                    Sophus::test::LocalParameterizationSE2 localParameterizationSE2;
                    localParameterizationSE2.ComputeJacobian(parameters[0],plusJacobian.data());
                    mini_jacobian0 = jacobian0 * plusJacobian;
                }
            }

        }
        return success;
    }
}


