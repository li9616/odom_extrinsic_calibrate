#include "odom_extrinsic_calibrate/OdomCalibrateError.h"

namespace  odom_calib {

    Sophus::SE2d vector2SE2(const Eigen::Vector3d &vec) {
        Sophus::SO2d so2d(vec[2]);
        Sophus::SE2d se2d(so2d, vec.head(2));
        return se2d;
    }

    Eigen::Vector3d SE22vector(const Sophus::SE2d &se2) {
        double angle = atan2(se2.rotationMatrix()(1,0),se2.rotationMatrix()(0,0) );
        Eigen::Vector3d vec;
        vec <<  se2.translation(), angle ;
        return vec;
    }

    bool OdomCalibrateError2::Evaluate(double const *const *parameters,
                                      double *residuals,
                                      double **jacobians) const {
        return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, NULL);
    }

    bool OdomCalibrateError2::EvaluateWithMinimalJacobians(double const *const *parameters,
                                                          double *residuals,
                                                          double **jacobians,
                                                          double **jacobiansMinimal) const {

        Eigen::Map<const Eigen::Vector2d>  odom_paramter(parameters[0]);

        Eigen::Vector3d pose_odom;
        Eigen::Vector2d ticks = ticks1_ - ticks0_;

        double delta_s_l = ticks[0] * odom_paramter[0] * rad_per_tick_;
        double delta_s_r = ticks[1] * odom_paramter[1] * rad_per_tick_;
        double delta_s = (delta_s_l + delta_s_r) * 0.5;
        double delta_theta = (delta_s_r - delta_s_l) / baseline_;
        double cos_half_delta_theta = cos(0.5*delta_theta);
        double sin_half_delta_theta = sin(0.5*delta_theta);
        pose_odom = Eigen::Matrix<double, 3, 1>(delta_s * cos_half_delta_theta,
                                                delta_s * sin_half_delta_theta,
                                                delta_theta);

        Eigen::Vector3d res_pose = pose_odom;
        if (res_pose[2] > M_PI) {
            res_pose[2] = res_pose[2] - 2* M_PI;
        }else if (res_pose[2] < -M_PI) {
            res_pose[2] = res_pose[2] + 2* M_PI;
        }


//        std::cout<< "res_pose: " << res_pose.transpose() << std::endl;
        Eigen::Vector3d  meas_pose;
        double angle = atan2(meas_T_O0O1_.rotationMatrix()(1,0),meas_T_O0O1_.rotationMatrix()(0,0));
        meas_pose << meas_T_O0O1_.translation(), angle;


        Eigen::Map<Eigen::Vector3d> error(residuals);
        error = res_pose - meas_pose;

        double left_rad = ticks[0]*rad_per_tick_;
        double right_rad = ticks[1]*rad_per_tick_;

        if (jacobians != NULL ) {
            if (jacobians[0] != NULL) {

                Eigen::Matrix<double,3,2, Eigen::RowMajor> J_delta_xytheta;
                // x
                J_delta_xytheta << left_rad*0.5 * cos_half_delta_theta + delta_s*sin_half_delta_theta*left_rad/(2*baseline_),
                        right_rad*0.5*cos_half_delta_theta - delta_s*sin_half_delta_theta*right_rad/(2*baseline_),
                        // y
                        left_rad*0.5 * sin_half_delta_theta - delta_s*cos_half_delta_theta*left_rad/(2*baseline_),
                        right_rad*0.5 * sin_half_delta_theta + delta_s*cos_half_delta_theta*right_rad/(2*baseline_),
                        // theta
                        -left_rad/baseline_, right_rad/baseline_;

                Eigen::Map<Eigen::Matrix<double,3,2, Eigen::RowMajor>>  jacobian0(jacobians[0]);
                jacobian0 = J_delta_xytheta;


                if (jacobiansMinimal != NULL && jacobiansMinimal[0] != NULL) {
                    Eigen::Map<Eigen::Matrix<double,3,2, Eigen::RowMajor>>  mini_jacobian0(jacobiansMinimal[0]);
                    mini_jacobian0 = jacobian0;
                }
            }
        }

        return true;
    }
}