#ifndef  _ODOMEXTRINSICCALIBRATEERROR_H_
#define  _ODOMEXTRINSICCALIBRATEERROR_H_
#include "sophus/se2.hpp"
#include <ceres/ceres.h>

using namespace Sophus;
namespace  odom_calib {
    struct OdomExtrinsicCalibrateFunctor {
        OdomExtrinsicCalibrateFunctor(const Sophus::SE2d& T_C0C1,
                                      const Eigen::Vector2d tick0,
                                      const Eigen::Vector2d tick1):
                T_C0C1_(T_C0C1), ticks0_(tick0), ticks1_(tick1){
        }
        template <class T>
        bool operator()( T const* const s_odom_parameter, T const* const sT_0C, T* sResiduals) const {
            Eigen::Map<Eigen::Matrix<T,2,1> const> const odom_parameter(s_odom_parameter);
            Eigen::Map<Sophus::SE2<T> const> const T_OC(sT_0C);
            Eigen::Map<Eigen::Matrix<T, 3, 1> > residuals(sResiduals);

            Eigen::Matrix<T,3,1> pose_odom;
            Eigen::Matrix<T,2,1> ticks = ticks1_.cast<T>() - ticks0_.cast<T>();

            T delta_s_l = ticks[0] * odom_parameter[0] * T(rad_per_tick_);
            T delta_s_r = ticks[1] * odom_parameter[1] * T(rad_per_tick_);
            T delta_s = (delta_s_l + delta_s_r) * T(0.5);
            T delta_theta = (delta_s_r - delta_s_l) / T(baseline_);
            T cos_half_delta_theta = cos(T(0.5) * delta_theta);
            T sin_half_delta_theta = sin(T(0.5) * delta_theta);
            pose_odom = Eigen::Matrix<T, 3, 1>(delta_s * cos_half_delta_theta,
                                                    delta_s * sin_half_delta_theta,
                                                    delta_theta);
            Eigen::Matrix<T,3,1>  res_pose = pose_odom;
            if (res_pose[2] > M_PI) {
                res_pose[2] = res_pose[2] - T(2 * M_PI);
            } else if (res_pose[2] < -M_PI) {
                res_pose[2] = res_pose[2] + T(2 * M_PI);
            }

            Sophus::SE2<T>  T_O0O1(res_pose[2], res_pose.template head<2>());
            Sophus::SE2<T> est_T_C0C1 = T_OC.inverse() * T_O0O1 *T_OC;
            residuals = (est_T_C0C1 * T_C0C1_.inverse().cast<T>()).log();
            return true;
        }

        Sophus::SE2d T_C0C1_;
        Eigen::Vector2d ticks0_;
        Eigen::Vector2d ticks1_;
        const double baseline_ = 0.4456;
        const double rad_per_tick_ = 2.0 * M_PI / 16384;
    };
    class OdomExtrinsicCalibrateError : public ceres::SizedCostFunction<3, 2, 4> {
    public:
        explicit OdomExtrinsicCalibrateError(const SE2d& T_WC0,
                                     const SE2d& T_WC1,
                                     const Eigen::Vector2d tick0,
                                     const Eigen::Vector2d tick1)
                 {
            SE2d T_WC0_inv = T_WC0.inverse();
            meas_T_C0C1_ = T_WC0_inv * T_WC1;
         odomExtrinsicCalibrateFunctor_ =
                 std::make_shared<OdomExtrinsicCalibrateFunctor>(meas_T_C0C1_, tick0, tick1);
        }

        virtual ~OdomExtrinsicCalibrateError() {}

        virtual bool Evaluate(double const *const *parameters,
                              double *residuals,
                              double **jacobians) const;

        bool EvaluateWithMinimalJacobians(double const *const *parameters,
                                          double *residuals,
                                          double **jacobians,
                                          double **jacobiansMinimal) const;

    private:

        SE2d meas_T_C0C1_;
        std::shared_ptr<OdomExtrinsicCalibrateFunctor> odomExtrinsicCalibrateFunctor_;
    };
}
#endif