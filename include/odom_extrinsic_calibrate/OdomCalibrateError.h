#ifndef  _ODOMCALIBRATEERROR_H_
#define  _ODOMCALIBRATEERROR_H_


#include "sophus/se2.hpp"
#include <ceres/ceres.h>

namespace  odom_calib {
    Sophus::SE2d vector2SE2(const Eigen::Vector3d &vec);
    Eigen::Vector3d SE22vector(const Sophus::SE2d &se2);

    class OdomCalibrateError2 : public ceres::SizedCostFunction<3, 2> {
    public:
        explicit OdomCalibrateError2(const Sophus::SE2d T_WC0, const Sophus::SE2d T_WC1,
                                     const Eigen::Vector2d tick0,
                                     const Eigen::Vector2d tick1,
                                     const Sophus::SE2d T_OC)
                : T_OC_(T_OC),
                  ticks0_(tick0),
                  ticks1_(tick1){
            Sophus::SE2d T_WC0_inv = T_WC0.inverse();
            T_C0C1_ = T_WC0_inv * T_WC1;
            meas_T_O0O1_ = T_OC_ * T_C0C1_ * T_OC_.inverse();
        }

        virtual ~OdomCalibrateError2() {}

        virtual bool Evaluate(double const *const *parameters,
                              double *residuals,
                              double **jacobians) const;

        bool EvaluateWithMinimalJacobians(double const *const *parameters,
                                          double *residuals,
                                          double **jacobians,
                                          double **jacobiansMinimal) const;

    private:

        Sophus::SE2d T_OC_;
        Sophus::SE2d T_C0C1_;
        Eigen::Vector2d ticks0_;
        Eigen::Vector2d ticks1_;
        Sophus::SE2d meas_T_O0O1_;
        const double baseline_ = 0.4456;
        const double rad_per_tick_ = 2.0 * M_PI / 16384;
    };

}

#endif