#ifndef  _ODOMSENSOREXTRINSICERROR_H_
#define  _ODOMSENSOREXTRINSICERROR_H_


#include <sophus/se2.hpp>
#include <ceres/ceres.h>
#include <odom_extrinsic_calibrate/sophus_ceres_cast.h>

namespace  odom_calib {

    struct OdomSensorExtrinsicFunctor {
        OdomSensorExtrinsicFunctor(const Sophus::SE2d& T_C0C1,
                                 const Sophus::SE2d& T_O0O1):
                                 T_C0C1_(T_C0C1),T_O0O1_(T_O0O1){
        }

        template <class T>
        bool operator()(T const* const sT_0C, T* sResiduals) const {
            Eigen::Map<Sophus::SE2<T> const> const T_OC(sT_0C);
            Eigen::Map<Eigen::Matrix<T, 3, 1> > residuals(sResiduals);

            Sophus::SE2<T> est_T_C0C1 = T_OC.inverse() * T_O0O1_.cast<T>() *T_OC;
            residuals = (est_T_C0C1 * T_C0C1_.inverse().cast<T>()).log();
            return true;
        }

        Sophus::SE2d T_C0C1_;
        Sophus::SE2d T_O0O1_;
    };
    class OdomSensorExtrinsicError : public ceres::SizedCostFunction<3, 4> {
    public:
        OdomSensorExtrinsicError(std::shared_ptr<OdomSensorExtrinsicFunctor>
                odomSensorExtrinsicFunctor):
        odomSensorExtrinsicFunctor_(odomSensorExtrinsicFunctor) {
        }

        virtual ~OdomSensorExtrinsicError() {}

        virtual bool Evaluate(double const *const *parameters,
                              double *residuals,
                              double **jacobians) const;

        bool EvaluateWithMinimalJacobians(double const *const *parameters,
                                          double *residuals,
                                          double **jacobians,
                                          double **jacobiansMinimal) const;

    private:
        std::shared_ptr<OdomSensorExtrinsicFunctor> odomSensorExtrinsicFunctor_;
    };
}
#endif