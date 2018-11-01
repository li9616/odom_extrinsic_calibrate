#ifndef SOPHUS_TEST_LOCAL_PARAMETERIZATION_SE2_HPP
#define SOPHUS_TEST_LOCAL_PARAMETERIZATION_SE2_HPP

#include <ceres/local_parameterization.h>
#include <sophus/se2.hpp>

namespace Sophus {
namespace test {

class LocalParameterizationSE2 : public ceres::LocalParameterization {
 public:
  virtual ~LocalParameterizationSE2() {}

  // SE3 plus operation for Ceres
  //
  //  T * exp(x)
  //
  virtual bool Plus(double const* T_raw, double const* delta_raw,
                    double* T_plus_delta_raw) const {
    Eigen::Map<SE2d const> const T(T_raw);
    Eigen::Map<Vector3d const> const delta(delta_raw);
    Eigen::Map<SE2d> T_plus_delta(T_plus_delta_raw);
    T_plus_delta = T * SE2d::exp(delta);
    return true;
  }

  // Jacobian of SE3 plus operation for Ceres
  //
  // Dx T * exp(x)  with  x=0
  //
  virtual bool ComputeJacobian(double const* T_raw,
                               double* jacobian_raw) const {
    Eigen::Map<SE2d const> T(T_raw);
    Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor>> jacobian(
        jacobian_raw);
    jacobian = T.Dx_this_mul_exp_x_at_0();
    return true;
  }

  virtual int GlobalSize() const { return SE2d::num_parameters; }

  virtual int LocalSize() const { return SE2d::DoF; }


};
}  // namespace test
}  // namespace Sophus

#endif
