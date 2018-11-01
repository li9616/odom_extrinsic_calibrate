#ifndef  _SOPHUS_CERES_CAST_H_
#define  _SOPHUS_CERES_CAST_H_

#include <sophus/se2.hpp>
#include <ceres/ceres.h>
// Eigen's ostream operator is not compatible with ceres::Jet types.
// In particular, Eigen assumes that the scalar type (here Jet<T,N>) can be
// casted to an arithmetic type, which is not true for ceres::Jet.
// Unfortunatly, the ceres::Jet class does not define a conversion
// operator (http://en.cppreference.com/w/cpp/language/cast_operator).
//
// This workaround creates a template specilization for Eigen's cast_impl,
// when casting from a ceres::Jet type. It relies on Eigen's internal API and
// might break with future versions of Eigen.
namespace Eigen {
    namespace internal {

        template <class T, int N, typename NewType>
        struct cast_impl<ceres::Jet<T, N>, NewType> {
        EIGEN_DEVICE_FUNC
        static inline NewType run(ceres::Jet<T, N> const& x) {
        return static_cast<NewType>(x.a);
    }
};

}  // namespace internal
}  // namespace Eigen
#endif