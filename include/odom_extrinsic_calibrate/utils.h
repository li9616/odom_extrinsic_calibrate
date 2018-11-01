#ifndef _UTILS_H_
#define _UTILS_H_

#include "sophus/se2.hpp"

namespace utils {
    Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R);

    Sophus::SE2d poseVector2SE2d(const Eigen::Matrix<double,7,1>& poseVector);

}

#endif