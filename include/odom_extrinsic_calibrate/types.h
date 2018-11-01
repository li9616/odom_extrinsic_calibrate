#ifndef  _TYPES_H_
#define  _TYPES_H_
#include <odom_extrinsic_calibrate/Time.hpp>
#include <Eigen/Core>

namespace odom_calib {
    template <typename T>
    struct Measurement {
        Time ts;
        T meas;
    };
    typedef  Eigen::Matrix<double,3,1> Pose;
    typedef  Measurement<Eigen::Vector2d>  EncoderMeas;
    typedef  Measurement<Pose>  PoseMeas;
    typedef  PoseMeas VioPoseMeas;
}
#endif