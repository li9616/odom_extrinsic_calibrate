#ifndef  _CALIBRATE_PROCESS_H_
#define  _CALIBRATE_PROCESS_H_

#include <thread>
#include <queue>

#include <odom_extrinsic_calibrate/buffer.h>
#include <odom_extrinsic_calibrate/types.h>


namespace odom_calib{
    class Calibrator;
    class OdomCalibrateSolver;
    struct CalibrateResult;

    template <typename T>
    T interpolateMeasurement(const Time& t, T& before, T& after) {
        T interpolated;
        interpolated.ts = t;
        double  r = (t - before.ts).toSec() / (after.ts - before.ts).toSec();
        interpolated.meas = (1 - r)* before.meas  + r * after.meas;
        return interpolated;
    }

    class CalibrateProcess {
    public:
        CalibrateProcess(const std::string calib_file_folder);
        ~CalibrateProcess();

        void addEncoderMeas(double ts, double left, double right);
        void addVioPoseMeas(double ts, Pose vioPose);
              
        bool calibrateOnFullPath(Eigen::Vector2d& parameter, Eigen::Matrix2d& cov);
        
        // tools
        std::vector<Eigen::Vector3d>
        intergateFullPath(const Eigen::Vector2d& odom_paramter);
    private:
          
        std::shared_ptr<common::Buffer<double,2>> encoderBuffer_;
        std::shared_ptr<std::queue<VioPoseMeas>> vioPoseBuffer_;
        std::shared_ptr<OdomCalibrateSolver> odomCalibrateSolver_;

        std::shared_ptr<CalibrateResult> priorCalibrateResult_;
        std::shared_ptr<CalibrateResult> newCalibrateResult_;

        std::vector<Eigen::Vector2d> fullEncoderMeas_;
        const int64_t odom_buffer_s_ = 3;
        std::string calib_file_folder_;
        double init_uncertainty_ = 1000;

        const double kRadiusToRot_ = 2*M_PI / 16384 / 0.4456;
        const double kTickToRotDefault_ = 0.1 * kRadiusToRot_;

    };
}

#endif
