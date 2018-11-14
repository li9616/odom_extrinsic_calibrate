#include <unistd.h>
#include "odom_extrinsic_calibrate/calibrate_process.h"
#include <odom_extrinsic_calibrate/file-system-tools.h>
#include <odom_extrinsic_calibrate/odom_calibrate_solver.h>
#include <odom_extrinsic_calibrate/odom_integrator.h>
#include <boost/lexical_cast.hpp>
#include <yaml-cpp/yaml.h>
#include <iomanip>
namespace odom_calib {
    std::string timeOfDay() {
        std::time_t rawtime;
        std::tm* timeinfo;
        char buffer [80];
        std::time(&rawtime);
        timeinfo = std::localtime(&rawtime);
        std::strftime(buffer,80,"%Y-%m-%d-%H-%M-%S",timeinfo);
        std::stringstream ss;
        ss << buffer;
        std::string str;
        ss >> str;
        return str;
    }

    CalibrateProcess::CalibrateProcess(const std::string calib_file_folder):
            calib_file_folder_(calib_file_folder) {
        encoderBuffer_ = std::make_shared<common::Buffer<double,2>>(odom_buffer_s_);
        vioPoseBuffer_ = std::make_shared<std::queue<VioPoseMeas>>();

        ConfigParams params;
        // CAD: http://10.10.80.30:8090/pages/viewpage.action?pageId=19005598
        Eigen::Isometry3d  T_OC = Eigen::Isometry3d::Identity();
        T_OC.translation() = Eigen::Vector3d(0.28878, 0.04625, 0.6425);
        odomCalibrateSolver_ = std::make_shared<OdomCalibrateSolver>(params, T_OC);
        priorCalibrateResult_ = std::make_shared<CalibrateResult>();
        
        /// 8.60627e-05, 8.60627e-05
        priorCalibrateResult_->estimate_parameter[0]  = 0.1 ;
        priorCalibrateResult_->estimate_parameter[1]  = 0.1 ;
        priorCalibrateResult_->covariance
            << 0.01, 0, 0, 0.01;
        priorCalibrateResult_->average_final_cost = 1;


        odomCalibrateSolver_->setPriorEstimate(*priorCalibrateResult_);

        //std::string calib_log_file = calib_file_folder_ + "/calib_log.txt";
        //calib_log_ofs_.open(calib_log_file, std::ios::out | std::ios::app);

        calib_log_yaml_ = calib_file_folder_ + "/calib_log_yaml.yaml";
        calib_log_yaml_ofs_.open(calib_log_yaml_,
                std::fstream::in | std::fstream::out | std::fstream::app);
        calib_log_yaml_ofs_.close();
    }

    CalibrateProcess::~CalibrateProcess() {
        //calib_log_ofs_.close();
    }

    void CalibrateProcess::addEncoderMeas(double ts, double left, double right) {
        EncoderMeas encoderMeas;
        encoderMeas.ts = Time(ts);
        encoderMeas.meas(0) = left;
        encoderMeas.meas(1) = right;
        encoderBuffer_->insert(TimeToNanoseconds(encoderMeas.ts), encoderMeas.meas);

        fullEncoderMeas_.push_back(Eigen::Vector2d(left, right));
    }

    void CalibrateProcess::addVioPoseMeas(double ts, Pose vioPose) {
        VioPoseMeas vioPoseMeas;
        vioPoseMeas.ts = Time(ts);
        vioPoseMeas.meas = vioPose;

        vioPoseBuffer_->push(vioPoseMeas);
        //ALOGI("NinebotSlam: [OdomCalib] addVioPoseMeas vioPoseBuffer_ before process %d", vioPoseBuffer_->size());

        EncoderMeas interpolated_encoder;

        while(!vioPoseBuffer_->empty()) {
            VioPoseMeas vioPoseMeas_i = vioPoseBuffer_->front();
            if (encoderBuffer_->empty()) {
                //ALOGI("NinebotSlam: [OdomCalib] addVioPoseMeas get encoder from buffer failed, buffer is empty.");
                return;
            }

            int64_t vio_pose_ts_ns = TimeToNanoseconds(vioPoseMeas_i.ts);
            auto oldestEnocder = encoderBuffer_->getOldestValue();
            auto newestEncoder = encoderBuffer_->getNewestValue();

            if (vio_pose_ts_ns < std::get<0>(oldestEnocder)) {
                vioPoseBuffer_->pop();
                //ALOGI("NinebotSlam: [OdomCalib] addVioPoseMeas vio_pose_ts < oldestEnocder");
                continue;
            }

            if (vio_pose_ts_ns > std::get<0>(newestEncoder)) {
                //ALOGI("NinebotSlam: [OdomCalib] addVioPoseMeas vio_pose_ts > newestEncoder");
                return;
            }

            auto interpolated = encoderBuffer_->getValueAtInterpolateIfNeeded(vio_pose_ts_ns);

            if(!std::get<2>(interpolated)) {
                //ALOGI("NinebotSlam: [OdomCalib] addVioPoseMeas can interpolate");
                return;
            }

            MeasPackage measPackage;
            measPackage.ts = vioPoseMeas_i.ts;
            measPackage.T_WC = vioPoseMeas_i.meas;
            measPackage.ticks = std::get<1>(interpolated);

            odomCalibrateSolver_->addMeasPackage(measPackage);
            //ALOGI("NinebotSlam: [OdomCalib] addMeasPackage  ");
            vioPoseBuffer_->pop();
        }
        //ALOGI("NinebotSlam: [OdomCalib] vioPoseBuffer_ after process %d", vioPoseBuffer_->size());
    }

 
    bool CalibrateProcess::calibrateOnFullPath(Eigen::Vector2d& parameter, Eigen::Matrix2d& cov) {
        return odomCalibrateSolver_->calibrateOnFullPath(parameter, cov);
    }


    std::vector<Eigen::Vector3d>
    CalibrateProcess::intergateFullPath(const Eigen::Vector2d& odom_paramter) {
       return odomCalibrateSolver_->intergateFullPath(odom_paramter);
    }
}