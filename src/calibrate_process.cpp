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

//    std::vector<Eigen::Vector2d>
//    getAllSuccessfulCalibrateFromLog(const std::string log_yaml) {
//        std::vector<Eigen::Vector2d> succ_result_vec;
//
//        YAML::Node element = YAML::LoadFile(log_yaml);
//        if(element.size() == 0) return succ_result_vec;
//
//        size_t  sucess_cnt = element["success_calib_cnt"].as<int>();
//        std::cout<< "Successful calibrate cnt: " << sucess_cnt << std::endl;
//        for (YAML::const_iterator it = element.begin(); it != element.end(); ++it){
//            std::string str = it->first.as<std::string>();
//            std::string str1 = str.substr (0,5);
//            if (str1.compare("calib") == 0) {
//                bool good_calib =  element[str]["successful"].as<bool>();
//                if(good_calib) {
//
//                    Eigen::Vector2d est =
//                            Eigen::Vector2d(element[str]["est_parameter"]["left"].as<double>(),
//                                    element[str]["est_parameter"]["right"].as<double>());
//                    succ_result_vec.push_back(est);
//                }
//            }
//        }
//        return succ_result_vec;
//    }


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

        double  val[7];
        if (loadCalibResult(val)) {
//            ALOGI("NinebotSlam: [OdomCalib] load calib result from file");
            Eigen::Matrix2d sigma;
            sigma << val[2], val[3],val[4],val[5];
//
//            ALOGI("NinebotSlam: [OdomCalib] mu: %f %f X1e-5", val[0]*1e5, val[1]*1e5);
//            ALOGI("NinebotSlam: [OdomCalib] radius sigma: %f %f %f %f ",
//                    val[2], val[3], val[4], val[5]);
//            ALOGI("NinebotSlam: [OdomCalib] average final cost: %f ",
//                  val[6]);

            priorCalibrateResult_->estimate_parameter[0]  = val[0] / kRadiusToRot_;
            priorCalibrateResult_->estimate_parameter[1]  = val[1] / kRadiusToRot_;
            priorCalibrateResult_->covariance << val[2], val[3], val[4], val[5];
            priorCalibrateResult_->average_final_cost = val[6];
        } else {
            /// 8.60627e-05, 8.60627e-05
            priorCalibrateResult_->estimate_parameter[0]  = 0.1 ;
            priorCalibrateResult_->estimate_parameter[1]  = 0.1 ;
            priorCalibrateResult_->covariance
                << 0.01, 0, 0, 0.01;
            priorCalibrateResult_->average_final_cost = 1;
//            ALOGI("NinebotSlam: [OdomCalib] do not have calibrate file , we will make it");
        }

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

    void CalibrateProcess::recordTrajectories(std::string path) {
        odomCalibrateSolver_->recordTrajectories(path);
    }
    bool CalibrateProcess::calibrateOnFullPath(Eigen::Vector2d& parameter, Eigen::Matrix2d& cov) {
        return odomCalibrateSolver_->calibrateOnFullPath(parameter, cov);
    }


    std::vector<Eigen::Vector3d>
    CalibrateProcess::intergateFullPath(const Eigen::Vector2d& odom_paramter) {
       return odomCalibrateSolver_->intergateFullPath(odom_paramter);
    }

    double CalibrateProcess::compareIntergatedOdomWithVioPose(Eigen::Vector2d& paramter){
        return odomCalibrateSolver_->compareIntergatedOdomWithVioPose(paramter);
    }


    void CalibrateProcess::getCalibrateResult() {

        newCalibrateResult_ = std::make_shared<CalibrateResult>();
        CalibrateResult calibrateResult;
        if (!odomCalibrateSolver_->detectBestEst(calibrateResult)) {
            *newCalibrateResult_ = *priorCalibrateResult_;

            std::string comments = "[Fail] Do Not get satisfied calibrate result for solver.";
            std::string log = printCalibrateReport(comments);
            //calib_log_ofs_ << log;

            std::cout<< log << std::endl;
//            writeCalibrateLogToYaml(false, comments);
//            saveCalibResult();
            return ;
        }

        // Rational check
        bool far_from_default = (fabs(calibrateResult.estimate_parameter[0]/0.1 - 1 ) > 0.05)
                                || (fabs(calibrateResult.estimate_parameter[1]/0.1 -1) > 0.05);
        if (far_from_default ){
//            ALOGD("StateEstimate: failed to load calib result, state is too far away from default. ");
            *newCalibrateResult_ = *priorCalibrateResult_;
            std::string comments = "[Fail] State is too far away from default.";
            std::string log = printCalibrateReport(comments);
            //calib_log_ofs_ << log;
            std::cout<< log << std::endl;
//            writeCalibrateLogToYaml(false, comments);
//            saveCalibResult();
            return ;
        }
        else if ( (calibrateResult.covariance(0,0) >= 0.015)
            || (calibrateResult.covariance(1,1) >= 0.015)) {
//            ALOGD("StateEstimate: failed to load calib result, covariance is much worse than default. ");
            *newCalibrateResult_ = *priorCalibrateResult_;
            std::cout<< "fail type 2" << std::endl;

            std::string comments = "[Fail] Covariance is much worse than default.";
            std::string log = printCalibrateReport(comments);
            //calib_log_ofs_ << log;
            std::cout<< log << std::endl;
//            writeCalibrateLogToYaml(false, comments);
//            saveCalibResult();
            return ;
        }
        // whether the new one is superior to the prior one;
        Eigen::Matrix2d newCov = calibrateResult.covariance;
        double newUncertainty = newCov(0,0) + newCov(1,1);
        double oldUncertainty =
                priorCalibrateResult_->covariance(0,0) + priorCalibrateResult_->covariance(1,1);
        if (newUncertainty < oldUncertainty
         && calibrateResult.average_final_cost < priorCalibrateResult_->average_final_cost) {
            *newCalibrateResult_ = calibrateResult;
            std::string comments = "[Success]";
            std::string log = printCalibrateReport(comments);

            //calib_log_ofs_ << log;
            std::cout<< log << std::endl;
//            writeCalibrateLogToYaml(true, comments);

//            saveCalibResult();
        } else {
            *newCalibrateResult_ = *priorCalibrateResult_;

            std::string comments = "[Fail] Covariance is much worse than prevous or "
                                   "average final cost larger than previous.";
            std::string log = printCalibrateReport(comments);
            //calib_log_ofs_ << log;
            std::cout<< log << std::endl;
//            writeCalibrateLogToYaml(false, comments);
//            saveCalibResult();
        }

        return ;
    }

//    void CalibrateProcess::writeCalibrateLogToYaml(bool successful, std::string comments) {
//        // todo
//        YAML::Node element = YAML::LoadFile(calib_log_yaml_);
//        int current_calib_cnt = 0;
//        int current_success_calib_cnt = 0;
//        if (element.size()) {
//            current_calib_cnt = element["total_calib_cnt"].as<int>();
//            current_success_calib_cnt = element["success_calib_cnt"].as<int>();
//        } else {
//            element["success_calib_cnt"] = 0;
//        }
//
//        element["total_calib_cnt"] = current_calib_cnt + 1;
//        if (successful){
//            element["success_calib_cnt"] = current_success_calib_cnt + 1;
//        }
//
//        std::string next_calib_tag = "calib_" + boost::lexical_cast<std::string>(current_calib_cnt);
//        element[next_calib_tag]["timestamp"] = timeOfDay();
//        element[next_calib_tag]["successful"] = successful;
//        CalibrateResult cr = odomCalibrateSolver_->getBestCalibrateResult();
//        element[next_calib_tag]["all_meas_cnt"] = cr.all_meas_cnt;
//        element[next_calib_tag]["valid_meas_cnt"] = cr.valid_meas_cnt;
//        element[next_calib_tag]["all_segment_cnt"] = cr.all_segment_cnt;
//        element[next_calib_tag]["valid_segment_cnt"] = cr.valid_segment_cnt;
//        element[next_calib_tag]["final_cost"] = cr.final_cost;
//        element[next_calib_tag]["average_final_cost"] = cr.average_final_cost;
//        element[next_calib_tag]["this_align_error"] = cr.this_align_error;
//        element[next_calib_tag]["default_align_error"] = cr.default_align_error;
//        element[next_calib_tag]["previous_align_error"] = cr.previous_align_error;
//        element[next_calib_tag]["est_parameter"]["left"] = cr.estimate_parameter[0];
//        element[next_calib_tag]["est_parameter"]["right"] = cr.estimate_parameter[1];
//        element[next_calib_tag]["uncertaity"] = cr.covariance(0,0) + cr.covariance(1,1);
//        element[next_calib_tag]["prev_est_parameter"]["left"] = priorCalibrateResult_->estimate_parameter[0];
//        element[next_calib_tag]["prev_est_parameter"]["right"] = priorCalibrateResult_->estimate_parameter[1];
//        element[next_calib_tag]["prev_uncertaity"] = priorCalibrateResult_->covariance(0,0)
//                + priorCalibrateResult_->covariance(1,1);
//        element[next_calib_tag]["comments"] = comments;
//
//
//        // write
//        std::ofstream fout(calib_log_yaml_);
//        fout << element;
//        fout.close();
//    }

    std::string CalibrateProcess::printCalibrateReport(std::string comments) {
        std::string log;
        log  = "\n<<<<<<<<<<<<<<<<<<<<<<< " + timeOfDay() + " <<<<<<<<<<<<<<<<<<<<<<<<<<< ";
        log += odomCalibrateSolver_->printCalibReport();
        log += "\n" + comments + "\n";
        log += "previous calib result: \n";
        log += boost::lexical_cast<std::string>(priorCalibrateResult_->estimate_parameter[0]) + " "
                + boost::lexical_cast<std::string>(priorCalibrateResult_->estimate_parameter[1]) + "\n";

        log += "previous cov: \n";
        log += boost::lexical_cast<std::string>(priorCalibrateResult_->covariance(0,0)) + " "
               + boost::lexical_cast<std::string>(priorCalibrateResult_->covariance(0,1)) + "\n";
        log += boost::lexical_cast<std::string>(priorCalibrateResult_->covariance(1,0)) + " "
               + boost::lexical_cast<std::string>(priorCalibrateResult_->covariance(1,1)) + "\n";
        log += "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< End <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< \n\n\n";

        return log;
    }

    bool CalibrateProcess::loadCalibResult(double *val){
        std::string file_result_name_ = calib_file_folder_ + "/result_odom_calib.txt";
        std::ifstream readResultFile;
        std::string snStringCalib;
        readResultFile.open(file_result_name_, std::istream::in);
        if (readResultFile.is_open()){
            readResultFile >> snStringCalib;
            readResultFile >> val[0];
            readResultFile >> val[1];
            readResultFile >> val[2];
            readResultFile >> val[3];
            readResultFile >> val[4];
            readResultFile >> val[5];
            readResultFile >> val[6];
            readResultFile.close();
        }
        else {
//            ALOGD("StateEstimate: failed to open calib result file. ");
            return false;
        }

#if defined(ANDROID) || defined(__ANDROID__)

		std::string snString = GetStdoutFromCommand("getprop ro.serialno");
		snString.resize(snString.size()-1);
		if (snString!=snStringCalib){
			ALOGD("StateEstimate: failed to load calib result, wrong S/N number in calib file. ");
			ALOGD("StateEstimate: robot device S/N = %s", snString.c_str());
			ALOGD("StateEstimate: calib file S/N = %s", snStringCalib.c_str());
			return false;
		}
#endif
        return true;
    }

//    void CalibrateProcess::saveCalibResult() {
//        Eigen::Vector2d mu ;
//        Eigen::Matrix2d sigma ;
//
//        if (!common::pathExists(calib_file_folder_)) {
//            common::createPath(calib_file_folder_);
//        }
//
//        std::string file_result_name_ = calib_file_folder_ + "/result_odom_calib.txt";
//        std::ofstream writeResultFile;
//        writeResultFile.open(file_result_name_);
//
//#if defined(ANDROID) || defined(__ANDROID__)
//        std::string snString = GetStdoutFromCommand("getprop ro.serialno");
//        snString.resize(snString.size()-1);
//#else
//        std::string snString = "111111111";
//#endif
//
//
//        mu[0] = newCalibrateResult_->estimate_parameter[0] * kRadiusToRot_;
//        mu[1] = newCalibrateResult_->estimate_parameter[1] * kRadiusToRot_;
//
//        sigma = newCalibrateResult_->covariance;
//        double average_final_cost = newCalibrateResult_->average_final_cost;
////        ALOGI("NinebotSlam: [OdomCalib] save calib result to file");
////        ALOGI("NinebotSlam: [OdomCalib] mu: %f %f X1e-5", mu[0]*1e5, mu[1]*1e5);
////        ALOGI("NinebotSlam: [OdomCalib] radius sigma: %f %f %f %f ",
////                sigma(0,0), sigma(0,1), sigma(1,0), sigma(1,1));
////
////        ALOGI("NinebotSlam: [OdomCalib] average final cost: %f ",
////              average_final_cost);
//        if (writeResultFile.is_open()){
//            writeResultFile << snString << "\n" << std::setprecision(16)
//                            << mu(0) << "\n"
//                            << mu(1) << "\n"
//                            << sigma(0,0) << "\n"
//                            << sigma(0,1) << "\n"
//                            << sigma(1,0) << "\n"
//                            << sigma(1,1) << "\n"
//                            << average_final_cost << "\n"
//                            << newCalibrateResult_->estimate_parameter[0] << "\n"
//                            << newCalibrateResult_->estimate_parameter[1]
//                            << std::endl;
//            writeResultFile.close();
//        }
////        ALOGD("NinebotSlam: [OdomCalib] succeeded to save calib result: %s",
////              file_result_name_.c_str());
//    }
}