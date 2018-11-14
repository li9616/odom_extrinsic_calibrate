#include <fstream>
#include <iostream>
#include <Eigen/Geometry>
#include <odom_extrinsic_calibrate/calibrate_process.h>
#include <odom_extrinsic_calibrate/Time.hpp>
#include "utils/read_dataset.h"

using namespace odom_calib;

int main(int argc, char** argv) {

    if (argc < 2) {
        std::cerr << "Usage: \n  ./calib   data_path  [calibrate_result_path] " << std::endl;
        return -1;
    }

    std::string result_save_path
            = "/home/pang/maplab_ws/src/maplab/algorithms/odom_extrinsic_calibrate/calibrate_dataset";

    if (argc >= 3) {
        result_save_path = std::string(argv[2]);
    }

    const std::string dataset_path =  std::string(argv[1]);

    const std::string vioPoseFile
    = dataset_path + "/calibrate_vio.txt";

    const std::string encoderMeasFile
    = dataset_path + "/encoder_meas.txt";

    std::vector<PoseMeas> poseMeasVec = loadVioPose(vioPoseFile);
    std::vector<EncoderMeas> encoderMeasVec = loadEncoder(encoderMeasFile);
    std::shared_ptr<CalibrateProcess> calib_process = std::make_shared<CalibrateProcess>(result_save_path);
    std::vector<EncoderMeas>::iterator encoderItr = encoderMeasVec.begin();
    std::vector<PoseMeas>::iterator vioPoseItr = poseMeasVec.begin();

    for (;vioPoseItr != poseMeasVec.end(); vioPoseItr += 1) {
        Time until(vioPoseItr->ts.toSec() + 0.2);
        while(encoderItr->ts <= until && encoderItr != encoderMeasVec.end()) {
            // add encoder
            calib_process->addEncoderMeas(encoderItr->ts.toSec(), encoderItr->meas[0], encoderItr->meas[1]);
            //std::cout<< "add encoder: " << encoderItr->ts << std::endl;

            encoderItr ++;
        }
        // add vioYaw
        calib_process->addVioPoseMeas(vioPoseItr->ts.toSec(),vioPoseItr->meas);
        //std::cout<< "add yaw: " << yawItr->ts << std::endl;
    }

   
    Eigen::Vector2d parameter;
    Eigen::Matrix2d cov;
    calib_process->calibrateOnFullPath(parameter,cov);

    return 0;
}