#ifndef _READ_DATASET_H_
#define _READ_DATASET_H_
#include <fstream>
#include <string>
#include <odom_extrinsic_calibrate/types.h>
using namespace odom_calib;
Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R);
std::vector<PoseMeas> loadVioPose (const std::string& vioPoseFile);
std::vector<EncoderMeas> loadEncoder (const std::string& encoderFile);
#endif