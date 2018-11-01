
#include "read_dataset.h"
#include <Eigen/Geometry>

Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R)
{
    Eigen::Vector3d n = R.col(0);
    Eigen::Vector3d o = R.col(1);
    Eigen::Vector3d a = R.col(2);

    Eigen::Vector3d ypr(3);
    double y = atan2(n(1), n(0));
    double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
    double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
    ypr(0) = y;
    ypr(1) = p;
    ypr(2) = r;

    return ypr;
}

std::vector<PoseMeas> loadVioPose (const std::string& vioPoseFile) {
    std::ifstream ifs(vioPoseFile);
    if (ifs.is_open()) {
        std::cout<<"Vio Pose file is opened: " << vioPoseFile << std::endl;
    } else {
        std::cout<<"Vio Pose file is NOT opened: " << vioPoseFile << std::endl;
    }

    std::vector<PoseMeas> PoseMeasVec;

    std::string oneLine;
    while(!ifs.eof()) {
        std::getline(ifs, oneLine);
        std::stringstream stream(oneLine);
        uint64_t  ts;
        double tx, ty, tz;
        double w, x, y, z;
        stream >> ts >> tx >> ty >> tz >> w >> x >> y >> z;

        Eigen::Quaterniond quat(w,x,y,z);
        Eigen::Vector3d ypr = R2ypr(quat.toRotationMatrix());

        Eigen::Matrix<double,3,1> pose;
        pose << tx, ty, ypr[0];

        Time time((double)ts/1e6);
        PoseMeas poseMeas;
        poseMeas.ts  = time;
        poseMeas.meas = pose;
        PoseMeasVec.push_back(poseMeas);
    }
    return PoseMeasVec;
}

std::vector<EncoderMeas> loadEncoder (const std::string& encoderFile) {
    std::ifstream ifs(encoderFile);
    if (ifs.is_open()) {
        std::cout<<"Encoder meas file is opened: " << encoderFile << std::endl;
    } else {
        std::cout<<"Encoder meas file is NOT opened: " << encoderFile << std::endl;
    }

    std::vector<EncoderMeas> encoderMeasVec;

    std::string oneLine;
    while(!ifs.eof()) {
        std::getline(ifs, oneLine);
        std::stringstream stream(oneLine);

        uint64_t  ts;
        double left, right;
        stream >> ts >>  left >> right;

        Time time((double)ts/1e6);
        EncoderMeas encoderMeas;
        encoderMeas.ts  = time;
        encoderMeas.meas = Eigen::Vector2d(left,right);

        encoderMeasVec.push_back(encoderMeas);
    }
    return encoderMeasVec;
}