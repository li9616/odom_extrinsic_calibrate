#include "odom_extrinsic_calibrate/utils.h"
#include <Eigen/Geometry>
#include <ctime>
namespace utils {
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
    Sophus::SE2d poseVector2SE2d(const Eigen::Matrix<double,7,1>& poseVector) {

        Eigen::Matrix3d R = Eigen::Quaterniond(poseVector[3],
                poseVector[4],poseVector[5],poseVector[6]).toRotationMatrix();
        double yaw = R2ypr(R)[2];
        Sophus::SE2d se2d(yaw, Eigen::Vector2d(poseVector[0], poseVector[1]));
        return se2d;
    }


}