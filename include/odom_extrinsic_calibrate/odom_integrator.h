#ifndef _ODOM_INTEGRATOR_H_
#define _ODOM_INTEGRATOR_H_
#include <Eigen/Core>


namespace odom_calib {

    class OdomIntergrator {
    public:
        OdomIntergrator() {}
        Eigen::Vector3d computeOdom(const Eigen::Vector2d &ticks,
                                    const Eigen::Vector2d parameter,
                                    const Eigen::Vector3d init_pose) const {
            Eigen::Vector3d pose_odom;

            double left_ticks = ticks[0];
            double right_ticks = ticks[1];
            if(left_ticks<-2147483648){ // if wrap around in positive direction
                left_ticks += 4294967296;
            }
            else if(left_ticks>21474836488){ // if wrap around in negative direction
                left_ticks -= 4294967296;
            }

            if(right_ticks<-2147483648){ // if wrap around in positive direction
                right_ticks += 4294967296;
            }
            else if(right_ticks>21474836488){ // if wrap around in negative direction
                right_ticks -= 4294967296;
            }


            double delta_s_l = left_ticks * parameter[0] * rad_per_tick_;
            double delta_s_r = right_ticks * parameter[1] * rad_per_tick_;
            double delta_s = (delta_s_l + delta_s_r) * 0.5;
            double delta_theta = (delta_s_r - delta_s_l) / baseline_;
            double theta = init_pose[2];
            pose_odom = Eigen::Matrix<double, 3, 1>(delta_s * cos(theta + 0.5 * delta_theta),
                                                    delta_s * sin(theta + 0.5 * delta_theta),
                                                    delta_theta);

            Eigen::Vector3d res_pose = init_pose + pose_odom;
            if (res_pose[2] > M_PI) {
                res_pose[2] = res_pose[2] - 2* M_PI;
            }else if (res_pose[2] < -M_PI) {
                res_pose[2] = res_pose[2] + 2* M_PI;
            }
            return res_pose;
        }

    private:
        const double baseline_ = 0.4456;
        const double rad_per_tick_ = 2.0 * M_PI / 16384;
    };
}

#endif