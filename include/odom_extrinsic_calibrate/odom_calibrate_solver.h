#ifndef _ODOMCALIBRATESOLVER_H_
#define _ODOMCALIBRATESOLVER_H_


#include <iostream>
#include <vector>
#include <memory>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <odom_extrinsic_calibrate/Time.hpp>
#include <fstream>
#include <ceres/ceres.h>


namespace  odom_calib {
    struct ConfigParams {
        double batch_length_ = 10.0;  // in time [second]
        double left_right_tick_diff_th = 200.0; //
        double valid_meas_ratio = 0.4; //

        int min_valid_segmenst_required_th_ = 10;
        int max_valid_segmenst_required_th_ = 40; // do not trust vio moves too long
        double evaluation_trans_error_th_ = 0.3; // [m]
    };


    struct CalibrateResult {
        int all_segment_cnt = 0;
        int valid_segment_cnt = 0;
        int all_meas_cnt = 0;
        int valid_meas_cnt = 0;
        double final_cost = 0;
        double average_final_cost = 0;
        double default_align_error = 0;
        double previous_align_error = 0;
        double this_align_error = 0;
        Eigen::Vector2d estimate_parameter;
        Eigen::Matrix2d covariance;
    };

    struct MeasPackage {
        Time ts;
        Eigen::Matrix<double, 3, 1> T_WC;
        Eigen::Vector2d ticks;
    };

    class Transform2DError: public ceres::SizedCostFunction<2,1,2> {
    public:
        Transform2DError(Eigen::Vector2d x, Eigen::Vector2d y):
        x_(x), y_(y){
        }

        virtual bool Evaluate(double const *const *parameters,
                              double *residuals,
                              double **jacobians) const;
    private:
        Eigen::Vector2d x_;
        Eigen::Vector2d y_;

    };

// todo(pang): add marginalization error part
    class OdomCalibrateSolver {
    public:
        OdomCalibrateSolver(const ConfigParams &params, const Eigen::Isometry3d &T_OC);
        void setPriorEstimate(const CalibrateResult& priorCalibrateResult);
        bool addMeasPackage(const MeasPackage &measPackage);

        bool calibrateOnFullPath(Eigen::Vector2d& parameter, Eigen::Matrix2d& cov);
        std::vector<Eigen::Vector3d>
        intergateFullPath(const Eigen::Vector2d& odom_paramter);
        bool detectBestEst(CalibrateResult& bestEst);
        void recordTrajectories(std::string path);
        
        std::string printCalibReport() {
            std::stringstream ss;
            printCalibReport(ss);
            return ss.str();
        }

        CalibrateResult getBestCalibrateResult();

    private:
        ///< calibrate
        bool solve(const std::vector<MeasPackage>& meas,
                ceres::Solver::Summary &summary, Eigen::Matrix2d& cov);

        bool solveOdomExtrinsic(const std::vector<MeasPackage>& meas,
                   ceres::Solver::Summary &summary, Eigen::Matrix2d& cov);



        bool isValidSegment(std::vector<MeasPackage> &meas_segment);
        double evaluateEstimatedOnFullMeasmests(Eigen::Vector2d &est_paramter);

        bool alignTwoTrajectory(std::vector<Eigen::Vector2d> &traj0,
                                std::vector<Eigen::Vector2d> &traj1,
                                Eigen::Matrix2d rot, Eigen::Vector2d translation,
                                double &trans_error);

        std::vector<Eigen::Vector2d>
        integrate2Dpositions(Eigen::Vector2d odom_paramter,
                          std::vector<MeasPackage>& full_meas,
                          std::vector<Eigen::Vector2d> *vio_2D_positions = NULL);
        void printCalibReport(std::ostream &out);


        int all_segment_cnt_;
        int valid_segment_cnt_;
        std::vector<std::vector<MeasPackage>> valid_segments_;
        std::vector<MeasPackage> full_meas_;
        bool first_massage_ = true;
        Eigen::Vector3d first_vio_pose_;
        std::vector<CalibrateResult> best_est_candidates_;
        CalibrateResult best_estimate_;
        CalibrateResult priorCalibrateResult_;
        CalibrateResult fullPathCalibrateResult_;
        const Eigen::Vector2d default_parameter_;

        std::vector<MeasPackage> meas_buffer_;
        int all_meas_cnt_;
        int valid_meas_cnt_;
        int cur_valid_meas_cnt_;
        ConfigParams params_;
        Eigen::Isometry3d T_OC_;
        // Estimated Parameter
        Eigen::Vector2d estimated_paramter_;
        Eigen::Vector3d cur_integrate_pose_;
        double best_trans_error_ = std::numeric_limits<double>::max();
    };
}
#endif