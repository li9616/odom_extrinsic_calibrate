#include "odom_extrinsic_calibrate/odom_calibrate_solver.h"

#include <iomanip>

#include <odom_extrinsic_calibrate/OdomCalibrateError.h>
#include <odom_extrinsic_calibrate/odom_integrator.h>
#include "odom_extrinsic_calibrate/utils.h"
#include "odom_extrinsic_calibrate/local_parameterization_se2.hpp"
#include "odom_extrinsic_calibrate/OdomExtrinsicCalibrateError.h"
namespace  odom_calib {
    OdomCalibrateSolver::OdomCalibrateSolver(const ConfigParams &params, const Eigen::Isometry3d& T_OC):
            all_segment_cnt_(0),
            valid_segment_cnt_(0),
            params_(params),T_OC_(T_OC),
            default_parameter_(0.1,0.1),
            all_meas_cnt_(0),
            valid_meas_cnt_(0),
            cur_valid_meas_cnt_(0){
        estimated_paramter_ = Eigen::Vector2d(0.1, 0.1);
        cur_integrate_pose_ << 0,0,0;
    }

    std::vector<Eigen::Vector3d>
    OdomCalibrateSolver::intergateFullPath(const Eigen::Vector2d& odom_paramter) {
        std::vector<Eigen::Vector3d> integrate;
        std::vector<MeasPackage>::const_iterator measItr = full_meas_.begin();
        OdomIntergrator odomIntergrator;
        Eigen::Vector3d pose(0,0,0);
        for (; measItr + 1 != full_meas_.end(); measItr ++) {
            Eigen::Vector2d ticks0 = measItr->ticks;
            Eigen::Vector2d ticks1 = (measItr+1)->ticks;

            Eigen::Vector2d delta_ticks = ticks1 - ticks0;
            pose = odomIntergrator.computeOdom(delta_ticks, odom_paramter,
                                               pose);
            integrate.push_back(pose);
        }
        return  integrate;
    }

    void OdomCalibrateSolver::setPriorEstimate(const CalibrateResult& priorCalibrateResult) {
        priorCalibrateResult_ = priorCalibrateResult;
        best_estimate_ = priorCalibrateResult_;
    }
    bool OdomCalibrateSolver::addMeasPackage(const MeasPackage &meas) {
        MeasPackage measPackage = meas;
        // align to first pose
        if (first_massage_) {
            first_vio_pose_ = measPackage.T_WC;
            measPackage.T_WC = Eigen::Matrix<double, 3, 1>::Zero();
            first_massage_ = false;
        } else {
            measPackage.T_WC =
                    SE22vector(vector2SE2(first_vio_pose_).inverse()*vector2SE2(measPackage.T_WC));
        }

        if(meas_buffer_.size() > 0) {
            Eigen::Vector2d delta_ticks =  measPackage.ticks - meas_buffer_.back().ticks;
            double left_right_diff = std::abs(delta_ticks[0] - delta_ticks[1]);


            if(left_right_diff > params_.left_right_tick_diff_th) {
                cur_valid_meas_cnt_ ++;
            }
        }

        all_meas_cnt_ ++;

        bool isValidSegment = false;
        if (cur_valid_meas_cnt_ > params_.batch_length_ / 0.1 * params_.valid_meas_ratio) {
            isValidSegment = true;
        }

        full_meas_.push_back(measPackage);
        meas_buffer_.push_back(measPackage);

        Time oldest_ts = meas_buffer_.front().ts;
        Time latest_ts = meas_buffer_.back().ts;

        return true;
    }

    bool OdomCalibrateSolver::calibrateOnFullPath(Eigen::Vector2d& parameter, Eigen::Matrix2d& cov) {
        if (full_meas_.empty())
            false;

        ceres::Solver::Summary summary;
        cov = Eigen::Matrix2d::Identity();
        bool success = solveOdomExtrinsic(full_meas_, summary, cov);
        if (success) {
//            std::cout << "Full path calibrate" << std::endl;
//            std::cout << summary.BriefReport() << std::endl;
//            std::cout << "final cast: " << summary.final_cost << std::endl;
//            std::cout<< "Noised parameter after Opt. " << estimated_paramter_.transpose() << std::endl;
            parameter = estimated_paramter_;
          return true;
        }

        return false;

    }

    void OdomCalibrateSolver::recordTrajectories(std::string path) {
        if (full_meas_.empty())
            return;

        std::ofstream original_integrate_odom;
        std::ofstream re_integrate_odom;
        std::ofstream vio_odom;

        original_integrate_odom.open(path + "/original_integrate_odom.txt");
        re_integrate_odom.open(path + "/re_integrate_odom.txt");
        vio_odom.open(path + "/vio_odom.txt");
        OdomIntergrator odomIntergrator;

        Eigen::Vector2d original_odom_parameter(0.1, 0.1);
//        Eigen::Vector2d est_odom_paramter = best_estimate_.estimate_parameter;
        Eigen::Vector2d est_odom_paramter = best_estimate_.estimate_parameter;

        std::vector<MeasPackage>::const_iterator measItr = full_meas_.begin();
        Eigen::Vector3d original_intergrate_pose = Eigen::Vector3d::Zero();
        Eigen::Vector3d re_intergrate_pose = Eigen::Vector3d::Zero();

        for (; measItr + 1 != full_meas_.end(); measItr ++) {
            Eigen::Vector2d ticks0 = measItr->ticks;
            Eigen::Vector2d ticks1 = (measItr+1)->ticks;

            Eigen::Vector2d delta_ticks = ticks1 - ticks0;
//            std::cout<< "delta ticks: " << delta_ticks.transpose() << std::endl;

            original_intergrate_pose =
                    odomIntergrator.computeOdom(delta_ticks,
                            original_odom_parameter,
                            original_intergrate_pose);


            re_intergrate_pose =
                    odomIntergrator.computeOdom(delta_ticks,
                            est_odom_paramter,
                            re_intergrate_pose);

            original_integrate_odom << original_intergrate_pose[0] << " "
                                    << original_intergrate_pose[1] << " "
                                    << original_intergrate_pose[2] << " "
                                    << std::setprecision(8) <<
                                        (measItr+1)->ticks[0] << " "
                                    << std::setprecision(8) <<
                                    (measItr+1)->ticks[1] << std::endl;

            re_integrate_odom << re_intergrate_pose[0] << " "
                                    << re_intergrate_pose[1] << " "
                                    << re_intergrate_pose[2] << " "
                                    << std::setprecision(8) <<
                                    (measItr+1)->ticks[0] << " "
                                    << std::setprecision(8) <<
                                    (measItr+1)->ticks[1] << std::endl;

            Eigen::Vector3d cur_vio_pose = (measItr+1)->T_WC;
            vio_odom << cur_vio_pose[0] << " "
                      << cur_vio_pose[1] << " "
                      << cur_vio_pose[2] << std::endl;

        }


        original_integrate_odom.close();
        re_integrate_odom.close();
        vio_odom.close();
    }


    bool OdomCalibrateSolver::solveOdomExtrinsic(const std::vector<MeasPackage>& meas, ceres::Solver::Summary& summary, Eigen::Matrix2d& cov) {
        // Build the problem.
        ceres::Problem problem;
        // Specify local update rule for our parameter
        problem.AddParameterBlock(estimated_paramter_.data(), 2);
        Sophus::SE2d T_OC(0, T_OC_.translation().head(2));

        Sophus::test::LocalParameterizationSE2* localParameterizationSE2 = new Sophus::test::LocalParameterizationSE2();
        problem.AddParameterBlock(T_OC.data(), 4, localParameterizationSE2);
        OdomIntergrator odomIntergrator;

        std::vector<MeasPackage>::const_iterator measItr = meas.begin();
        for (; measItr + 1 != meas.end(); measItr ++) {
            MeasPackage measPackage0 = *measItr;
            MeasPackage measPackage1 = *(measItr+1);
            Sophus::SE2d T_WC0 = vector2SE2(measPackage0.T_WC);
            Sophus::SE2d T_WC1 = vector2SE2(measPackage1.T_WC);


            ceres::CostFunction* cost_function =
                    new OdomExtrinsicCalibrateError(T_WC0, T_WC1,
                                            measPackage0.ticks, measPackage1.ticks);
            problem.AddResidualBlock(cost_function, NULL, estimated_paramter_.data(), T_OC.data());

        }

        // Set solver options (precision / method)
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_SCHUR;
        options.trust_region_strategy_type = ceres::DOGLEG;
        options.max_num_iterations = 10;
        options.minimizer_progress_to_stdout = false;

        // Solve
        Solve(options, &problem, &summary);
        if(summary.termination_type != 0) {
            return false;
        }

        ceres::Covariance::Options cov_options;
        ceres::Covariance covariance(cov_options);

        std::vector<std::pair<const double*, const double*> > covariance_blocks;
        covariance_blocks.push_back(std::make_pair(estimated_paramter_.data(),
                                                   estimated_paramter_.data()));

        CHECK(covariance.Compute(covariance_blocks, &problem));

        double covariance_xx[2 * 2];
        covariance.GetCovarianceBlock(estimated_paramter_.data(),
                                      estimated_paramter_.data(), covariance_xx);

        Eigen::Map<Eigen::Matrix<double,2,2,Eigen::RowMajor>> temp_cov(covariance_xx);
        cov = temp_cov;
//        std::cout<< "covaraince: \n" << temp_cov << std::endl;

        std::cout<< "odom_parameter: " << estimated_paramter_.transpose() << std::endl;
        std::cout<< "T_OC parameter: " << T_OC.params().transpose() << std::endl;
//
        return true;
    }


    std::vector<Eigen::Vector2d> OdomCalibrateSolver::integrate2Dpositions(Eigen::Vector2d odom_paramter,
                                                               std::vector<MeasPackage>& full_meas,
                                                               std::vector<Eigen::Vector2d>* vio_2D_positions) {
        std::vector<Eigen::Vector2d> integrate_2d_positions;
        std::vector<MeasPackage>::const_iterator measItr = full_meas.begin();
        OdomIntergrator odomIntergrator;
        Eigen::Vector3d pose(0,0,0);
        for (; measItr + 1 != full_meas_.end(); measItr ++) {
            Eigen::Vector2d ticks0 = measItr->ticks;
            Eigen::Vector2d ticks1 = (measItr+1)->ticks;

            Eigen::Vector2d delta_ticks = ticks1 - ticks0;
            pose = odomIntergrator.computeOdom(delta_ticks, odom_paramter,
                                               pose);
            integrate_2d_positions.push_back(pose.head<2>());
            if (vio_2D_positions) {
                vio_2D_positions->push_back((measItr+1)->T_WC.head<2>());
            }
        }
        return  integrate_2d_positions;
    }
}