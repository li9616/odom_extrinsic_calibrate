#include "odom_extrinsic_calibrate/odom_calibrate_solver.h"

#include <iomanip>

#include <odom_extrinsic_calibrate/OdomCalibrateError.h>
#include <odom_extrinsic_calibrate/odom_integrator.h>
#include "odom_extrinsic_calibrate/utils.h"
#include "odom_extrinsic_calibrate/local_parameterization_se2.hpp"
#include "odom_extrinsic_calibrate/OdomExtrinsicCalibrateError.h"
namespace  odom_calib {
    bool Transform2DError::Evaluate(double const *const *parameters,
                          double *residuals,
                          double **jacobians) const {

        const double theta = parameters[0][0];
        Eigen::Map<const Eigen::Vector2d> trans(parameters[1]);

        double c = cos(theta);
        double s = sin(theta);
        Eigen::Matrix2d R;
        R << c, -s, s, c;

        Eigen::Map<Eigen::Vector2d> error(residuals);
        error = R*x_ + trans - y_;
        if(jacobians != NULL) {
            if(jacobians[0] != NULL) {
                Eigen::Map<Eigen::Matrix<double,2,1>> jacobian0(jacobians[0]);
                jacobian0 << - x_[0]*s - x_[1]*c,
                            x_[0]*c - x_[1]*s;
            }
            if(jacobians[1] != NULL) {
                Eigen::Map<Eigen::Matrix<double,2,2, Eigen::RowMajor>> jacobian1(jacobians[1]);
                jacobian1 = Eigen::Matrix2d::Identity();
            }
        }
        return true;
    }
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


       if (latest_ts.toSec() - oldest_ts.toSec() > params_.batch_length_) {

            if(isValidSegment) {
                ceres::Solver::Summary summary;

//                std::cout<< "cur_valid_meas_cnt_: " << cur_valid_meas_cnt_ << std::endl;

                Eigen::Matrix2d cov = Eigen::Matrix2d::Identity();
                bool success = solveOdomExtrinsic(meas_buffer_, summary, cov);
                if (success) {
                    std::cout << summary.BriefReport() << std::endl;
                    std::cout << "final cast: " << summary.final_cost << std::endl;
                    std::cout<< "Noised parameter after Opt. " << estimated_paramter_.transpose() << std::endl;

                    CalibrateResult calibrateResult;
                    calibrateResult.final_cost = summary.final_cost;
                    calibrateResult.covariance = cov;
                    calibrateResult.estimate_parameter = estimated_paramter_;
                    calibrateResult.valid_meas_cnt = cur_valid_meas_cnt_;
                    calibrateResult.average_final_cost = summary.final_cost / cur_valid_meas_cnt_;
                    best_est_candidates_.push_back(calibrateResult);
                }
                valid_segment_cnt_ ++;
                valid_meas_cnt_ += cur_valid_meas_cnt_;
            }

            // clear buffer
            meas_buffer_.clear();
            cur_valid_meas_cnt_ = 0;
            all_segment_cnt_ ++;
        }
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



    bool OdomCalibrateSolver::solve(const std::vector<MeasPackage>& meas, ceres::Solver::Summary& summary, Eigen::Matrix2d& cov) {
        // Build the problem.
        ceres::Problem problem;
        // Specify local update rule for our parameter
        problem.AddParameterBlock(estimated_paramter_.data(), 2);
        OdomIntergrator odomIntergrator;

        std::vector<MeasPackage>::const_iterator measItr = meas.begin();
        for (; measItr + 1 != meas.end(); measItr ++) {
            MeasPackage measPackage0 = *measItr;
            MeasPackage measPackage1 = *(measItr+1);
            Sophus::SE2d T_WC0 = vector2SE2(measPackage0.T_WC);
            Sophus::SE2d T_WC1 = vector2SE2(measPackage1.T_WC);

            Sophus::SE2d T_OC(0, T_OC_.translation().head(2));

            ceres::CostFunction* cost_function =
                        new OdomCalibrateError2(T_WC0, T_WC1,
                                                measPackage0.ticks, measPackage1.ticks,
                                                T_OC);
            problem.AddResidualBlock(cost_function, NULL, estimated_paramter_.data());

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
//
        return true;
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


    bool OdomCalibrateSolver::detectBestEst(CalibrateResult& bestEst) {
        std::vector<Eigen::Vector2d> vio_2d_positions;
        double first_integrate = true;

        std::vector<Eigen::Vector2d> intergate_2d_position;
        Eigen::Matrix2d rot;
        Eigen::Vector2d translation;
        double trans_error;

        for (auto  candidate: best_est_candidates_) {
            //std::cout<< "candidate: " << candidate.estimate_parameter.transpose() << std::endl;
            // integerate odom

            if (first_integrate) {
                intergate_2d_position =
                        integrate2Dpositions(candidate.estimate_parameter, full_meas_, &vio_2d_positions);
                first_integrate = false;
            } else {
                intergate_2d_position =
                        integrate2Dpositions(candidate.estimate_parameter, full_meas_);
            }


            // align odom and vio positions
            alignTwoTrajectory(vio_2d_positions, intergate_2d_position, rot, translation, trans_error);

            if (best_trans_error_ > trans_error) {
                best_trans_error_ = trans_error;
                bestEst = candidate;
                best_estimate_ = candidate;
            }
        }


        best_estimate_.all_segment_cnt = all_segment_cnt_;
        best_estimate_.valid_segment_cnt = valid_segment_cnt_;
        best_estimate_.all_meas_cnt = all_meas_cnt_;

//        Eigen::Vector2d parameter;
//        Eigen::Matrix2d cov;
//
//        calibrateOnFullPath(parameter, cov);
//        std::cout<< "parameter: " << parameter.transpose() << std::endl;
        double average_trans_error = best_trans_error_ / full_meas_.size();
        best_estimate_.this_align_error = average_trans_error;
        if (average_trans_error > params_.evaluation_trans_error_th_) {
            std::cerr << "average_trans_error: " << average_trans_error << std::endl;
            return false;
        }

        // Is the better than default parameter?
        double defualt_trans_error = -1;
        std::vector<Eigen::Vector2d> delfault_intergate_2d_position =
                integrate2Dpositions(default_parameter_, full_meas_);
        alignTwoTrajectory(vio_2d_positions, delfault_intergate_2d_position,
                rot, translation, defualt_trans_error);

        best_estimate_.default_align_error =
                defualt_trans_error / full_meas_.size();
        if (best_trans_error_ > defualt_trans_error) {
            return false;
        }


        // Is this better than previous one
        double previous_trans_error = -1;
        std::vector<Eigen::Vector2d> previous_intergate_2d_position =
                integrate2Dpositions(priorCalibrateResult_.estimate_parameter, full_meas_);
        alignTwoTrajectory(vio_2d_positions, previous_intergate_2d_position,
                           rot, translation, previous_trans_error);

        best_estimate_.previous_align_error =
                previous_trans_error / full_meas_.size();

        if (best_trans_error_ > previous_trans_error) {
            return false;
        }


        if (best_est_candidates_.size() < params_.min_valid_segmenst_required_th_
            || best_est_candidates_.size() > params_.max_valid_segmenst_required_th_)
            return false;


        return true;
    }


    double OdomCalibrateSolver::compareIntergatedOdomWithVioPose(Eigen::Vector2d& paramter) {
        double trans_error = 10000;
        std::vector<Eigen::Vector2d> vio_2d_positions;
        std::vector<Eigen::Vector2d> intergate_2d_position;
        Eigen::Matrix2d rot;
        Eigen::Vector2d translation;
        intergate_2d_position =
                integrate2Dpositions(paramter, full_meas_, &vio_2d_positions);
        // align odom and vio positions
        alignTwoTrajectory(vio_2d_positions, intergate_2d_position, rot, translation, trans_error);

        return trans_error;
    }

    double OdomCalibrateSolver::evaluateEstimatedOnFullMeasmests(Eigen::Vector2d& est_paramter) {
        double rmse = 0;
        std::vector<MeasPackage>::const_iterator measItr = full_meas_.begin();

        OdomIntergrator odomIntergrator;
        int cnt = 0;
        for (; measItr + 1 != full_meas_.end(); measItr ++) {
            Eigen::Vector2d ticks0 = measItr->ticks;
            Eigen::Vector2d ticks1 = (measItr+1)->ticks;

            Eigen::Vector2d delta_ticks = ticks1 - ticks0;

            Eigen::Vector3d delta_pose = odomIntergrator.computeOdom(delta_ticks, est_paramter,
                                                             Eigen::Vector3d::Zero());

            Sophus::SE2d T_WC0 =vector2SE2((measItr)->T_WC);
            Sophus::SE2d T_WC1 =vector2SE2((measItr+1)->T_WC);
            Sophus::SE2d T_C0C1 = T_WC0.inverse()*T_WC1;
            Eigen::Vector3d vio_delta_pose = SE22vector(T_C0C1);

            rmse += (delta_pose - vio_delta_pose).transpose() * (delta_pose - vio_delta_pose);
            cnt ++;
        }

        rmse /= cnt;

        return rmse;
    }


    bool OdomCalibrateSolver::alignTwoTrajectory(std::vector<Eigen::Vector2d>& traj0,
            std::vector<Eigen::Vector2d>& traj1, Eigen::Matrix2d rot, Eigen::Vector2d translation,
            double& trans_error) {

        double theta = 0;
        Eigen::Vector2d trans = Eigen::Vector2d::Zero();

        // Build the problem.
        ceres::Problem problem;
        // Specify local update rule for our parameter
        problem.AddParameterBlock(&theta, 1);
        problem.AddParameterBlock(trans.data(), 2);

        std::vector<Eigen::Vector2d>::const_iterator traj0_itr = traj0.begin();
        std::vector<Eigen::Vector2d>::const_iterator traj1_itr = traj1.begin();

        for (; traj0_itr != traj0.end() && traj1_itr != traj1.end(); traj0_itr++ , traj1_itr++) {
            ceres::CostFunction* costFunction = new Transform2DError(*traj0_itr, *traj1_itr);
            problem.AddResidualBlock(costFunction, NULL, &theta, trans.data());
        }
        // Set solver options (precision / method)
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_SCHUR;
        options.trust_region_strategy_type = ceres::DOGLEG;
        options.max_num_iterations = 10;
        //options.minimizer_progress_to_stdout = true;
        ceres::Solver::Summary summary;
        Solve(options, &problem, &summary);
//        std::cout<< "align trajecotries:  rot : "
//                    << theta << " trans: " << trans.transpose()
//                    << " final cost: " <<  summary.final_cost << std::endl;

        double s = sin(theta);
        double c = cos(theta);
        rot << c,-s, s ,c;
        translation = trans;

        trans_error = summary.final_cost;
        return true;
    }

    CalibrateResult OdomCalibrateSolver::getBestCalibrateResult() {
        return best_estimate_;
    }

    void OdomCalibrateSolver::printCalibReport(std::ostream &out) {
        out << "\n";
        out << "--------------- Odom Calibrate Report Begin ---------------" << std::endl;
        out << "Total Segments: " << all_segment_cnt_
            << " " << " Valid Segments: " << valid_segment_cnt_ << std::endl;

        out << "Total Measurements: " << all_meas_cnt_
            << " " << " Valid Measurements: " << valid_meas_cnt_ << std::endl;

        out << "Best Segment Valid Measurements: " << best_estimate_.valid_meas_cnt << std::endl;
        out << "Final cost : " << best_estimate_.final_cost
            << " Normalize: " << best_estimate_.final_cost / best_estimate_.valid_meas_cnt << std::endl;
        out << "Best alignment error: " << best_estimate_.this_align_error << " Should smaller than "
            << params_.evaluation_trans_error_th_ << std::endl;
        out << "Default alignment error: " << best_estimate_.default_align_error << std::endl;
        out << "Previous alignment error: " << best_estimate_.previous_align_error << std::endl;
        out << "Cov : \n" << best_estimate_.covariance << std::endl;

        out << "Estimate parameter: " << best_estimate_.estimate_parameter[0]
                << " " << best_estimate_.estimate_parameter[1] << std::endl;
        out << "--------------- Odom Calibrate Report End   ---------------" << std::endl;
    }
}