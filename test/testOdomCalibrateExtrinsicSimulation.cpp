#include <iostream>
#include <ceres/ceres.h>
#include <odom_extrinsic_calibrate/OdomExtrinsicCalibrateError.h>
#include <odom_extrinsic_calibrate/NumbDifferentiator.hpp>
#include <odom_extrinsic_calibrate/calibrate_process.h>
#include <odom_extrinsic_calibrate/odom_calibrate_solver.h>
#include <odom_extrinsic_calibrate/utils.h>
#include <odom_extrinsic_calibrate/odom_integrator.h>
#include <odom_extrinsic_calibrate/local_parameterization_se2.hpp>
using namespace   odom_calib;

struct SimulateMeasurement {
    Time ts;
    Sophus::SE2d T_WO;
    Sophus::SE2d T_WC;
};

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


std::vector<SimulateMeasurement> simulateVioPose(const std::vector<EncoderMeas>& encoderMeas,
                                                 Eigen::Vector2d& odom_parameter, const Sophus::SE2d T_OC) {

    OdomIntergrator odomIntergrator;
    Eigen::Vector3d pose(0,0,0);
    std::vector<SimulateMeasurement> poseMeasVec;
    SimulateMeasurement poseMeas;
    poseMeas.ts = encoderMeas.front().ts;

    Sophus::SE2d T_WO(pose[2], pose.head(2));
    Sophus::SE2d T_WC = T_WO*T_OC;
    poseMeas.T_WO = T_WO;
    poseMeas.T_WC = T_WC;
    poseMeasVec.push_back(poseMeas);

    std::vector<EncoderMeas>::const_iterator itr = encoderMeas.begin()+1;
    for (; itr != encoderMeas.end(); itr++) {
        Eigen::Vector2d delta_ticks = itr->meas - (itr-1)->meas;
        pose = odomIntergrator.computeOdom(delta_ticks,odom_parameter,pose);

        Sophus::SE2d T_WO(pose[2], pose.head(2));
        Sophus::SE2d T_WC = T_WO*T_OC;
        poseMeas.T_WO = T_WO;
        poseMeas.T_WC = T_WC;
        poseMeasVec.push_back(poseMeas);
    }

    return poseMeasVec;
}
int main() {

    /**
     *  Simulate data
     */
    std::string dataset_path =
            "/home/pang/maplab_ws/src/maplab/algorithms/odom_extrinsic_calibrate/calibrate_dataset/evt_3_2/calibrate_1";
    std::string encoder_meas_file = dataset_path + "/encoder_meas.txt";

    std::vector<EncoderMeas> encoderMeasVec = loadEncoder(encoder_meas_file);
    std::cout<< "encoder size: " << encoderMeasVec.size() << std::endl;

    Eigen::Vector2d odom_parameter(0.1,0.1);
    Sophus::SE2d extrinsic_parameter( 0.01, Eigen::Vector2d(0.3, 0.00));

    std::vector<SimulateMeasurement> poseMeasVector = simulateVioPose(encoderMeasVec,
            odom_parameter,extrinsic_parameter);
    std::cout<< "simulates pose: " << poseMeasVector.size() << std::endl;


    /**
     *  Test
     */


    ConfigParams params;
    std::vector<EncoderMeas>::const_iterator encoderItr = encoderMeasVec.begin();
    std::vector<SimulateMeasurement>::const_iterator poseItr = poseMeasVector.begin();


    Eigen::Vector2d noised_odom_parameter(0.123, 0.0941);
    Eigen::Vector2d init_odom_parameter = noised_odom_parameter;
    Sophus::SE2d delta(0.0011, Eigen::Vector2d(0.031, 0.1));
    Sophus::SE2d noised_extrinsic_parameter = extrinsic_parameter * delta;
    Sophus::SE2d init_extrinsic_parameter = noised_extrinsic_parameter;


    // Build the problem.
    ceres::Problem problem;

    test::LocalParameterizationSE2* poseLocalParameterization = new test::LocalParameterizationSE2();
    // Specify local update rule for our parameter
    problem.AddParameterBlock(noised_odom_parameter.data(), 2);
    problem.AddParameterBlock(noised_extrinsic_parameter.data(), 4, poseLocalParameterization);


    for (; encoderItr+1 != encoderMeasVec.end() && poseItr+1 != poseMeasVector.end();
           encoderItr ++ , poseItr ++) {

        MeasPackage firstMeasPackage,secondMeasPackage;
        firstMeasPackage.ts = encoderItr->ts;
        firstMeasPackage.ticks = encoderItr->meas;
        Sophus::SE2d T_WC0 = poseItr->T_WC;


        secondMeasPackage.ts = (encoderItr + 1)->ts;
        secondMeasPackage.ticks =  (encoderItr + 1)->meas;
        Sophus::SE2d T_WC1 = (poseItr + 1)->T_WC;

        ceres::CostFunction* costFunction =
                new OdomExtrinsicCalibrateError(T_WC0, T_WC1,
                        firstMeasPackage.ticks, secondMeasPackage.ticks);

        problem.AddResidualBlock(costFunction, NULL,
                noised_odom_parameter.data(), noised_extrinsic_parameter.data());

    }


    // Set solver options (precision / method)
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = 100;
    options.minimizer_progress_to_stdout = true;

    // Solve
    ceres::Solver::Summary summary;
    Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << std::endl;

    std::cout<< "Init odom parameter              " << init_odom_parameter.transpose() << std::endl;
    std::cout<< "Noised odom parameter after Opt. " << noised_odom_parameter.transpose() << std::endl;
    std::cout<< "True odom parameter              " << odom_parameter.transpose() << std::endl;

    std::cout<< "Init extrinsic parameter              "
             << init_extrinsic_parameter.params().transpose() << std::endl;
    std::cout<< "Noised extrinsic parameter after Opt. "
             << noised_extrinsic_parameter.params().transpose() << std::endl;
    std::cout<< "true extrinsic parameter after Opt.   "
                << extrinsic_parameter.params().transpose() << std::endl;

    return 0;
}