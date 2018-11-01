#include <iostream>
#include <ceres/ceres.h>
#include <odom_extrinsic_calibrate/OdomCalibrateError.h>
#include <odom_extrinsic_calibrate/NumbDifferentiator.hpp>
#include <odom_extrinsic_calibrate/calibrate_process.h>
#include <odom_extrinsic_calibrate/odom_calibrate_solver.h>
#include <odom_extrinsic_calibrate/utils.h>
#include <odom_extrinsic_calibrate/odom_integrator.h>
using namespace   odom_calib;
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


std::vector<PoseMeas> simulateVioPose(const std::vector<EncoderMeas>& encoderMeas,
        Eigen::Vector2d& odom_parameter) {

    std::ofstream encoder_integrate_odom("/home/pang/encoder_integrate_odom.txt");
    std::ofstream simulated_odom("/home/pang/simulated_odom.txt");

    OdomIntergrator odomIntergrator;
    Eigen::Vector3d pose(0,0,0);
    std::vector<PoseMeas> poseMeasVec;
    PoseMeas poseMeas;
    poseMeas.ts = encoderMeas.front().ts;
    poseMeas.meas = pose;
    poseMeasVec.push_back(poseMeas);

    std::vector<EncoderMeas>::const_iterator itr = encoderMeas.begin()+1;
    for (; itr != encoderMeas.end(); itr++) {
        Eigen::Vector2d delta_ticks = itr->meas - (itr-1)->meas;
        pose = odomIntergrator.computeOdom(delta_ticks,odom_parameter,pose);

        // apply noise
        Eigen::Vector3d noise = Eigen::Vector3d::Random();
        noise << 0.01* noise[0], 0.01*noise[1], 0.001*noise(2);
        //std::cout<<"noise: " << noise.transpose() << std::endl;
        pose = pose+noise;
        PoseMeas poseMeas;
        poseMeas.ts = itr->ts;
        poseMeas.meas = pose;
        poseMeasVec.push_back(poseMeas);

        encoder_integrate_odom << pose[0]
            << " " << pose[1] << " " << pose[2] << std::endl;
    }
    encoder_integrate_odom.close();
    simulated_odom.close();
    return poseMeasVec;
}

int main() {

    /**
     *  Simulate data
     */
    std::string dataset_path =
            "/home/pang/maplab_ws/src/maplab/algorithms/odom_extrinsic_calibrate/calibrate_dataset/evt_3_2/calibrate_2";
    std::string encoder_meas_file = dataset_path + "/encoder_meas.txt";

    std::vector<EncoderMeas> encoderMeasVec = loadEncoder(encoder_meas_file);
    std::cout<< "encoder size: " << encoderMeasVec.size() << std::endl;

    Eigen::Vector2d odom_parameter(0.1,0.1);

    std::vector<PoseMeas> poseMeasVector = simulateVioPose(encoderMeasVec,odom_parameter);
    std::cout<< "simulates pose: " << poseMeasVector.size() << std::endl;


    /**
     *  Test
     */


    ConfigParams params;
    Eigen::Isometry3d T_OC = Eigen::Isometry3d::Identity();
    OdomCalibrateSolver odomCalibrateSolver(params, T_OC);

    std::vector<EncoderMeas>::const_iterator encoderItr = encoderMeasVec.begin();
    std::vector<PoseMeas>::const_iterator poseItr = poseMeasVector.begin();

    bool is_first_message = true;
    MeasPackage last_meas_package;
    int cnt = 0;

    Eigen::Vector2d noised_parameter(0.123, 0.0941);


    // Build the problem.
    ceres::Problem problem;

    // Specify local update rule for our parameter
    problem.AddParameterBlock(noised_parameter.data(), 2);


    for (; encoderItr+1 != encoderMeasVec.end() && poseItr+1 != poseMeasVector.end();
           encoderItr ++ , poseItr ++) {

        MeasPackage firstMeasPackage,secondMeasPackage;
        firstMeasPackage.ts = encoderItr->ts;
        firstMeasPackage.ticks = encoderItr->meas;
        firstMeasPackage.T_WC = poseItr->meas;


        secondMeasPackage.ts = (encoderItr + 1)->ts;
        secondMeasPackage.ticks =  (encoderItr + 1)->meas;
        secondMeasPackage.T_WC = (poseItr + 1)->meas;

        Sophus::SE2d T_WC0 = vector2SE2( firstMeasPackage.T_WC);
        Sophus::SE2d T_WC1 = vector2SE2( secondMeasPackage.T_WC);

        Sophus::SE2d  T_OC;
        ceres::CostFunction* costFunction =
                new OdomCalibrateError2(T_WC0, T_WC1,
                        firstMeasPackage.ticks, secondMeasPackage.ticks,
                                        T_OC);

        problem.AddResidualBlock(costFunction, NULL, noised_parameter.data());

    }


    // Set solver options (precision / method)
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = 10;
    options.minimizer_progress_to_stdout = true;

    // Solve
    ceres::Solver::Summary summary;
    Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << std::endl;

    std::cout<< "Noised parameter after Opt. " << noised_parameter.transpose() << std::endl;

    return 0;
}