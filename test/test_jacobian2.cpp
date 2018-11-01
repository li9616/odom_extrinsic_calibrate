#include <iostream>
#include <sophus/se2.hpp>
#include <ceres/ceres.h>
#include <odom_extrinsic_calibrate/local_parameterization_se2.hpp>

// Eigen's ostream operator is not compatible with ceres::Jet types.
// In particular, Eigen assumes that the scalar type (here Jet<T,N>) can be
// casted to an arithmetic type, which is not true for ceres::Jet.
// Unfortunatly, the ceres::Jet class does not define a conversion
// operator (http://en.cppreference.com/w/cpp/language/cast_operator).
//
// This workaround creates a template specilization for Eigen's cast_impl,
// when casting from a ceres::Jet type. It relies on Eigen's internal API and
// might break with future versions of Eigen.
namespace Eigen {
    namespace internal {

        template <class T, int N, typename NewType>
        struct cast_impl<ceres::Jet<T, N>, NewType> {
        EIGEN_DEVICE_FUNC
        static inline NewType run(ceres::Jet<T, N> const& x) {
        return static_cast<NewType>(x.a);
    }
};

}  // namespace internal
}  // namespace Eigen


struct SE2DiffError{
    explicit SE2DiffError(const Sophus::SE2d target)
            : target_pose_(target){
    }

    template <class T>
    bool operator()(T const* const sT_wa, T* sResiduals) const {
        Eigen::Map<Sophus::SE2<T> const> const T_wa(sT_wa);
        Eigen::Map<Eigen::Matrix<T, 3, 1> > residuals(sResiduals);

        // We are able to mix Sophus types with doubles and Jet types withou needing
        // to cast to T.
        residuals = (target_pose_ * T_wa).log();
        // Reverse order of multiplication. This forces the compiler to verify that
        // (Jet, double) and (double, Jet) SE3 multiplication work correctly.
        residuals = (T_wa * target_pose_).log();
        // Finally, ensure that Jet-to-Jet multiplication works.
        residuals = (T_wa * target_pose_.cast<T>()).log();
        return true;
    }

private:

    Sophus::SE2d target_pose_;
};


bool test(Sophus::SE2d const& T_w_targ, Sophus::SE2d const& T_w_init) {


    // Optimisation parameters.
    Sophus::SE2d T_wr = T_w_init;

    // Build the problem.
    ceres::Problem problem;

    // Specify local update rule for our parameter
    problem.AddParameterBlock(T_wr.data(), Sophus::SE2d::num_parameters,
                              new Sophus::test::LocalParameterizationSE2);

    // Create and add cost functions. Derivatives will be evaluated via
    // automatic differentiation
    ceres::CostFunction* cost_function1 =
            new ceres::AutoDiffCostFunction<SE2DiffError, Sophus::SE2d::DoF,
                    Sophus::SE2d::num_parameters>(
                    new SE2DiffError(T_w_targ.inverse()));
    problem.AddResidualBlock(cost_function1, NULL, T_wr.data());


    // Set solver options (precision / method)
    ceres::Solver::Options options;
    options.gradient_tolerance = 0.01 * Sophus::Constants<double>::epsilon();
    options.function_tolerance = 0.01 * Sophus::Constants<double>::epsilon();
    options.linear_solver_type = ceres::DENSE_QR;

    // Solve
    ceres::Solver::Summary summary;
    Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << std::endl;

    // Difference between target and parameter
    double const mse = (T_w_targ.inverse() * T_wr).log().squaredNorm();
    bool const passed = mse < 10. * Sophus::Constants<double>::epsilon();
    return passed;
}



int main(int, char**) {
    using SE2Type = Sophus::SE2<double>;
    using SO2Type = Sophus::SO2<double>;
    using Point = SE2Type::Point;
    double const kPi = Sophus::Constants<double>::pi();

    std::vector<SE2Type> se2_vec;
    se2_vec.push_back(
            SE2Type(SO2Type::exp( 0.0), Point(0, 0)));
    se2_vec.push_back(
            SE2Type(SO2Type::exp( -1.0), Point(10, 0)));
    se2_vec.push_back(SE2Type(SO2Type::exp( 0.), Point(0, 100)));
    se2_vec.push_back(
            SE2Type(SO2Type::exp(0.00001), Point(0, 0)));
    se2_vec.push_back(SE2Type(SO2Type::exp( 0.00001),
                              Point(0, 0.0000000001)));
    se2_vec.push_back(
            SE2Type(SO2Type::exp(0.00001), Point(0.01, 0)));
    se2_vec.push_back(SE2Type(SO2Type::exp(kPi), Point(4, -5)));
    se2_vec.push_back(
            SE2Type(SO2Type::exp(0.2), Point(0, 0)) *
            SE2Type(SO2Type::exp(kPi), Point(0, 0)) *
            SE2Type(SO2Type::exp(-0.2), Point(0,  0)));
    se2_vec.push_back(
            SE2Type(SO2Type::exp(0.3), Point(2,  -7)) *
            SE2Type(SO2Type::exp(-0.3), Point(0, 6)));



    for (size_t i = 0; i < se2_vec.size(); ++i) {
        const int other_index = (i + 3) % se2_vec.size();
        bool const passed = test(se2_vec[i], se2_vec[other_index]);
        if (!passed) {
            std::cerr << "failed!" << std::endl << std::endl;
            exit(-1);
        }
    }

    return 0;
}