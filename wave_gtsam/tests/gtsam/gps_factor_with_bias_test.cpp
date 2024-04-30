#include <random>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include "wave/gtsam/gps_factor_with_bias.hpp"
#include "wave/gtsam/gps_factor_with_bias_general.hpp"
#include "wave/gtsam/pose_vel_bias.hpp"
#include "wave/wave_test.hpp"

namespace wave {

TEST(gps_with_bias, init) {
    gtsam::Key O_S1 = 3;
    gtsam::Key BiasKey = 4;
    gtsam::Pose3 T_local_s1;
    Mat6 info;
    info.setIdentity();
    auto model = gtsam::noiseModel::Gaussian::Information(info);
    GPSFactorWBias factor(O_S1, BiasKey, T_local_s1, model);
}

TEST(custom_state, init) {
    PoseVelBias dummy;
    gtsam::Key key;
    key = 1;
    Mat6 info;
    info.setIdentity();
    auto model = gtsam::noiseModel::Gaussian::Information(info);

    auto factor = GPSFactorWithBiasGeneral<PoseVelBias>(key, dummy.pose, model);
}

// Test that the factor produces zero error when transforms form a chain
TEST(gps_with_bias, zero_error) {
    gtsam::Key O_S1 = 3;
    gtsam::Key BiasKey = 4;
    Eigen::Affine3d T_local_s1;
    gtsam::Point3 B_Z;

    T_local_s1.matrix() << 0.936293363584199, -0.275095847318244,
      0.218350663146334, 32.000000000000000, 0.289629477625516,
      0.956425085849232, -0.036957013524625, 2.000000000000000,
      -0.198669330795061, 0.097843395007256, 0.975170327201816,
      3.000000000000000, 0, 0, 0, 1.000000000000000;

    gtsam::Pose3 T_loc_1(T_local_s1.matrix());

    Mat6 info;
    info.setIdentity();
    auto model = gtsam::noiseModel::Gaussian::Information(info);
    GPSFactorWBias factor(O_S1, BiasKey, T_loc_1, model);

    auto err = factor.evaluateError(T_loc_1, B_Z);

    EXPECT_NEAR(err.norm(), 0, 1e-6);
}

// Test that the factor jacobians are good when error is small
TEST(gps_with_bias, jacobians) {
    gtsam::Key O_S1 = 3;
    gtsam::Key BiasKey = 4;
    Eigen::Affine3d T_local_s1;
    gtsam::Point3 B_Z;

    T_local_s1.matrix() << 0.936293363584199, -0.275095847318244,
      0.218350663146334, 32.000000000000000, 0.289629477625516,
      0.956425085849232, -0.036957013524625, 2.000000000000000,
      -0.198669330795061, 0.097843395007256, 0.975170327201816,
      3.000000000000000, 0, 0, 0, 1.000000000000000;

    gtsam::Pose3 T_loc_1(T_local_s1.matrix());

    Mat6 info;
    info.setIdentity();
    auto model = gtsam::noiseModel::Gaussian::Information(info);
    GPSFactorWBias factor(O_S1, BiasKey, T_loc_1, model);

    gtsam::Matrix J_loc1, J_B_Z;
    auto err = factor.evaluateError(T_loc_1, B_Z, J_loc1, J_B_Z);

    auto fun = boost::bind(&GPSFactorWBias::evaluateError,
                           boost::ref(factor),
                           boost::placeholders::_1,
                           boost::placeholders::_2,
                           boost::none,
                           boost::none);

    gtsam::Matrix J_loc1num =
      gtsam::numericalDerivative21<gtsam::Vector, gtsam::Pose3, gtsam::Point3>(
        fun, T_loc_1, B_Z, 1e-6);
    gtsam::Matrix J_BZnum =
      gtsam::numericalDerivative22<gtsam::Vector, gtsam::Pose3, gtsam::Point3>(
        fun, T_loc_1, B_Z, 1e-6);

    EXPECT_NEAR(err.norm(), 0, 1e-6);

    EXPECT_NEAR((J_loc1 - J_loc1num).norm(), 0, 1e-8);
    EXPECT_NEAR((J_B_Z - J_BZnum).norm(), 0, 1e-8);
}

}  // namespace wave
