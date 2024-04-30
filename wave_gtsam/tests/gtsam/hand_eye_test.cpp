#include <random>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include "wave/gtsam/hand_eye.hpp"
#include "wave/wave_test.hpp"

namespace wave {

TEST(hand_eye, init) {
    gtsam::Key O_S2 = 3;
    gtsam::Key S1_S2 = 2;
    gtsam::Key BiasKey = 4;
    gtsam::Pose3 T_local_s1;
    Mat6 info;
    info.setIdentity();
    auto model = gtsam::noiseModel::Gaussian::Information(info);
    HandEyeFactor factor(O_S2, S1_S2, BiasKey, T_local_s1, model);
}

// Test that the factor produces zero error when transforms form a chain
TEST(hand_eye, zero_error) {
    gtsam::Key O_S2 = 3;
    gtsam::Key S1_S2 = 2;
    gtsam::Key BiasKey = 4;
    Eigen::Affine3d T_local_s1;
    Eigen::Affine3d T_local_s2;
    Eigen::Affine3d T_s1_s2;
    gtsam::Point3 B_Z;

    T_local_s1.matrix() << 0.936293363584199, -0.275095847318244,
      0.218350663146334, 32.000000000000000, 0.289629477625516,
      0.956425085849232, -0.036957013524625, 2.000000000000000,
      -0.198669330795061, 0.097843395007256, 0.975170327201816,
      3.000000000000000, 0, 0, 0, 1.000000000000000;

    T_s1_s2.matrix() << 1.000000000000000, 0, 0, 0.200000000000000, 0,
      0.995004165278026, -0.099833416646828, 0.300000000000000, 0,
      0.099833416646828, 0.995004165278026, -0.100000000000000, 0, 0, 0,
      1.000000000000000;

    T_local_s2.matrix() << 0.936293363584199, -0.251922821203341,
      0.244723577664956, 32.082894852206735, 0.289629477625516,
      0.947957399267153, -0.132255566480303, 2.348549122632335,
      -0.198669330795061, 0.194709171154326, 0.960530497001443,
      2.892102119622983, 0, 0, 0, 1.000000000000000;

    gtsam::Pose3 T_loc_1(T_local_s1.matrix());
    gtsam::Pose3 T_loc_2(T_local_s2.matrix());
    gtsam::Pose3 T_s1s2(T_s1_s2.matrix());

    Mat6 info;
    info.setIdentity();
    auto model = gtsam::noiseModel::Gaussian::Information(info);
    HandEyeFactor factor(O_S2, S1_S2, BiasKey, T_loc_1, model);

    auto err = factor.evaluateError(T_loc_2, T_s1s2, B_Z);

    EXPECT_NEAR(err.norm(), 0, 1e-6);
}

// Test that the factor jacobians are good when error is small
TEST(hand_eye, jacobians) {
    gtsam::Key O_S2 = 3;
    gtsam::Key S1_S2 = 2;
    gtsam::Key BiasKey = 4;
    Eigen::Affine3d T_local_s1;
    Eigen::Affine3d T_local_s2;
    Eigen::Affine3d T_s1_s2;
    gtsam::Point3 B_Z;

    T_local_s1.matrix() << 0.936293363584199, -0.275095847318244,
      0.218350663146334, 32.000000000000000, 0.289629477625516,
      0.956425085849232, -0.036957013524625, 2.000000000000000,
      -0.198669330795061, 0.097843395007256, 0.975170327201816,
      3.000000000000000, 0, 0, 0, 1.000000000000000;

    T_s1_s2.matrix() << 1.000000000000000, 0, 0, 0.200000000000000, 0,
      0.995004165278026, -0.099833416646828, 0.300000000000000, 0,
      0.099833416646828, 0.995004165278026, -0.100000000000000, 0, 0, 0,
      1.000000000000000;

    T_local_s2.matrix() << 0.936293363584199, -0.251922821203341,
      0.244723577664956, 32.082894852206735, 0.289629477625516,
      0.947957399267153, -0.132255566480303, 2.348549122632335,
      -0.198669330795061, 0.194709171154326, 0.960530497001443,
      2.892102119622983, 0, 0, 0, 1.000000000000000;

    gtsam::Pose3 T_loc_1(T_local_s1.matrix());
    gtsam::Pose3 T_loc_2(T_local_s2.matrix());
    gtsam::Pose3 T_s1s2(T_s1_s2.matrix());

    Mat6 info;
    info.setIdentity();
    auto model = gtsam::noiseModel::Gaussian::Information(info);
    HandEyeFactor factor(O_S2, S1_S2, BiasKey, T_loc_1, model);

    gtsam::Matrix J_loc2, J_s1s2, J_B_Z;
    auto err =
      factor.evaluateError(T_loc_2, T_s1s2, B_Z, J_loc2, J_s1s2, J_B_Z);

    auto fun = boost::bind(&HandEyeFactor::evaluateError,
                           boost::ref(factor),
                           boost::placeholders::_1,
                           boost::placeholders::_2,
                           boost::placeholders::_3,
                           boost::none,
                           boost::none,
                           boost::none);

    gtsam::Matrix J_loc2num = gtsam::numericalDerivative31<gtsam::Vector,
                                                           gtsam::Pose3,
                                                           gtsam::Pose3,
                                                           gtsam::Point3>(
      fun, T_loc_2, T_s1s2, B_Z, 1e-6);
    gtsam::Matrix J_s1s2num = gtsam::numericalDerivative32<gtsam::Vector,
                                                           gtsam::Pose3,
                                                           gtsam::Pose3,
                                                           gtsam::Point3>(
      fun, T_loc_2, T_s1s2, B_Z, 1e-6);
    gtsam::Matrix J_BZnum = gtsam::numericalDerivative33<gtsam::Vector,
                                                         gtsam::Pose3,
                                                         gtsam::Pose3,
                                                         gtsam::Point3>(
      fun, T_loc_2, T_s1s2, B_Z, 1e-6);

    EXPECT_NEAR(err.norm(), 0, 1e-6);

    EXPECT_NEAR((J_loc2 - J_loc2num).norm(), 0, 1e-8);
    EXPECT_NEAR((J_s1s2 - J_s1s2num).norm(), 0, 1e-8);
    EXPECT_NEAR((J_B_Z - J_BZnum).norm(), 0, 1e-8);
}

}  // namespace wave
