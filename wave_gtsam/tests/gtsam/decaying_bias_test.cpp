#include <gtsam/base/numericalDerivative.h>
#include "wave/gtsam/decaying_bias.hpp"
#include "wave/wave_test.hpp"

namespace wave {

TEST(decaying_bias, init) {
    gtsam::Key B_1 = 0;
    gtsam::Key B_2 = 1;

    gtsam::Matrix1 info;
    info.setIdentity();
    auto model = gtsam::noiseModel::Gaussian::Information(info);

    DecayingBias factor(B_1, B_2, 0.1, 5, model);
}

TEST(decaying_bias, zero_error) {
    gtsam::Key B_1 = 0;
    gtsam::Key B_2 = 1;
    gtsam::Point3 B1val, B2val;
    B1val.matrix()(0) = 10;
    B1val.matrix()(1) = 10;
    B1val.matrix()(2) = 10;
    B2val.matrix()(0) = 10 * std::exp(-(0.1 / 5.0));
    B2val.matrix()(1) = 10 * std::exp(-(0.1 / 5.0));
    B2val.matrix()(2) = 10 * std::exp(-(0.1 / 5.0));

    gtsam::Matrix1 info;
    info.setIdentity();
    auto model = gtsam::noiseModel::Gaussian::Information(info);

    DecayingBias factor(B_1, B_2, 0.1, 5, model);

    auto err = factor.evaluateError(B1val, B2val);

    EXPECT_NEAR(err.norm(), 0, 1e-8);
}

TEST(decaying_bias, jacobians) {
    gtsam::Key B_1 = 0;
    gtsam::Key B_2 = 1;
    gtsam::Point3 B1val, B2val;
    B1val.matrix().setConstant(10.0);
    B2val.matrix().setConstant(10.0 * std::exp(-(0.1 / 5.0)));

    gtsam::Matrix1 info;
    info.setIdentity();
    auto model = gtsam::noiseModel::Gaussian::Information(info);

    gtsam::Matrix J_B_1, J_B_2;

    DecayingBias factor(B_1, B_2, 0.1, 5, model);

    auto err = factor.evaluateError(B1val, B2val, J_B_1, J_B_2);

    auto fun = boost::bind(&DecayingBias::evaluateError,
                           boost::ref(factor),
                           boost::placeholders::_1,
                           boost::placeholders::_2,
                           boost::none,
                           boost::none);

    gtsam::Matrix J_B_1num =
      gtsam::numericalDerivative21<gtsam::Vector, gtsam::Point3, gtsam::Point3>(
        fun, B1val, B2val, 1e-6);
    gtsam::Matrix J_B_2num =
      gtsam::numericalDerivative22<gtsam::Vector, gtsam::Point3, gtsam::Point3>(
        fun, B1val, B2val, 1e-6);

    EXPECT_NEAR(err.norm(), 0, 1e-8);
    EXPECT_NEAR((J_B_1 - J_B_1num).norm(), 0, 1e-8);
    EXPECT_NEAR((J_B_1 - J_B_1num).norm(), 0, 1e-8);
}

}  // namespace wave
