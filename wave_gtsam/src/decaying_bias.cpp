#include "wave/gtsam/decaying_bias.hpp"

namespace wave {

DecayingBias::DecayingBias(gtsam::Key B_1,
                           gtsam::Key B_2,
                           double dT,
                           double T,
                           const gtsam::SharedNoiseModel &model)
    : NoiseModelFactor2(model, B_1, B_2) {
    this->decay_factor = std::exp(-dT / T);
}

gtsam::Vector DecayingBias::evaluateError(
  const gtsam::Point3 &B_1,
  const gtsam::Point3 &B_2,
  boost::optional<gtsam::Matrix &> J_B_1,
  boost::optional<gtsam::Matrix &> J_B_2) const {
    if (J_B_1) {
        J_B_1->resize(B_1.NumDimensions, B_1.NumDimensions);
        (*J_B_1) = -this->decay_factor * gtsam::Matrix3::Identity();
    }
    if (J_B_2) {
        J_B_2->setIdentity(B_2.NumDimensions, B_2.NumDimensions);
    }

    gtsam::Vector retval;
    retval.resize(B_1.NumDimensions);
    retval = B_2 - B_1 * (this->decay_factor);
    return retval;
}
}