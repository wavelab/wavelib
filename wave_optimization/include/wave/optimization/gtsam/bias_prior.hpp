#ifndef WAVE_TWIST_PRIOR_HPP
#define WAVE_TWIST_PRIOR_HPP

#include <gtsam/nonlinear/NonlinearFactor.h>

namespace wave {

template<class T>
class BiasPrior : public gtsam::NoiseModelFactor1<T> {
 private:
    decltype(T.bias) prior;
 public:
    BiasPrior(gtsam::Key key, T m, const gtsam::SharedNoiseModel &model)
            : gtsam::NoiseModelFactor1<T>::NoiseModelFactor1(model, key), prior(m.bias) {}

    gtsam::Vector evaluateError(
            const T &m, boost::optional<gtsam::Matrix &> H = boost::none) const;
};

}

#include "wave/optimization/gtsam/impl/bias_prior_impl.hpp"

#endif //WAVE_TWIST_PRIOR_HPP
