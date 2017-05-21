#ifndef WAVE_MATCHING_MATCHER_HPP
#define WAVE_MATCHING_MATCHER_HPP

#include <exception>

#include "wave/utils/utils.hpp"

namespace wave {

class ConfigException : public __exception {
    virtual const char *what() const throw() {
        return "Failed to Load Matcher Config";
    }
};

// sensor data type
template <typename T>
class Matcher {
 public:
    Matcher(float res) : resolution(res) {}
    Matcher() {
        resolution = -1;
    }

    virtual ~Matcher() {}

    const Eigen::Affine3d &getResult() {
        return this->result;
    };
    const Eigen::MatrixXd &getInfo() {
        return this->information;
    };
    float getRes() {
        return this->resolution;
    };

    virtual void setRef(const T &ref) = 0;
    virtual void setTarget(const T &target) = 0;
    void setup(const T &ref, const T &target) {
        this->setRef(ref);
        this->setTarget(target);
    };

    virtual bool match() {
        return 0;
    }

    virtual void estimate_info() {
        this->information = MatX::Identity(6,6);
    }

 protected:
    float resolution;
    Affine3 result;
    MatX information;
};

}  // end of wave namespace
#endif
