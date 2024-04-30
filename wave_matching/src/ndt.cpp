#include "wave/utils/utils.hpp"
#include "wave/matching/ndt.hpp"

namespace wave {

NDTMatcherParams::NDTMatcherParams(const std::string &config_path) {
    ConfigParser parser;
    parser.addParam("step_size", &this->step_size);
    parser.addParam("max_iter", &this->max_iter);
    parser.addParam("t_eps", &this->t_eps);
    parser.addParam("res", &this->res);

    if (parser.load(config_path) != ConfigStatus::OK) {
        throw std::runtime_error{"Failed to Load Matcher Config"};
    }
}

NDTMatcher::NDTMatcher(NDTMatcherParams params1) : params(params1) {
    this->ref = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    this->target = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    this->final = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    if (this->params.res < this->params.min_res) {
        LOG_ERROR("Invalid resolution given, using minimum");
        this->params.res = this->params.min_res;
    }

    this->resolution = this->params.res;

    this->ndt.setTransformationEpsilon(this->params.t_eps);
    this->ndt.setStepSize(this->params.step_size);
    this->ndt.setResolution(this->params.res);
    this->ndt.setMaximumIterations(this->params.max_iter);
}

NDTMatcher::~NDTMatcher() {
    if (this->ref) {
        this->ref.reset();
    }
    if (this->target) {
        this->target.reset();
    }
    if (this->final) {
        this->final.reset();
    }
}

void NDTMatcher::setRef(const PCLPointCloudPtr &ref) {
    this->ref = ref;
    this->ndt.setInputSource(this->ref);
}

void NDTMatcher::setTarget(const PCLPointCloudPtr &target) {
    this->target = target;
    this->ndt.setInputTarget(this->target);
}

bool NDTMatcher::match() {
    this->ndt.align(*(this->final));
    if (this->ndt.hasConverged()) {
        this->result.matrix() = ndt.getFinalTransformation().cast<double>();
        return true;
    }
    return false;
}

}  // namespace wave
