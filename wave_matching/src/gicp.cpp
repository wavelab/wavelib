#include "wave/utils/config.hpp"
#include "wave/matching/gicp.hpp"

namespace wave {

GICPMatcherParams::GICPMatcherParams(const std::string &config_path) {
    ConfigParser parser;
    double r_eps = 1e-8, fit_eps = 1e-2;
    int corr_rand = 10, max_iter = 100;
    parser.addParam("corr_rand", &corr_rand);
    parser.addParam("max_iter", &max_iter);
    parser.addParam("r_eps", &r_eps);
    parser.addParam("fit_eps", &fit_eps);

    if (parser.load(config_path) != ConfigStatus::OK) {
        throw std::runtime_error{"Failed to Load Matcher Config"};
    }
}

GICPMatcher::GICPMatcher(GICPMatcherParams params1) : params(params1) {
    this->ref = std::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
    this->target = std::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
    this->final = std::make_shared<pcl::PointCloud<pcl::PointXYZ> >();

    if (params.res > 0) {
        this->resolution = params.res;
        this->filter.setLeafSize(params.res, params.res, params.res);
    } else {
        this->resolution = -1;
    }
    this->gicp.setCorrespondenceRandomness(this->params.corr_rand);
    this->gicp.setMaximumIterations(this->params.max_iter);
    this->gicp.setRotationEpsilon(this->params.r_eps);
    this->gicp.setEuclideanFitnessEpsilon(this->params.fit_eps);
}

void GICPMatcher::setRef(const PCLPointCloudPtr &ref) {
    if (this->resolution > 0) {
        this->filter.setInputCloud(ref);
        this->filter.filter(*(this->ref));
    } else {
        this->ref = ref;
    }
    this->gicp.setInputSource(this->ref);
}

void GICPMatcher::setTarget(const PCLPointCloudPtr &target) {
    if (resolution > 0) {
        this->filter.setInputCloud(target);
        this->filter.filter(*(this->target));
    } else {
        this->target = target;
    }
    this->gicp.setInputTarget(this->target);
}

bool GICPMatcher::match() {
    this->gicp.align(*(this->final));
    if (this->gicp.hasConverged()) {
        this->result.matrix() = gicp.getFinalTransformation().cast<double>();
        return true;
    }
    return false;
}

}  // namespace wave
