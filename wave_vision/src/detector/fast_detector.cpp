#include "wave/vision/detector/fast_detector.hpp"

namespace wave {

// Filesystem constructor for FASTDetectorParams struct
FASTDetectorParams::FASTDetectorParams(const std::string &config_path) {
    // Extract parameters from .yaml file.
    ConfigParser parser;

    int threshold;
    bool nonmax_suppression;
    int type;
    int num_features;

    // Add parameters to parser, to be loaded. If path cannot be found,
    // throw an exception.
    parser.addParam("threshold", &threshold);
    parser.addParam("nonmax_suppression", &nonmax_suppression);
    parser.addParam("type", &type);
    parser.addParam("num_features", &num_features);

    if (parser.load(config_path) != ConfigStatus::OK) {
        throw std::invalid_argument(
          "Failed to Load FASTDetectorParams Configuration");
    }

    this->threshold = threshold;
    this->nonmax_suppression = nonmax_suppression;
    if (type == 0) {
        this->type = cv::FastFeatureDetector::DetectorType::TYPE_5_8;
    } else if (type == 1) {
        this->type = cv::FastFeatureDetector::DetectorType::TYPE_7_12;
    } else if (type == 2) {
        this->type = cv::FastFeatureDetector::DetectorType::TYPE_9_16;
    }
    this->num_features = num_features;
}

// Default Constructor
FASTDetector::FASTDetector(const FASTDetectorParams &config) {
    // Ensure parameters are valid
    this->checkConfiguration(config);

    // Create FastFeatureDetector with these parameters
    this->fast_detector = cv::FastFeatureDetector::create(
      config.threshold, config.nonmax_suppression, config.type);

    // Store num_features
    this->num_features = config.num_features;
}

void FASTDetector::checkConfiguration(const FASTDetectorParams &check_config) {
    // Check parameters. If invalid, throw an exception.
    if (check_config.threshold <= 0) {
        throw std::invalid_argument("threshold must be greater than 0!");
    } else if (check_config.type < 0 || check_config.type > 3) {
        throw std::invalid_argument("Invalid type for FASTDetector!");
    } else if (check_config.num_features < 0) {
        throw std::invalid_argument(
          "num_features must be greater than/equal to 0!");
    }
}

void FASTDetector::configure(const FASTDetectorParams &new_config) {
    // Confirm configuration parameters are valid
    this->checkConfiguration(new_config);

    // Set configuration values in detector.
    this->fast_detector->setThreshold(new_config.threshold);
    this->fast_detector->setNonmaxSuppression(new_config.nonmax_suppression);
    this->fast_detector->setType(new_config.type);
    this->num_features = new_config.num_features;
}

FASTDetectorParams FASTDetector::getConfiguration() const {
    // Obtain current configuration values using cv::FastFeatureDetector::get**
    auto threshold = this->fast_detector->getThreshold();
    auto nonmax_suppression = this->fast_detector->getNonmaxSuppression();
    auto type = this->fast_detector->getType();

    FASTDetectorParams current_config{
      threshold, nonmax_suppression, type, this->num_features};

    return current_config;
}

std::vector<cv::KeyPoint> FASTDetector::detectFeatures(const cv::Mat &image) {
    std::vector<cv::KeyPoint> keypoints;

    // Detect features in image and return keypoints.
    this->fast_detector->detect(image, keypoints);

    // Retain best keypoints, if specified.
    if (this->num_features != 0) {
        cv::KeyPointsFilter::retainBest(keypoints, this->num_features);
    }

    // Store num detected keypoints for diagnostics
    this->num_keypoints_detected = keypoints.size();

    return keypoints;
}
}  // namespace wave
