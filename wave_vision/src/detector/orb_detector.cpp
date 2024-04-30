#include "wave/vision/detector/orb_detector.hpp"

namespace wave {

// Filesystem constructor for ORBDetectorParams struct
ORBDetectorParams::ORBDetectorParams(const std::string &config_path) {
    // Extract parameters from .yaml file.
    ConfigParser parser;

    int num_features;
    float scale_factor;
    int num_levels;
    int edge_threshold;
    int score_type;
    int fast_threshold;

    // Add parameters to parser, to be loaded. If path cannot be found,
    // throw an exception.
    parser.addParam("num_features", &num_features);
    parser.addParam("scale_factor", &scale_factor);
    parser.addParam("num_levels", &num_levels);
    parser.addParam("edge_threshold", &edge_threshold);
    parser.addParam("score_type", &score_type);
    parser.addParam("fast_threshold", &fast_threshold);

    if (parser.load(config_path) != ConfigStatus::OK) {
        throw std::invalid_argument(
          "Failed to Load ORBDetectorParams Configuration");
    }

    this->num_features = num_features;
    this->scale_factor = scale_factor;
    this->num_levels = num_levels;
    this->edge_threshold = edge_threshold;
    if (score_type == 0) {
        this->score_type = cv::ORB::ScoreType::HARRIS_SCORE;
    } else if (score_type == 1) {
        this->score_type = cv::ORB::ScoreType::FAST_SCORE;
    }
    this->fast_threshold = fast_threshold;
}

// Default Constructor
ORBDetector::ORBDetector(const ORBDetectorParams &config) {
    // Ensure parameters are valid
    this->checkConfiguration(config);

    // Default parameters that should not be modified, and are not associated
    // with the detector. These values are the defaults recommended by OpenCV.
    //-------------------------------------------------------------------------
    int first_level = 0;  // As per OpenCV docs, first_level must be zero.
    int tuple_size = 2;
    int patch_size = 31;

    // Create ORB with these parameters
    this->orb_detector = cv::ORB::create(config.num_features,
                                         config.scale_factor,
                                         config.num_levels,
                                         config.edge_threshold,
                                         first_level,
                                         tuple_size,
                                         config.score_type,
                                         patch_size,
                                         config.fast_threshold);
}

void ORBDetector::checkConfiguration(const ORBDetectorParams &check_config) {
    // Check parameters. If invalid, throw an exception.
    if (check_config.num_features < 0) {
        throw std::invalid_argument(
          "num_features must be greater than/equal to 0");
    } else if (check_config.scale_factor < 1.0) {
        throw std::invalid_argument(
          "scale_factor must be greater than/equal to 1.0!");
    } else if (check_config.num_levels <= 0) {
        throw std::invalid_argument("num_levels must be greater than 0");
    } else if (check_config.edge_threshold < 0) {
        throw std::invalid_argument(
          "edge_threshold must be greater than/equal to 0");
    } else if (check_config.score_type < cv::ORB::HARRIS_SCORE ||
               check_config.score_type > cv::ORB::FAST_SCORE) {
        throw std::invalid_argument("Invalid score_type for ORBDetector!");
    } else if (check_config.fast_threshold <= 0) {
        throw std::invalid_argument("fast_threshold must be greater than 0");
    }
}

void ORBDetector::configure(const ORBDetectorParams &new_config) {
    // Confirm configuration parameters are valid
    this->checkConfiguration(new_config);

    // Set configuration values in detector.
    this->orb_detector->setMaxFeatures(new_config.num_features);
    this->orb_detector->setScaleFactor((double) new_config.scale_factor);
    this->orb_detector->setNLevels(new_config.num_levels);
    this->orb_detector->setEdgeThreshold(new_config.edge_threshold);
    this->orb_detector->setScoreType(new_config.score_type);
    this->orb_detector->setFastThreshold(new_config.fast_threshold);
}

ORBDetectorParams ORBDetector::getConfiguration() const {
    // Obtain current configuration values using cv::ORB::get**
    auto num_features = this->orb_detector->getMaxFeatures();
    auto scale_factor = (float) this->orb_detector->getScaleFactor();
    auto num_levels = this->orb_detector->getNLevels();
    auto edge_threshold = this->orb_detector->getEdgeThreshold();
    auto score_type = this->orb_detector->getScoreType();
    auto fast_threshold = this->orb_detector->getFastThreshold();

    ORBDetectorParams current_config{num_features,
                                     scale_factor,
                                     num_levels,
                                     edge_threshold,
                                     score_type,
                                     fast_threshold};

    return current_config;
}

std::vector<cv::KeyPoint> ORBDetector::detectFeatures(const cv::Mat &image) {
    std::vector<cv::KeyPoint> keypoints;

    // Detect features in image and return keypoints.
    this->orb_detector->detect(image, keypoints);

    // Store num detected keypoints for diagnostics
    this->num_keypoints_detected = keypoints.size();

    return keypoints;
}
}  // namespace wave
