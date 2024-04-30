/**
 * @file
 * FAST Feature Detector implementation, derived from Feature Detector base
 * class.
 * @ingroup vision
 */
#ifndef WAVE_VISION_FAST_DETECTOR_HPP
#define WAVE_VISION_FAST_DETECTOR_HPP

#include <string>
#include <vector>

#include "wave/vision/detector/feature_detector.hpp"

namespace wave {
/** @addtogroup vision
 *  @{ */

/** Configuration parameters for the FASTDetector. */
struct FASTDetectorParams {
    FASTDetectorParams() = default;

    FASTDetectorParams(const int threshold,
                       const bool nonmax_suppression,
                       const cv::FastFeatureDetector::DetectorType type,
                       const int num_features)
        : threshold(threshold),
          nonmax_suppression(nonmax_suppression),
          type(type),
          num_features(num_features) {}

    /** Constructor using parameters extracted from a configuration file.
     *
     *  @param config_path the path to the location of the configuration file.
     */
    explicit FASTDetectorParams(const std::string &config_path);

    /** Threshold on difference between intensity of the central pixel, and
     *  pixels in a circle (Bresenham radius 3) around this pixel.
     *
     *  Recommended: 10. Must be greater than zero.
     */
    int threshold = 10;

    /** Removes keypoints in adjacent locations.
     *
     *  Recommended: true
     */
    bool nonmax_suppression = true;

    /** Neighbourhood, as defined in the paper by Rosten. TYPE_N_M refers to
     *  the pixel circumference (M) and number of consecutive pixels (N) that
     *  must be brighter or darker than the center pixel for the algorithm to
     *  deem the point as a corner.
     *
     *  Options:
     *  ccv::FastFeatureDetector::DetectorType::TYPE_5_8
     *  ccv::FastFeatureDetector::DetectorType::TYPE_7_12
     *  ccv::FastFeatureDetector::DetectorType::TYPE_9_16 (recommended)
     */
    cv::FastFeatureDetector::DetectorType type = cv::FastFeatureDetector::DetectorType::TYPE_9_16;

    /** The number of features to keep from detection.
     *
     *  If set to 0, the detector will keep all features detected. Else, it will
     *  retain the best keypoints up to the number specified.
     *
     *  Default: 0.
     */
    int num_features = 0;
};

/** Representation of a feature detector using the FAST algorithm.
 *
 *  Internally, this class is wrapping OpenCV's FastFeatureDetector module.
 *  Further reference on the FastFeatureDetector can be found
 *  [here][opencv_fast_detector].
 *
 *  [opencv_fast_detector]:
 *  http://docs.opencv.org/trunk/df/d74/classcv_1_1FastFeatureDetector.html
 */
class FASTDetector : public FeatureDetector {
 public:
    /** Default constructor. The user can also specify their own struct with
     *  desired values. If no struct is provided, default values are used.
     *
     *  @param config contains the desired parameter values.
     */
    explicit FASTDetector(
      const FASTDetectorParams &config = FASTDetectorParams{});

    /** Reconfigures the cv::FastFeatureDetector object with new values
     *  requested by the user.
     *
     * @param new_config containing the desired configuration values.
     */
    void configure(const FASTDetectorParams &new_config);

    /** Returns the current configuration parameters being used by the
     *  FASTDetector.
     *
     * @return a struct containing the current configuration values.
     */
    FASTDetectorParams getConfiguration() const;

    /** Detects features in an image.
     *
     *  @param image the image to detect features in.
     *  @return a vector containing all of the keypoints found within the image.
     */
    std::vector<cv::KeyPoint> detectFeatures(const cv::Mat &image);

 private:
    /** The pointer to the wrapped cv::FastFeatureDetector object. */
    cv::Ptr<cv::FastFeatureDetector> fast_detector;

    /** The number of features to retain. This is a custom parameter for the
     *  FASTDetector, and it cannot be returned from the
     *  cv::FastFeatureDetector::get** functions.
     */
    int num_features;

    /** Checks whether the desired configuration is valid.
     *
     *  The threshold value must be greater than zero, while the type must be
     *  0, 1, or 2.
     *
     *  @param check_config contains the desired configuration values.
     */
    void checkConfiguration(const FASTDetectorParams &check_config);
};

/** @} group vision */
}  // namespace wave

#endif  // WAVE_VISION_FAST_DETECTOR_HPP
