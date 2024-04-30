/**
 * @file
 * ORB Feature Detector implementation, derived from Feature Detector base
 * class.
 * @ingroup vision
 */
#ifndef WAVE_VISION_ORB_DETECTOR_HPP
#define WAVE_VISION_ORB_DETECTOR_HPP

#include <string>
#include <vector>

#include "wave/vision/detector/feature_detector.hpp"

namespace wave {
/** @addtogroup vision
 *  @{ */

/** Configuration parameters for the ORBDetector.
 *
 *  The full description of each parameter can be found
 *  [here][ORBDetectorParams]. Note: the ORBDetectorParams struct only consists
 *  of the parameters required for feature detection, not description.
 *  ORB Feature Description parameters can be found under ORBDescriptorParams.
 *
 *  [ORBDetectorParams]:
 *  http://docs.opencv.org/trunk/db/d95/classcv_1_1ORB.html#adc371099dc902a9674bd98936e79739c
 */
struct ORBDetectorParams {
    ORBDetectorParams() = default;

    ORBDetectorParams(const int num_features,
                      const float scale_factor,
                      const int num_levels,
                      const int edge_threshold,
                      const cv::ORB::ScoreType score_type,
                      const int fast_threshold)
        : num_features(num_features),
          scale_factor(scale_factor),
          num_levels(num_levels),
          edge_threshold(edge_threshold),
          score_type(score_type),
          fast_threshold(fast_threshold) {}

    /** Constructor using parameters extracted from a configuration file.
     *
     * @param config_path the path to the location of the configuration file.
     */
    ORBDetectorParams(const std::string &config_path);

    /** The number of features to keep from detection.
     *
     *  Default: 2000. Must be greater than zero.
     */
    int num_features = 2000;

    /** Pyramid scaling ratio. A value of 2 corresponds to the standard pyramid,
     *  where the number of pixels decrease by fourfold. A large value (ex. 2)
     *  can result in poor feature matching scores. A value closer to 1 will
     *  require more pyramid levels to extract truly scale-invariant features.
     *
     *  Default: 1.2f. Must be greater than 1.0f.
     */
    float scale_factor = 1.2f;

    /** The number of pyramid levels.
     *
     * Default: 8. Must be greater than zero.
     */
    int num_levels = 8;

    /** The border size where no features are detected. If using the
     * ORBDescriptor, this value should be approximately equal to the
     * patch_size.
     *
     * Default: 31. Must be greater than or equal to zero.
     */
    int edge_threshold = 31;

    /** Scoring method used to rank the keypoints. cv::ORB::HARRIS_SCORE uses
     *  the Harris algorithm to rank points, while cv::ORB::FAST_SCORE is an
     *  alternative option. cv::ORB::FAST_SCORE is slightly faster than
     *  cv::ORB::HARRIS_SCORE, but the results are not as stable.
     *
     *  Options: cv::ORB::ScoreType::HARRIS_SCORE, cv::ORB::ScoreType::FAST_SCORE
     *
     *  Default: cv::ORB::ScoreType::HARRIS_SCORE
     */
    cv::ORB::ScoreType score_type = cv::ORB::ScoreType::HARRIS_SCORE;

    /** Threshold on difference between intensity of the central pixel, and
     *  pixels in a circle (Bresenham radius 3) around this pixel.
     *
     *  ORB uses the FAST_9_16 implementation, which means that 9 consecutive
     *  pixels (out of a 16 pixel circumference circle) must be brighter or
     *  darker than the center pixel for the algorithm to deem the point as a
     *  corner.
     *
     *  Default: 20. Must be greater than zero.
     */
    int fast_threshold = 20;
};

/** Representation of a feature detector using the FAST algorithm.
 *
 *  Internally, this class is wrapping OpenCV's ORB module. Further reference
 *  on ORB can be found [here][opencv_orb_detector].
 *
 *  [opencv_orb_detector]:
 *  http://docs.opencv.org/trunk/db/d95/classcv_1_1ORB.html
 */
class ORBDetector : public FeatureDetector {
 public:
    /** Default constructor. The user can also specify their own struct with
     *  desired values. If no struct is provided, default values are used.
     *
     * @param config contains the desired parameter values.
     */
    explicit ORBDetector(const ORBDetectorParams &config = ORBDetectorParams{});

    /** Reconfigures the cv::ORB object with new values requested by the user.
     *
     * @param new_config containing the desired configuration values.
     */
    void configure(const ORBDetectorParams &new_config);

    /** Returns the current configuration parameters being used by the
     *  ORBDetector.
     *
     * @return a struct containing the current configuration values.
     */
    ORBDetectorParams getConfiguration() const;

    /** Detects features in an image,
     *
     * @param image the image to detect features in.
     * @return a vector containing all of the keypoints found within the image.
     */
    std::vector<cv::KeyPoint> detectFeatures(const cv::Mat &image);

 private:
    /** The pointer to the wrapped cv::ORB object. */
    cv::Ptr<cv::ORB> orb_detector;

    /** Checks whether the desired configuration is valid.
     *
     * @param check_config containing the desired configuration values.
     */
    void checkConfiguration(const ORBDetectorParams &check_config);
};

/** @} group vision */
}  // namespace wave

#endif  // WAVE_VISION_ORB_DETECTOR_HPP
