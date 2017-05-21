#ifndef WAVE_FAST_DETECTOR_HPP
#define WAVE_FAST_DETECTOR_HPP

#include "wave/vision/feature_detector.hpp"

namespace wave {

class FASTDetector : public FeatureDetector<Image> {
 public:
    explicit FASTDetector(const std::string&);
    ~FASTDetector();
    void configureDetector(int  new_threshold,
                           bool new_nonmax_suppression,
                           int  new_type);
    void getConfiguration(int&  current_threshold,
                          bool& current_nonmax_suppression,
                          int&  current_type);
    std::vector<Keypoint>& detectFeatures(Image& image_to_detect);

 private:
    // Member Variables
    // -------------------------------------------------------------------------

    // FASTDetector-specific Configuration Parameters
    int threshold;
    bool nonmax_suppression;
    int type;

    // OpenCV Variables
    // -------------------------------------------------------------------------
    static cv::Ptr<cv::FastFeatureDetector> fast_detector;
    Image image;

    // Private Functions
    // -------------------------------------------------------------------------
    void checkConfiguration(int new_threshold, int new_type);
};

} // end of namespace wave



#endif //WAVE_FAST_DETECTOR_HPP
