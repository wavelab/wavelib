#include "wave/wave_test.hpp"
#include "wave/vision/detector/fast_detector.hpp"

namespace wave {

const auto TEST_CONFIG = "tests/config/detector/fast.yaml";

// Checks that the default configuration has no issues
TEST(FASTTests, GoodConfig) {
    // Default
    EXPECT_NO_THROW(FASTDetectorParams config1);

    // Custom params struct (with good values)
    int threshold = 10;
    bool nms = true;
    cv::FastFeatureDetector::DetectorType type = cv::FastFeatureDetector::DetectorType::TYPE_9_16;
    int num_features = 10;

    EXPECT_NO_THROW(
      FASTDetectorParams config2(threshold, nms, type, num_features));

    // From fast.yaml file, with good values.
    EXPECT_NO_THROW(FASTDetectorParams config3(TEST_CONFIG));
}

// Checks that incorrect configuration path throws an exception
TEST(FASTTests, BadInitialization) {
    const std::string bad_path = "bad_path";

    ASSERT_THROW(FASTDetectorParams bad_config(bad_path),
                 std::invalid_argument);
}

TEST(FASTTests, ConstructorTest) {
    FASTDetectorParams config;

    EXPECT_NO_THROW(FASTDetector detector);
    EXPECT_NO_THROW(FASTDetector detector1(config));
}

// Check that incorrect parameter values throw exceptions.
TEST(FASTTests, BadThresholdConfiguration) {
    FASTDetectorParams config;
    config.threshold = -1;

    ASSERT_THROW(FASTDetector bad_threshold(config), std::invalid_argument);
}

TEST(FASTTests, BadNumFeatures) {
    FASTDetectorParams config;
    config.num_features = -1;

    ASSERT_THROW(FASTDetector bad_nfeatures(config), std::invalid_argument);
}


TEST(FASTTests, ConfigurationTests) {
    FASTDetector detector;

    FASTDetectorParams ref_config;
    FASTDetectorParams curr_config_1 = detector.getConfiguration();

    ASSERT_EQ(curr_config_1.threshold, ref_config.threshold);
    ASSERT_EQ(curr_config_1.nonmax_suppression, ref_config.nonmax_suppression);
    ASSERT_EQ(curr_config_1.type, ref_config.type);
    ASSERT_EQ(curr_config_1.num_features, ref_config.num_features);

    FASTDetectorParams new_config{
      20, false, cv::FastFeatureDetector::TYPE_5_8, 3000};

    // Configure detector with valid values
    detector.configure(new_config);

    // Extract configuration from detector, assert values have been properly set
    FASTDetectorParams curr_config_2 = detector.getConfiguration();

    ASSERT_EQ(curr_config_2.threshold, new_config.threshold);
    ASSERT_EQ(curr_config_2.nonmax_suppression, new_config.nonmax_suppression);
    ASSERT_EQ(curr_config_2.type, new_config.type);
    ASSERT_EQ(curr_config_2.num_features, new_config.num_features);
}

TEST(FASTTests, BadNewConfiguration) {
    FASTDetector detector;
    FASTDetectorParams new_config{
      0, true, cv::FastFeatureDetector::TYPE_7_12, 0};

    ASSERT_THROW(detector.configure(new_config), std::invalid_argument);

    FASTDetectorParams new_config_3;
    new_config_3.num_features = -5;
    ASSERT_THROW(detector.configure(new_config_3), std::invalid_argument);
}
}  // namespace wave
