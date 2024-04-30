#include "wave/wave_test.hpp"
#include "wave/vision/detector/orb_detector.hpp"

namespace wave {

const auto TEST_CONFIG = "tests/config/detector/orb.yaml";

// Checks that the default configuration has no issues
TEST(ORBDetectorTests, GoodConfig) {
    // Default
    EXPECT_NO_THROW(ORBDetectorParams config1);

    // Custom params struct (with good values)
    int num_features = 500;
    float scale_factor = 1.2f;
    int num_levels = 8;
    int edge_threshold = 31;
    cv::ORB::ScoreType score_type = cv::ORB::HARRIS_SCORE;
    int fast_threshold = 20;

    EXPECT_NO_THROW(ORBDetectorParams config2(num_features,
                                              scale_factor,
                                              num_levels,
                                              edge_threshold,
                                              score_type,
                                              fast_threshold));

    // From orb.yaml file, with good values.
    EXPECT_NO_THROW(ORBDetectorParams config3(TEST_CONFIG));
}

// Checks that an incorrect configuration path throws an exception
TEST(ORBDetectorTests, BadConfigPath) {
    const std::string bad_path = "bad_path";

    ASSERT_THROW(ORBDetectorParams bad_config(bad_path), std::invalid_argument);
}

TEST(ORBDetectorTests, ConstructorTest) {
    ORBDetectorParams config;

    EXPECT_NO_THROW(ORBDetector detector);
    EXPECT_NO_THROW(ORBDetector detector1(config));
}

// Check that incorrect parameter values throw exceptions.
TEST(ORBDetectorTests, BadNumFeatures) {
    ORBDetectorParams config;
    config.num_features = -1;

    ASSERT_THROW(ORBDetector bad_nfeatures(config), std::invalid_argument);
}

TEST(ORBDetectorTests, BadScaleFactor) {
    ORBDetectorParams config;
    config.scale_factor = 0.99;

    ASSERT_THROW(ORBDetector bad_scalefactor1(config), std::invalid_argument);

    config.scale_factor = -1;
    ASSERT_THROW(ORBDetector bad_scalefactor2(config), std::invalid_argument);
}

TEST(ORBDetectorTests, BadNumLevels) {
    ORBDetectorParams config;
    config.num_levels = 0;

    ASSERT_THROW(ORBDetector bad_nlevels1(config), std::invalid_argument);

    config.num_levels = -1;
    ASSERT_THROW(ORBDetector bad_nlevels2(config), std::invalid_argument);
}

TEST(ORBDetectorTests, BadEdgeThreshold) {
    ORBDetectorParams config;

    config.edge_threshold = -1;

    ASSERT_THROW(ORBDetector bad_edgethreshold(config), std::invalid_argument);
}

TEST(ORBDetectorTests, BadFastThreshold) {
    ORBDetectorParams config;
    config.fast_threshold = 0;

    ASSERT_THROW(ORBDetector bad_fastthreshold(config), std::invalid_argument);

    config.fast_threshold = -1;
    ASSERT_THROW(ORBDetector bad_fastthreshold1(config), std::invalid_argument);
}

TEST(ORBDetectorTests, ConfigurationTests) {
    ORBDetector detector;

    ORBDetectorParams ref_config;
    ORBDetectorParams curr_config_1 = detector.getConfiguration();

    ASSERT_EQ(curr_config_1.num_features, ref_config.num_features);
    ASSERT_EQ(curr_config_1.scale_factor, ref_config.scale_factor);
    ASSERT_EQ(curr_config_1.num_levels, ref_config.num_levels);
    ASSERT_EQ(curr_config_1.edge_threshold, ref_config.edge_threshold);
    ASSERT_EQ(curr_config_1.score_type, ref_config.score_type);
    ASSERT_EQ(curr_config_1.fast_threshold, ref_config.fast_threshold);

    ORBDetectorParams new_config{700, 1.5f, 8, 31, cv::ORB::FAST_SCORE, 20};

    ASSERT_NO_THROW(detector.configure(new_config));

    detector.configure(new_config);
    ORBDetectorParams curr_config_2 = detector.getConfiguration();

    ASSERT_EQ(curr_config_2.num_features, new_config.num_features);
    ASSERT_EQ(curr_config_2.scale_factor, new_config.scale_factor);
    ASSERT_EQ(curr_config_2.num_levels, new_config.num_levels);
    ASSERT_EQ(curr_config_2.edge_threshold, new_config.edge_threshold);
    ASSERT_EQ(curr_config_2.score_type, new_config.score_type);
    ASSERT_EQ(curr_config_2.fast_threshold, new_config.fast_threshold);
}

TEST(ORBDetectorTests, BadNewConfiguration) {
    ORBDetector detector;
    ORBDetectorParams config;
    config.num_features = -1;

    ASSERT_THROW(detector.configure(config), std::invalid_argument);

    ORBDetectorParams config2;
    config2.scale_factor = 0.99;
    ASSERT_THROW(detector.configure(config2), std::invalid_argument);

    ORBDetectorParams config3;
    config3.num_levels = -1;
    ASSERT_THROW(detector.configure(config3), std::invalid_argument);

    ORBDetectorParams config4;
    config4.edge_threshold = -1;
    ASSERT_THROW(detector.configure(config4), std::invalid_argument);

    ORBDetectorParams config6;
    config6.fast_threshold = -1;
    ASSERT_THROW(detector.configure(config6), std::invalid_argument);
}
}  // namespace wave
