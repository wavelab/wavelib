#include <pcl/io/pcd_io.h>

#include "wave/wave_test.hpp"
#include "wave/matching/ndt.hpp"

namespace wave {

const auto TEST_SCAN = "tests/data/testscan.pcd";
const auto TEST_CONFIG = "tests/config/ndt.yaml";

// Fixture to load same pointcloud all the time
class NDTTest : public testing::Test {
 protected:
    NDTTest() {}

    virtual ~NDTTest() {
        if (this->matcher) {
            delete this->matcher;
        }
    }

    virtual void SetUp() {
        this->ref = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        this->target = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::io::loadPCDFile(TEST_SCAN, *(this->ref));
    }

    void initMatcher(const NDTMatcherParams params,
                     const Eigen::Affine3d perturb) {
        this->matcher = new NDTMatcher(params);
        pcl::transformPointCloud(*(this->ref), *(this->target), perturb);
        this->matcher->setup(this->ref, this->target);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr ref, target;
    NDTMatcher *matcher;
    const float threshold = 0.12;
};

TEST(NDTTests, initialization) {
    NDTMatcher matcher{NDTMatcherParams()};
}

// Zero displacement using resolution from config
TEST_F(NDTTest, fullResNullMatch) {
    Affine3 perturb;
    Affine3 result;
    bool match_success = false;

    // setup
    perturb = Affine3::Identity();
    perturb.translation() << 0, 0, 0;
    NDTMatcherParams params(TEST_CONFIG);
    this->initMatcher(params, perturb);

    // test and assert
    match_success = matcher->match();
    double diff = (matcher->getResult().matrix() - perturb.matrix()).norm();
    EXPECT_TRUE(match_success);
    EXPECT_LT(diff, this->threshold);
}

// Zero displacement using resolution set by constructor
TEST_F(NDTTest, nullDisplacement) {
    Affine3 perturb;
    Affine3 result;
    bool match_success = false;

    // setup
    perturb = Affine3::Identity();
    perturb.translation() << 0, 0, 0;

    NDTMatcherParams params(TEST_CONFIG);
    params.res = 0.1f;
    this->initMatcher(params, perturb);

    // test and assert
    match_success = matcher->match();
    double diff = (matcher->getResult().matrix() - perturb.matrix()).norm();
    EXPECT_TRUE(match_success);
    EXPECT_LT(diff, this->threshold);
}

// Small displacement using resolution set by constructor
TEST_F(NDTTest, smallDisplacement) {
    Affine3 perturb;
    Affine3 result;
    bool match_success = false;

    // setup
    perturb = Affine3::Identity();
    perturb.translation() << 0.2, 0, 0;
    NDTMatcherParams params(TEST_CONFIG);
    params.res = 0.3f;
    this->initMatcher(params, perturb);

    // test and assert
    match_success = matcher->match();
    double diff = (matcher->getResult().matrix() - perturb.matrix()).norm();
    EXPECT_TRUE(match_success);
    EXPECT_LT(diff, this->threshold);
}

}  // namespace wave
