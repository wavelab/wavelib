#include <pcl/io/pcd_io.h>

#include "wave/wave_test.hpp"
#include "wave/matching/gicp.hpp"

namespace wave {

const auto TEST_SCAN = "tests/data/testscan.pcd";
const auto TEST_CONFIG = "tests/config/gicp.yaml";

// Fixture to load same pointcloud all the time
class GICPTest : public testing::Test {
 protected:
    GICPTest() {}

    virtual ~GICPTest() {
        if (this->matcher) {
            delete this->matcher;
        }
    }

    virtual void SetUp() {
        this->ref = std::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
        this->target = std::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
        pcl::io::loadPCDFile(TEST_SCAN, *(this->ref));
    }

    void setParams(const GICPMatcherParams params, const Affine3 perturb) {
        this->matcher = new GICPMatcher(params);
        pcl::transformPointCloud(*(this->ref), *(this->target), perturb);
        this->matcher->setup(this->ref, this->target);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr ref, target;
    GICPMatcher *matcher;
};

TEST(gicp_tests, initialization) {
    GICPMatcher matcher{GICPMatcherParams()};
}

// Zero displacement without downsampling
TEST_F(GICPTest, fullResNullMatch) {
    Affine3 perturb;
    Affine3 result;
    bool match_success = false;

    // setup
    perturb = Affine3::Identity();
    perturb.translation() << 0, 0, 0;
    GICPMatcherParams params(TEST_CONFIG);
    params.res = -1;
    this->setParams(params, perturb);

    // test and assert
    match_success = matcher->match();
    double diff = (matcher->getResult().matrix() - perturb.matrix()).norm();
    EXPECT_TRUE(match_success);
    EXPECT_LT(diff, 0.1);
}

// Zero displacement using voxel downsampling
TEST_F(GICPTest, nullDisplacement) {
    Affine3 perturb;
    Affine3 result;
    bool match_success = false;

    // setup
    perturb = Affine3::Identity();
    perturb.translation() << 0, 0, 0;
    GICPMatcherParams params(TEST_CONFIG);
    params.res = 0.05f;
    this->setParams(params, perturb);

    // test and assert
    match_success = matcher->match();
    double diff = (matcher->getResult().matrix() - perturb.matrix()).norm();
    EXPECT_TRUE(match_success);
    EXPECT_LT(diff, 0.1);
}

// Small displacement using voxel downsampling
TEST_F(GICPTest, smallDisplacement) {
    Affine3 perturb;
    Affine3 result;
    bool match_success = false;

    // setup
    perturb = Affine3::Identity();
    perturb.translation() << 0.2, 0, 0;
    GICPMatcherParams params(TEST_CONFIG);
    params.res = 0.05f;
    this->setParams(params, perturb);

    // test and assert
    match_success = matcher->match();
    double diff = (matcher->getResult().matrix() - perturb.matrix()).norm();
    EXPECT_TRUE(match_success);
    EXPECT_LT(diff, 0.1);
}

}  // namespace wave
