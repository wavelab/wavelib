#include <pcl/io/pcd_io.h>

#include "wave/wave_test.hpp"
#include "wave/matching/icp.hpp"
#include "wave/matching/multi_matcher.hpp"

namespace wave {

const auto TEST_SCAN = "tests/data/testscan.pcd";

class MultiTest : public testing::Test {
 protected:
    MultiTest() {}
    MultiTest(MultiTest&) = delete;
    MultiTest(MultiTest&&) = default;

    virtual ~MultiTest() {}

    virtual void SetUp() {
        this->cld = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::io::loadPCDFile(TEST_SCAN, *(this->cld));
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cld;
    MultiMatcher<ICPMatcher, ICPMatcherParams> matcher;
};

// Tests that threads are created and destroyed properly
TEST(MultiTests, initialization) {
    MultiMatcher<ICPMatcher, ICPMatcherParams> matcher;
}

TEST_F(MultiTest, simultaneousmatching) {
    // Setup
    pcl::PointCloud<pcl::PointXYZ>::Ptr dupes[9];
    for (int i = 0; i < 9; i++) {
        dupes[i] = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        *(dupes[i]) = *(this->cld);
    }
    for (int i = 0; i < 8; i++) {
        this->matcher.insert(i, dupes[i], dupes[i + 1]);
    }
    while (!this->matcher.done()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

}  // namespace wave
