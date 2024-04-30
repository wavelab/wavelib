#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#include "wave/wave_test.hpp"
#include "wave/matching/pointcloud_display.hpp"
#include "wave/utils/log.hpp"
#include "wave/utils/math.hpp"

namespace wave {

const auto TEST_SCAN = "tests/data/testscan.pcd";

TEST(viewer, init_test) {
    PointCloudDisplay display("init_test");
    display.startSpin();
    std::this_thread::sleep_for(std::chrono::seconds(5));
    display.stopSpin();
}

TEST(viewer, pointcloud_test) {
    PointCloudDisplay display("pointcloud_test");
    display.startSpin();
    PCLPointCloudPtr cloud =
      std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::io::loadPCDFile(TEST_SCAN, *cloud);
    display.addPointcloud(cloud, 0, true);
    std::this_thread::sleep_for(std::chrono::seconds(5));
    display.stopSpin();
}

TEST(viewer, multiple_clouds_test) {
    PointCloudDisplay display("multiple_clouds_test");
    display.startSpin();
    PCLPointCloudPtr cloud =
      std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::io::loadPCDFile(TEST_SCAN, *cloud);
    display.addPointcloud(cloud, 0, true);
    Eigen::Affine3f transform(Eigen::Translation3f(Eigen::Vector3f(20, 0, 0)));
    for (int i = 0; i < 3; i++) {
        pcl::transformPointCloud(*cloud, *cloud, transform);
        display.addPointcloud(cloud, i + 1);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::this_thread::sleep_for(std::chrono::seconds(5));
    display.stopSpin();
}

TEST(viewer, line_test) {
    PointCloudDisplay display("line_test");
    display.startSpin();
    std::vector<pcl::PointXYZ> pts(10);
    for (int i = 0; i < 10; i++) {
        pts.at(i) = pcl::PointXYZ(cos(i), sin(i), 0.3 * i);
    }
    for (int j = 0; j < 9; j++) {
        display.addLine(pts.at(j), pts.at(j + 1), j, j + 1);
    }
    std::this_thread::sleep_for(std::chrono::seconds(5));
    display.stopSpin();
}

TEST(viewer, pointcloud_intensity) {
    PointCloudDisplay display("intensity_test");
    display.startSpin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud =
      std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl::io::loadPCDFile(TEST_SCAN, *cloud);
    display.addPointcloud(cloud, 1, true);
    std::this_thread::sleep_for(std::chrono::seconds(5));
    display.stopSpin();
}

}  // namespace wave
