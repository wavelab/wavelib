PROJECT(wave_matching)

WAVE_ADD_MODULE(${PROJECT_NAME}
    DEPENDS
    wave::utils
    Eigen3::Eigen
    Boost::boost
    ${PCL_LIBRARIES}
    SOURCES
    src/gicp.cpp
    src/icp.cpp
    src/icp_pcl_functions.cpp
    src/ndt.cpp
    src/ground_segmentation.cpp
    src/pointcloud_display.cpp)

# Unit tests
IF(BUILD_TESTING)
    WAVE_ADD_TEST(${PROJECT_NAME}_tests
        tests/icp_tests.cpp
        tests/ndt_tests.cpp
        tests/gicp_tests.cpp
        tests/multi_matcher_tests.cpp)

WAVE_ADD_TEST(
    ${PROJECT_NAME}_viz_tests
    tests/pointcloud_display_tests.cpp
    DISABLED  # probably requires display to work properly
)

WAVE_ADD_TEST(${PROJECT_NAME}_ground_segmentation_tests
    tests/ground_segmentation_test.cpp
    DISABLED  # Uses viewer to check results
)

TARGET_LINK_LIBRARIES(
    ${PROJECT_NAME}_tests
    ${PROJECT_NAME}
    wave_utils
)

TARGET_LINK_LIBRARIES(
    ${PROJECT_NAME}_viz_tests
    ${PROJECT_NAME}
    wave_utils
)

TARGET_LINK_LIBRARIES(
    ${PROJECT_NAME}_ground_segmentation_tests
    ${PROJECT_NAME}
    wave_utils
)

    TARGET_LINK_LIBRARIES(${PROJECT_NAME}_viz_tests ${PROJECT_NAME})

    # Copy the test data
    file(COPY tests/data tests/config DESTINATION ${PROJECT_BINARY_DIR}/tests)
ENDIF(BUILD_TESTING)
