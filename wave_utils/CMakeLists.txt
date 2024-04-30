PROJECT(wave_utils)

WAVE_ADD_MODULE(${PROJECT_NAME}
    DEPENDS
    Eigen3::Eigen
    yaml-cpp::yaml-cpp
    SOURCES
    src/config.cpp
    src/data.cpp
    src/file.cpp
    src/math.cpp
    src/time.cpp
    src/angles.cpp
    src/pose_cov_comp.cpp)

# Unit tests
IF(BUILD_TESTING)
    WAVE_ADD_TEST(${PROJECT_NAME}_tests
        tests/utils/config_test.cpp
        tests/utils/data_test.cpp
        tests/utils/file_test.cpp
        tests/utils/math_test.cpp
        tests/utils/time_test.cpp
        tests/utils/test_angles.cpp
        tests/utils/test_pose_cov_comp.cpp)

    TARGET_LINK_LIBRARIES(${PROJECT_NAME}_tests ${PROJECT_NAME})

    # COPY TEST DATA
    FILE(COPY tests/data DESTINATION ${PROJECT_BINARY_DIR}/tests)
ENDIF(BUILD_TESTING)
