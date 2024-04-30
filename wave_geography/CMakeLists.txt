PROJECT(wave_geography)

WAVE_ADD_MODULE(${PROJECT_NAME}
    DEPENDS
    Eigen3::Eigen
    GeographicLib::GeographicLib
    SOURCES
    src/world_frame_conversions.cpp)

# Unit tests
IF(BUILD_TESTING)
    WAVE_ADD_TEST(${PROJECT_NAME}_tests
        tests/geography/test_ecef_llh_point_conversions.cpp
        tests/geography/test_ecef_enu_transforms.cpp
        tests/geography/test_enu_llh_point_conversions.cpp)

    TARGET_LINK_LIBRARIES(${PROJECT_NAME}_tests ${PROJECT_NAME})
ENDIF(BUILD_TESTING)
