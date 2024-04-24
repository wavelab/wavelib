PROJECT(wave_controls)

WAVE_ADD_MODULE(${PROJECT_NAME}
    DEPENDS
    Eigen3::Eigen
    wave::utils
    SOURCES
    src/pid.cpp)

# Unit tests
IF(BUILD_TESTING)
    WAVE_ADD_TEST(${PROJECT_NAME}_tests tests/pid_test.cpp)
    TARGET_LINK_LIBRARIES(${PROJECT_NAME}_tests ${PROJECT_NAME})
ENDIF(BUILD_TESTING)
