cmake_minimum_required(VERSION 3.16)
PROJECT(wave)

# Package version, used when other projects FIND_PACKAGE(wave <version>)
SET(WAVE_PACKAGE_VERSION 0.1.1)

# Compiler settings for all targets
SET(CMAKE_CXX_STANDARD 17)
SET(CMAKE_POSITION_INDEPENDENT_CODE ON)
ADD_COMPILE_OPTIONS(-Wall -Wextra -Wno-strict-overflow)

# Default to Release build type, otherwise some modules will be slow
IF(NOT CMAKE_BUILD_TYPE)
   MESSAGE(STATUS "Using 'Release' build type as CMAKE_BUILD_TYPE is not set")
   # Set the value but keep it as an option in CMake GUI
   SET(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
   SET_PROPERTY(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
       "None" "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
ENDIF()

# CMake modules
LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
INCLUDE(CMakePackageConfigHelpers)
INCLUDE(cmake/WaveHelpers.cmake)
ENABLE_TESTING()

# User options
OPTION(BUILD_SHARED_LIBS "Build shared instead of static libraries" OFF)
OPTION(EXPORT_BUILD
 "Add this build directory to CMake's user package registry.\
 Allows the package to be found without install." OFF)
OPTION(BUILD_TESTING "Build tests" ON)
OPTION(BUILD_BENCHMARKS
    "Build benchmarks for some components. Requires google benchmark package."
    OFF)


# Find all dependencies here, and ensure they have IMPORTED targets
FIND_PACKAGE(Eigen3 3.4 REQUIRED)
FIND_PACKAGE(Boost 1.82 REQUIRED COMPONENTS system filesystem)
FIND_PACKAGE(PCL 1.13 CONFIG REQUIRED COMPONENTS
    common filters registration kdtree search io visualization)
FIND_PACKAGE(kindr)
FIND_PACKAGE(OpenCV 4.9)
FIND_PACKAGE(yaml-cpp 0.8 REQUIRED)
FIND_PACKAGE(Ceres 1.14)
FIND_PACKAGE(GTSAM 4.2)
FIND_PACKAGE(GeographicLib 2.3 REQUIRED CONFIG)

# Optionally build tests. `gtest` is included with this project
IF(BUILD_TESTING)
    # Build gtest from source
    # Unless another GTEST_ROOT is set by the user, tell the AddGTest script to
    # first look for the bundled copy in deps/googletest
    SET(GTEST_ROOT "${PROJECT_SOURCE_DIR}/deps/googletest"
        CACHE PATH "Path to gtest source")
    INCLUDE(cmake/AddGTest.cmake)

    # This target is used to build all tests, without running them
    ADD_CUSTOM_TARGET(tests)
    MESSAGE(STATUS "Building tests")
ENDIF(BUILD_TESTING)

# Optionally build benchmarks (requires `benchmark` package to be installed)
IF(BUILD_BENCHMARKS)
    FIND_PACKAGE(benchmark REQUIRED)

    STRING(TOLOWER "${CMAKE_BUILD_TYPE}" build_type)
    IF(NOT build_type MATCHES rel)
        MESSAGE(WARNING "Building benchmarks but not in Release mode."
            " (CMAKE_BUILD_TYPE=\"${CMAKE_BUILD_TYPE}\")\n"
            "Benchmark results might not be useful.")
    ENDIF()

    # This target is used to build (but not run) benchmarks
    ADD_CUSTOM_TARGET(benchmarks)

    # This target is used to run benchmarks via "make benchmark".
    # Like "make test", it does not build anything. (@todo change?)
    # It runs all tests labelled "benchmark" by WAVE_ADD_BENCHMARK helper.
    ADD_CUSTOM_TARGET(benchmark
         COMMAND ${CMAKE_CTEST_COMMAND} -C benchmark -L benchmark)
    MESSAGE(STATUS "Building benchmarks")
ENDIF(BUILD_BENCHMARKS)

# Add a special "wave" target including all modules
# The WAVE_ADD_LIBRARY helper in WaveUtils.cmake will add each module to this
ADD_LIBRARY(wave INTERFACE)
INSTALL(TARGETS wave EXPORT waveTargets)

# Add each module to the project
# Modules with missing dependencies are not built, and circular dependencies are
# not supported. Thus modules must be listed after their dependencies, for now.
ADD_SUBDIRECTORY(wave_utils)
ADD_SUBDIRECTORY(wave_geometry)
ADD_SUBDIRECTORY(wave_containers)
ADD_SUBDIRECTORY(wave_benchmark)
ADD_SUBDIRECTORY(wave_controls)
ADD_SUBDIRECTORY(wave_kinematics)
ADD_SUBDIRECTORY(wave_matching)
ADD_SUBDIRECTORY(wave_vision)
ADD_SUBDIRECTORY(wave_optimization)
ADD_SUBDIRECTORY(wave_gtsam)
ADD_SUBDIRECTORY(wave_geography)

# Documentation
SET(WAVE_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
SET(WAVE_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR})
ADD_SUBDIRECTORY(docs)

# This is where .cmake files will be installed (default under share/)
SET(INSTALL_CMAKE_DIR "${CMAKE_INSTALL_LIBDIR}/wave/cmake" CACHE PATH
    "Installation directory for CMake files")

# Add an export file listing all targets
EXPORT(EXPORT waveTargets NAMESPACE wave:: FILE waveTargets.cmake)
# Install the export file
INSTALL(EXPORT waveTargets NAMESPACE wave:: DESTINATION "${INSTALL_CMAKE_DIR}")

# Create the waveConfig and waveConfigVersion files

# Generate the Config file for the install tree
SET(WAVE_EXTRA_CMAKE_DIR "cmake")
CONFIGURE_PACKAGE_CONFIG_FILE(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/waveConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/waveConfig.cmake"
    INSTALL_DESTINATION "${INSTALL_CMAKE_DIR}")

# Generate the Version file
WRITE_BASIC_PACKAGE_VERSION_FILE(waveConfigVersion.cmake
    VERSION ${WAVE_PACKAGE_VERSION}
    COMPATIBILITY SameMajorVersion)

# Install the Config and ConfigVersion files
INSTALL(FILES
    "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/waveConfig.cmake"
    "${PROJECT_BINARY_DIR}/waveConfigVersion.cmake"
    DESTINATION "${INSTALL_CMAKE_DIR}")

# Install our import scripts so we can use them in the Config script. That way,
# other projects using libwave will get transient dependencies automatically
INSTALL(DIRECTORY cmake/
    DESTINATION "${INSTALL_CMAKE_DIR}/cmake"
    FILES_MATCHING PATTERN "Import*.cmake" PATTERN "Find*.cmake")

IF(EXPORT_BUILD)
    # Export this build so the package can be found through CMake's registry
    # without being installed.
    EXPORT(PACKAGE wave)

    # Generate the Config file for the build tree
    # It differs from the installed Config file in where our bundled
    # Import*.cmake scripts are included from. Here, use the relative path from
    # the build directory to this source directory
    FILE(RELATIVE_PATH path_to_wave_source
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR})
    SET(WAVE_EXTRA_CMAKE_DIR "${path_to_wave_source}cmake")
    CONFIGURE_PACKAGE_CONFIG_FILE(
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/waveConfig.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/waveConfig.cmake"
        INSTALL_DESTINATION "${INSTALL_CMAKE_DIR}")
ENDIF(EXPORT_BUILD)
