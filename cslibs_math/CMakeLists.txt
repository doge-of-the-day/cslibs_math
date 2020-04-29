cmake_minimum_required(VERSION 2.8.3)
project(cslibs_math)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include(cmake/cslibs_math_enable_c++17.cmake)
include(cmake/cslibs_math_add_unit_test_gtest.cmake)
include(cmake/cslibs_math_add_unit_test_ros.cmake)
include(cmake/cslibs_math_add_benchmark.cmake)

find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
    INCLUDE_DIRS
        include
    CFG_EXTRAS
        cslibs_math_enable_c++17.cmake
        cslibs_math_add_unit_test_gtest.cmake
        cslibs_math_add_unit_test_ros.cmake
        cslibs_math_add_benchmark.cmake
    DEPENDS
        EIGEN3
)

set(TARGET_INCLUDE_DIRS
    include
    ${Eigen3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME}
    INTERFACE
        ${TARGET_INCLUDE_DIRS}
)

if(NOT ${CMAKE_BUILD_TYPE} STREQUAL Debug)
    set(TARGET_COMPILE_OPTIONS
        -Wall -Werror -Ofast -ffast-math
    )
    # enabling -march=native my result in problems on some machines,
    # my try -mtune=native

    message(STATUS "[${PROJECT_NAME}]: Compiling with optimization!")
endif()


cslibs_math_add_unit_test_gtest(mod
    INCLUDE_DIRS
        ${TARGET_INCLUDE_DIRS}
    SOURCE_FILES
        test/mod.cpp
    COMPILE_OPTIONS
        ${TARGET_COMPILE_OPTIONS}
)

cslibs_math_add_unit_test_gtest(div
    INCLUDE_DIRS
        ${TARGET_INCLUDE_DIRS}
    SOURCE_FILES
        test/div.cpp
    COMPILE_OPTIONS
        ${TARGET_COMPILE_OPTIONS}
)

cslibs_math_add_unit_test_gtest(array
    INCLUDE_DIRS
        ${TARGET_INCLUDE_DIRS}
    SOURCE_FILES
        test/array.cpp
    COMPILE_OPTIONS
        ${TARGET_COMPILE_OPTIONS}
)

cslibs_math_add_unit_test_gtest(random
    INCLUDE_DIRS
        ${TARGET_INCLUDE_DIRS}
    SOURCE_FILES
        test/random.cpp
    COMPILE_OPTIONS
        ${TARGET_COMPILE_OPTIONS}
)

cslibs_math_add_unit_test_gtest(sqrt
    INCLUDE_DIRS
        ${TARGET_INCLUDE_DIRS}
    SOURCE_FILES
        test/sqrt.cpp
    COMPILE_OPTIONS
        ${TARGET_COMPILE_OPTIONS}
)

cslibs_math_add_unit_test_gtest(distribution_serialization
    INCLUDE_DIRS
        ${TARGET_INCLUDE_DIRS}
    SOURCE_FILES
        test/distribution_serialization.cpp
    LINK_LIBRARIES
        yaml-cpp
    COMPILE_OPTIONS
        ${TARGET_COMPILE_OPTIONS}
)

cslibs_math_add_unit_test_ros(distribution
    LAUNCH_FILE
        test/test_distribution.test
    INCLUDE_DIRS
        ${TARGET_INCLUDE_DIRS}
    SOURCE_FILES
        test/test_node_distribution.cpp
    LINK_LIBRARIES
        yaml-cpp
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
    COMPILE_OPTIONS
        ${TARGET_COMPILE_OPTIONS}
)

cslibs_math_add_unit_test_ros(distribution_float
    LAUNCH_FILE
        test/test_distribution_float.test
    INCLUDE_DIRS
        ${TARGET_INCLUDE_DIRS}
    SOURCE_FILES
        test/test_node_distribution_float.cpp
    LINK_LIBRARIES
        yaml-cpp
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
    COMPILE_OPTIONS
        ${TARGET_COMPILE_OPTIONS}
)

cslibs_math_add_benchmark(benchmark_exp
    INCLUDE_DIRS
        ${TARGET_INCLUDE_DIRS}
    SOURCE_FILES
        benchmark/benchmark_exp.cpp
    COMPILE_OPTIONS
        ${TARGET_COMPILE_OPTIONS}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
