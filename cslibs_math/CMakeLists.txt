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

cslibs_math_add_unit_test_gtest(test_mod
    INCLUDE_DIRS
        ${TARGET_INCLUDE_DIRS}
    SOURCE_FILES
        test/test_mod.cpp
    COMPILE_OPTIONS
        ${TARGET_COMPILE_OPTIONS}
)

cslibs_math_add_unit_test_gtest(test_pow
    INCLUDE_DIRS
        ${TARGET_INCLUDE_DIRS}
    SOURCE_FILES
        test/test_pow.cpp
    COMPILE_OPTIONS
        ${TARGET_COMPILE_OPTIONS}
)

# cslibs_math_add_unit_test_gtest(test_exp
#     INCLUDE_DIRS
#         ${TARGET_INCLUDE_DIRS}
#     SOURCE_FILES
#         test/test_exp.cpp
#     COMPILE_OPTIONS
#         ${TARGET_COMPILE_OPTIONS}
# )

cslibs_math_add_unit_test_gtest(test_div
    INCLUDE_DIRS
        ${TARGET_INCLUDE_DIRS}
    SOURCE_FILES
        test/test_div.cpp
    COMPILE_OPTIONS
        ${TARGET_COMPILE_OPTIONS}
)

cslibs_math_add_unit_test_gtest(test_array
    INCLUDE_DIRS
        ${TARGET_INCLUDE_DIRS}
    SOURCE_FILES
        test/test_array.cpp
    COMPILE_OPTIONS
        ${TARGET_COMPILE_OPTIONS}
)

cslibs_math_add_unit_test_gtest(test_random
    INCLUDE_DIRS
        ${TARGET_INCLUDE_DIRS}
    SOURCE_FILES
        test/test_random.cpp
    COMPILE_OPTIONS
        ${TARGET_COMPILE_OPTIONS}
)

cslibs_math_add_unit_test_gtest(test_sqrt
    INCLUDE_DIRS
        ${TARGET_INCLUDE_DIRS}
    SOURCE_FILES
        test/test_sqrt.cpp
    COMPILE_OPTIONS
        ${TARGET_COMPILE_OPTIONS}
)

cslibs_math_add_unit_test_gtest(test_fractional
    INCLUDE_DIRS
        ${TARGET_INCLUDE_DIRS}
    SOURCE_FILES
        test/test_fractional.cpp
    COMPILE_OPTIONS
        ${TARGET_COMPILE_OPTIONS}
)

find_package(yaml-cpp QUIET)
if(${YAML_CPP_FOUND})
    cslibs_math_add_unit_test_gtest(test_distribution_serialization
        INCLUDE_DIRS
            ${TARGET_INCLUDE_DIRS}
            ${YAML_CPP_INCLUDE_DIRS}
        SOURCE_FILES
            test/test_distribution_serialization.cpp
        LINK_LIBRARIES
            yaml-cpp
        COMPILE_OPTIONS
            ${TARGET_COMPILE_OPTIONS}
    )


    cslibs_math_add_unit_test_ros(test_distribution
        LAUNCH_FILE
            test/test_distribution.test
        INCLUDE_DIRS
            ${TARGET_INCLUDE_DIRS}
            ${YAML_CPP_INCLUDE_DIRS}
        SOURCE_FILES
            test/test_node_distribution.cpp
        LINK_LIBRARIES
            yaml-cpp
            ${Boost_LIBRARIES}
            ${catkin_LIBRARIES}
        COMPILE_OPTIONS
            ${TARGET_COMPILE_OPTIONS}
    )

    cslibs_math_add_unit_test_ros(test_distribution_float
        LAUNCH_FILE
            test/test_distribution_float.test
        INCLUDE_DIRS
            ${TARGET_INCLUDE_DIRS}
            ${YAML_CPP_INCLUDE_DIRS}
        SOURCE_FILES
            test/test_node_distribution_float.cpp
        LINK_LIBRARIES
            yaml-cpp
            ${Boost_LIBRARIES}
            ${catkin_LIBRARIES}
        COMPILE_OPTIONS
            ${TARGET_COMPILE_OPTIONS}
    )
endif()


cslibs_math_add_benchmark(benchmark_exp
    INCLUDE_DIRS
        ${TARGET_INCLUDE_DIRS}
    SOURCE_FILES
        benchmark/benchmark_exp.cpp
    COMPILE_OPTIONS
        ${TARGET_COMPILE_OPTIONS}
)

cslibs_math_add_benchmark(benchmark_angles
    INCLUDE_DIRS
        ${TARGET_INCLUDE_DIRS}
    SOURCE_FILES
        benchmark/benchmark_angles.cpp
    COMPILE_OPTIONS
        ${TARGET_COMPILE_OPTIONS}
)


install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
