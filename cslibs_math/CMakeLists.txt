cmake_minimum_required(VERSION 2.8.3)
project(cslibs_math)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include(cmake/cslibs_math_enable_c++17.cmake)
include(cmake/cslibs_math_extras.cmake)
include(cmake/cslibs_math_add_unit_test_gtest.cmake)
include(cmake/cslibs_math_add_unit_test_ros.cmake)


find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
    INCLUDE_DIRS
        include
    CFG_EXTRAS
        cslibs_math_extras.cmake
        cslibs_math_enable_c++17.cmake
        cslibs_math_add_unit_test_gtest.cmake
    DEPENDS
        Eigen3
)

include_directories(
    SYSTEM
       ${Eigen3_INCLUDE_DIR}
)

set(TARGET_INCLUDE_DIRS
    include
)

cslibs_math_add_unit_test_gtest(mod
    INCLUDE_DIRS
        ${TARGET_INCLUDE_DIRS}
    SOURCE_FILES
        test/mod.cpp
)

cslibs_math_add_unit_test_gtest(div
    INCLUDE_DIRS
        ${TARGET_INCLUDE_DIRS}
    SOURCE_FILES
        test/div.cpp
)

cslibs_math_add_unit_test_gtest(array
    INCLUDE_DIRS
        ${TARGET_INCLUDE_DIRS}
    SOURCE_FILES
        test/array.cpp
)

cslibs_math_add_unit_test_gtest(random
    INCLUDE_DIRS
        ${TARGET_INCLUDE_DIRS}
    SOURCE_FILES
        test/random.cpp
)

cslibs_math_add_unit_test_gtest(sqrt
    INCLUDE_DIRS
        ${TARGET_INCLUDE_DIRS}
    SOURCE_FILES
        test/sqrt.cpp
)

cslibs_math_add_unit_test_gtest(distribution_serialization
    INCLUDE_DIRS
        ${TARGET_INCLUDE_DIRS}
    SOURCE_FILES
        test/distribution_serialization.cpp
    LINK_LIBRARIES
        yaml-cpp
)

cslibs_math_add_unit_test_ros(distribution
    LAUNCH_FILES
        test/test_distribution.test
    INCLUDE_DIRS
        ${TARGET_INCLUDE_DIRS}
    SOURCE_FILES
        test/test_node_distribution.cpp
    LINK_LIBRARIES
        yaml-cpp
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
)

cslibs_math_add_unit_test_ros(distribution_float
    LAUNCH_FILES
        test/test_distribution_float.test
    INCLUDE_DIRS
        ${TARGET_INCLUDE_DIRS}
    SOURCE_FILES
        test/test_node_distribution_float.cpp
    LINK_LIBRARIES
        yaml-cpp
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
