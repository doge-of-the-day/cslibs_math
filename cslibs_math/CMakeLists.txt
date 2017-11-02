cmake_minimum_required(VERSION 2.8.3)
project(cslibs_math)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include(cmake/cslibs_math_enable_c++11.cmake)
include(cmake/cslibs_math_extras.cmake)
include(cmake/cslibs_math_openmp.cmake)
include(cmake/cslibs_math_show_headers.cmake)
include(cmake/cslibs_math_add_unit_test_gtest.cmake)

find_package(catkin QUIET)
if(${catkin_FOUND})
catkin_package(
    INCLUDE_DIRS    include
    DEPENDS         Eigen3
    CFG_EXTRAS      cslibs_math_extras.cmake
                    cslibs_math_enable_c++11.cmake
                    cslibs_math_openmp.cmake
                    cslibs_math_show_headers.cmake
                    cslibs_math_add_unit_test_gtest.cmake)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
endif()

find_package(Eigen3 REQUIRED)
find_package(tf                    QUIET)
find_package(cslibs_boost_geometry QUIET)

include_directories(
  ${Eigen3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  include
  ${tf_INCLUDE_DIRS}
  ${cslibs_boost_geometry_INCLUDE_DIRS}
)

cslibs_math_show_headers()

cslibs_math_add_unit_test_gtest(test_common
    SRCS test/common.cpp
)

cslibs_math_add_unit_test_gtest(test_array
    SRCS test/array.cpp
)
