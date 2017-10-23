cmake_minimum_required(VERSION 2.8.3)
project(cslibs_math)

include(cmake/cslibs_math_enable_c++11.cmake)
include(cmake/cslibs_math_extras.cmake)
include(cmake/cslibs_math_openmp.cmake)
include(cmake/cslibs_math_show_headers.cmake)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS
    tf
)
find_package(Eigen3 REQUIRED)

catkin_package(
    INCLUDE_DIRS    include
    DEPENDS         Eigen3
    CFG_EXTRAS      cslibs_math_extras.cmake
                    cslibs_math_enable_c++11.cmake
                    cslibs_math_openmp.cmake
                    cslibs_math_show_headers.cmake
)

include_directories(
  ${Eigen3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  include
)

cslibs_math_show_headers()

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
