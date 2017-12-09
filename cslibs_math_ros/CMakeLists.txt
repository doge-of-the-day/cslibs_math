cmake_minimum_required(VERSION 2.8.3)
project(cslibs_math_ros)

find_package(catkin REQUIRED)

find_package(tf             QUIET)
find_package(sensor_msgs    QUIET)
find_package(roscpp         QUIET)
find_package(geometry_msgs  QUIET)
find_package(cslibs_time    QUIET)
find_package(cslibs_utility QUIET)
find_package(cslibs_math    QUIET)
find_package(cslibs_math_2d QUIET)
find_package(cslibs_math_3d QUIET)

catkin_package(
    INCLUDE_DIRS    include
)

include_directories(
    ${Eigen3_INCLUDE_DIR}
    ${roscpp_INCLUDE_DIRS}
    ${sensors_msgs_INCLUDE_DIRS}
    ${tf_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${cslibs_time_INCLUDE_DIRS}
    ${cslibs_utility_INCLUDE_DIRS}
    ${cslibs_math_INCLUDE_DIRS}
    ${cslibs_math_2d_ty_INCLUDE_DIRS}
    ${cslibs_math_3d_INCLUDE_DIRS}
    include
)

cslibs_math_ros_show_headers()

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
