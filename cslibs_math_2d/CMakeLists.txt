cmake_minimum_required(VERSION 2.8.3)
project(cslibs_math_2d)

find_package(catkin REQUIRED COMPONENTS
    cslibs_math
)
catkin_package(
    INCLUDE_DIRS    include
    CATKIN_DEPENDS  cslibs_math
    DEPENDS         Eigen3
)
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

include_directories(
    ${Eigen3_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    include
)

cslibs_math_2d_show_headers()

