cmake_minimum_required(VERSION 2.8.3)
project(cslibs_math_2d)

find_package(catkin REQUIRED COMPONENTS
    cslibs_math
)

catkin_package(
    INCLUDE_DIRS
        include
    CATKIN_DEPENDS
        cslibs_math
)

set(TARGET_INCLUDE_DIRS
    include
    ${catkin_INCLUDE_DIRS}
)

find_package(tf QUIET)
if(${tf_FOUND})
    cslibs_math_2d_add_unit_test_gtest(transform
        INCLUDE_DIRS
            ${TARGET_INCLUDE_DIRS}
            ${tf_INCLUDE_DIRS}
        SOURCE_FILES
            test/transform.cpp
        LINK_LIBRARIES
    )
    cslibs_math_2d_add_unit_test_gtest(vector
        INCLUDE_DIRS
            ${TARGET_INCLUDE_DIRS}
            ${tf_INCLUDE_DIRS}
        SOURCE_FILES
            test/vector.cpp
    )
endif()

cslibs_math_2d_add_unit_test_gtest(ray_tracing
    INCLUDE_DIRS
        ${TARGET_INCLUDE_DIRS}
    SOURCE_FILES
        test/ray_tracing.cpp
)

cslibs_math_2d_add_unit_test_gtest(ellipse
    INCLUDE_DIRS
        ${TARGET_INCLUDE_DIRS}
    SOURCE_FILES
        test/ellipse_test.cpp
)


cslibs_math_2d_add_unit_test_gtest(box
    INCLUDE_DIRS
        ${TARGET_INCLUDE_DIRS}
    SOURCE_FILES
        test/box.cpp
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

