cmake_minimum_required(VERSION 2.8.3)
project(cslibs_math_3d)

find_package(catkin REQUIRED COMPONENTS
    cslibs_math
)

catkin_package(
    INCLUDE_DIRS
        include
    CATKIN_DEPENDS
        cslibs_math
)

if(NOT ${CMAKE_BUILD_TYPE} STREQUAL Debug)
    set(TARGET_COMPILE_DEFINITIONS
        -Wall -WError -Ofast -ffastmath
    )
    # enabling -march=native my result in problems on some machines,
    # my try -mtune=native

    message(STATUS "[${PROJECT_NAME}]: Compiling with optimization!")
endif()

find_package(tf QUIET)
if(${tf_FOUND})
    set(TARGET_INCLUDE_DIRS
        include
        ${catkin_INCLUDE_DIRS}
        ${tf_INCLUDE_DIRS}
    )
    cslibs_math_3d_add_unit_test_gtest(quaternion
        INCLUDE_DIRS
            ${TARGET_INCLUDE_DIRS}
        SOURCE_FILES
            test/quaternion.cpp
        COMPILE_DEFINITIONS
            ${TARGET_COMPILE_DEFINITIONS}
    )
    cslibs_math_3d_add_unit_test_gtest(transform
        INCLUDE_DIRS
            ${TARGET_INCLUDE_DIRS}
        SOURCE_FILES
            test/transform.cpp
        COMPILE_DEFINITIONS
            ${TARGET_COMPILE_DEFINITIONS}
    )
    cslibs_math_3d_add_unit_test_gtest(ray_tracing
        INCLUDE_DIRS
            ${TARGET_INCLUDE_DIRS}
        SOURCE_FILES
            test/ray_tracing.cpp
        COMPILE_DEFINITIONS
            ${TARGET_COMPILE_DEFINITIONS}
    )
endif()

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
