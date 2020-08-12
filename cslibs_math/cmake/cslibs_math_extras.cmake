find_package(yaml-cpp REQUIRED)
# makes the version numbers of yaml-cpp accessible in code and prevents
# redifintion of template overwrites
add_definitions(-DCSLIBS_MATH_YAML_CPP_VERSION_MAJOR=${yaml-cpp_VERSION_MAJOR}
                -DCSLIBS_MATH_YAML_CPP_VERSION_MINOR=${yaml-cpp_VERSION_MINOR}
                -DCSLIBS_MATH_YAML_CPP_VERSION_PATCHT=${yaml-cpp_VERSION_PATCH})