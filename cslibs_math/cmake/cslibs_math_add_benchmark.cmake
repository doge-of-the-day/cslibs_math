# ${PROJECT_NAME}_add_benchmark(<NAME_OF_THE_BENCHMARK>
#    [<INCLUDE_DIRS> ...]
#    [<SOURCE_FILES> ...]
#    [<LINK_LIBRARIES> ...]
#    [<COMPILE_FLAGS> ...]
#)
function(${PROJECT_NAME}_add_benchmark)
    cmake_parse_arguments(bench
        # list of names of the boolean arguments (only defined ones will be true)
        ""
        # list of names of mono-valued arguments
        ""
        # list of names of multi-valued arguments (output variables are lists)
        "INCLUDE_DIRS;SOURCE_FILES;LINK_LIBRARIES;COMPILE_OPTIONS"
        # arguments of the function to parse, here we take the all original ones
        ${ARGN}
    )

    set(bench_NAME ${ARGV0})
    find_package(benchmark QUIET)
    if(${benchmark_FOUND})
	    find_package(Boost REQUIRED COMPONENTS system)

	    add_executable(${bench_NAME}
	        ${bench_SOURCE_FILES}
	    )

	    target_include_directories(${bench_NAME}
	        PRIVATE
	            ${bench_INCLUDE_DIRS}
	            ${benchmark_INCLUDE_DIRS}
	    )

	    target_link_libraries(${bench_NAME}
	        PRIVATE
	            ${bench_LINK_LIBRARIES}
	            ${Boost_LIBRARIES}
	             benchmark::benchmark
	            -pthread
	    )
	    target_compile_options(${bench_NAME}
	        PRIVATE
	            ${bench_COMPILE_OPTIONS}
	    )
	endif()
endfunction()
