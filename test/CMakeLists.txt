project(tests)

# Only compile googletest into a static library
# if ROS is not enabled.
# If ROS is labelled, it already does this,
# and this will cause cmake to fail.
if (NOT ENABLE_ROS)
# Include Google Test
add_subdirectory(lib/googletest)
endif()

# Include test categories
add_subdirectory(basic_tests)
