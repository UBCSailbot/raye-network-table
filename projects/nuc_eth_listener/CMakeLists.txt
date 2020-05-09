# Set a variable for commands below
set(PROJECT_NAME nuc_eth_listener)

# this is where the custom messages are
# we have to build them and use them
# in nuc_eth_listener
add_subdirectory(sailbot-msg)

# Define your project and language
project(${PROJECT_NAME} CXX)

# Define the source code
set(${PROJECT_NAME}_SRCS main.cpp)

# Define the executable
add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS})
target_link_libraries(${PROJECT_NAME} ${ZMQ_LIBRARIES}
    ${PROTOBUF_LIBRARIES} ${catkin_LIBRARIES} src_core)
