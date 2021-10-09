# Set a variable for commands below
set(PROJECT_NAME nuc_manual_override)

# this is where the custom messages are
# we have to build them and use them
# in nuc_manual_override
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../nuc_eth_listener/sailbot-msg ${CMAKE_CURRENT_BINARY_DIR}/sailbot-msg)

# Define your project and language
project(${PROJECT_NAME} CXX)

# Define the source code
set(${PROJECT_NAME}_SRCS main.cpp)

# Define the executable
add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS})
target_link_libraries(${PROJECT_NAME} ${ZMQ_LIBRARIES}
    ${PROTOBUF_LIBRARIES} ${catkin_LIBRARIES} nt_client)
add_dependencies(${PROJECT_NAME} sailbot_msg_generate_messages_cpp)
