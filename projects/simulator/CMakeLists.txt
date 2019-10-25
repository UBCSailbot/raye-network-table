# Set a variable for commands below
set(PROJECT_NAME simulator)

# Define your project and language
project(${PROJECT_NAME} CXX)

# Define the source code
set(${PROJECT_NAME}_SRCS main.cpp)

add_message_files(
  FILES
  Sensors.msg
)
generate_messages()

# Define the executable
add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS})
target_link_libraries(${PROJECT_NAME} ${ZMQ_LIBRARIES}
    ${PROTOBUF_LIBRARIES} ${catkin_LIBRARIES} src_core)
