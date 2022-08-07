# Set a variable for commands below
set(PROJECT_NAME bbb_canbus_listener)

# Define your project and language
project(${PROJECT_NAME} CXX)

add_subdirectory(bms_pwr_mgr)

# Define the source code
set(${PROJECT_NAME}_SRCS main.cpp)

# Define the executable
add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS})
target_link_libraries(${PROJECT_NAME} ${ZMQ_LIBRARIES} ${PROTOBUF_LIBRARIES} nt_client bms_pwr_mgr)
target_include_directories(${PROJECT_NAME} PUBLIC
                           "${CMAKE_CURRENT_SOURCE_DIR}/bms_pwr_mgr")
