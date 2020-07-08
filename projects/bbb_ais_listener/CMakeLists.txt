#########################
# bbb_ais_listener main #
#########################
# Set a variable for commands below
set(PROJECT_NAME bbb_ais_listener)

# Define your project and language
project(${PROJECT_NAME} CXX)

# Define the source code
set(${PROJECT_NAME}_SRCS main.cpp)

# Define the executable
add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS})
target_link_libraries(${PROJECT_NAME} ${ZMQ_LIBRARIES} ${PROTOBUF_LIBRARIES} src_core aislib)
link_directories(ais/aislib ais/aisd)

# Directories with required header files
include_directories(ais/aislib ais/aisd)

#############################
# bbb_ais_listener mock_ais #
#############################
# Set a variable for commands below
set(TEST_NAME mock_ais)

# Define your project and language
project(${TEST_NAME} CXX)

# Define the source code
set(${TEST_NAME}_SRCS test.cpp)

# Define the executable
add_executable(${TEST_NAME} ${${TEST_NAME}_SRCS})
target_link_libraries(${TEST_NAME} ${ZMQ_LIBRARIES} ${PROTOBUF_LIBRARIES} src_core aislib)
link_directories(ais/aislib ais/aisd)

include_directories(ais/aislib ais/aisd)

################################
# Adding required subdirectories
add_subdirectory(ais/aislib)
add_subdirectory(ais/aisd)
