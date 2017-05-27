# Set a variable for commands below
set(PROJECT_NAME network-table)

# Define your project and language
project(${PROJECT_NAME} CXX)

# Define the source code
set(${PROJECT_NAME}_SRCS main.cpp)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../lib/SimpleAmqpClient ${CMAKE_CURRENT_BINARY_DIR}/SimpleAmqpClient)

#include_directories(${SimpleAmqpClient_INCLUDE_DIRS})

# Define the executable
add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS})
target_link_libraries(${PROJECT_NAME} SimpleAmqpClient)
