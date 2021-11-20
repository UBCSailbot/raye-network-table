set(NUC_MOCK_GPS_TEST nuc_eth_listener_test)

if (NOT TARGET sailbot_msg_generate_messages_cpp)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../projects/nuc_eth_listener/sailbot-msg 
        ${CMAKE_CURRENT_BINARY_DIR}/sailbot-msg_build)
endif()

project(${NUC_MOCK_GPS_TEST} CXX)

set(${NUC_MOCK_GPS_TEST}_SRCS main.cpp)

add_executable(${NUC_MOCK_GPS_TEST} ${${NUC_MOCK_GPS_TEST}_SRCS})

target_link_libraries(${NUC_MOCK_GPS_TEST} ${ZMQ_LIBRARIES}
    ${PROTOBUF_LIBRARIES} ${catkin_LIBRARIES} nt_client)

add_dependencies(${NUC_MOCK_GPS_TEST} sailbot_msg_generate_messages_cpp)
