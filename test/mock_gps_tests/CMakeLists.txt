set(MOCK_GPS_TEST mock_gps_tests)

if (NOT TARGET sailbot_msg_generate_messages_cpp)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../projects/nuc_eth_listener/sailbot-msg 
        ${CMAKE_CURRENT_BINARY_DIR}/sailbot-msg_build)
endif()

project(${MOCK_GPS_TEST} CXX)

set(${MOCK_GPS_TEST}_SRCS main.cpp)

add_executable(${MOCK_GPS_TEST} ${${MOCK_GPS_TEST}_SRCS})

target_link_libraries(${MOCK_GPS_TEST} ${ZMQ_LIBRARIES}
    ${PROTOBUF_LIBRARIES} ${catkin_LIBRARIES} nt_client)

add_dependencies(${MOCK_GPS_TEST} sailbot_msg_generate_messages_cpp)