include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

set(TEST_FILES
    HelpTest.cpp)

add_executable(run_basic_tests ${TEST_FILES})

target_link_libraries(run_basic_tests nt_server nt_client ${PROTOBUF_LIBRARIES} gtest gtest_main)
