find_package(Boost REQUIRED COMPONENTS thread)
# Add the main library file
add_library(bms_pwr_mgr bms_pwr_mgr.cpp)
target_link_libraries(bms_pwr_mgr Boost::thread)
target_include_directories(bms_pwr_mgr
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Add test file and test executable
add_executable(test_bms_pwr_mgr test_bms_pwr_mgr.cpp)
target_link_libraries(test_bms_pwr_mgr bms_pwr_mgr Boost::thread)
target_include_directories(test_bms_pwr_mgr
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
