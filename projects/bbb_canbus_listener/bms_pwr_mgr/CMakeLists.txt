add_library(bms_pwr_mgr bms_pwr_mgr.cpp)

target_include_directories(bms_pwr_mgr
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})