add_subdirectory(bbb_canbus_listener)
add_subdirectory(bbb_eth_listener)
add_subdirectory(bbb_satellite_listener)
add_subdirectory(bbb_ais_listener)
add_subdirectory(client)
add_subdirectory(light_client)
add_subdirectory(init_gps_coords)
add_subdirectory(network_table_server)
add_subdirectory(viewtree)
add_subdirectory(logger_test)
<<<<<<< HEAD
#This is a test client for the logging class. DELETE BEFORE MERGING WITH MAIN
add_subdirectory(log_client)
=======
add_subdirectory(second_logger_test)
>>>>>>> File logging works. Separate mains log to different file names.
if(ENABLE_ROS)
add_subdirectory(nuc_eth_listener)
endif()
