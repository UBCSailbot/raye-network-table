#################
# CORE LIBRARIES #
#################

set( CORE_LIBS )

list(APPEND CORE_LIBS ${CMAKE_THREAD_LIBS_INIT})

find_package(Boost 1.58 REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

add_subdirectory("lib/SimpleAmqpClient")
include_directories("lib/SimpleAmqpClient/src")
