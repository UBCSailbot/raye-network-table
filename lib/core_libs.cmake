#################
# CORE LIBRARIES #
#################

set(CORE_LIBS)

list(APPEND CORE_LIBS ${CMAKE_THREAD_LIBS_INIT})

find_package(Boost 1.58 REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

include(FindPkgConfig)
pkg_search_module(ZMQ REQUIRED libzmq)
include_directories(${AISLIB_INCLUDE_DIRS} ${ZMQ_INCLUDE_DIRS})
