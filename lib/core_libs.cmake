#################
# CORE LIBRARIES #
#################

set(CORE_LIBS)

list(APPEND CORE_LIBS ${CMAKE_THREAD_LIBS_INIT})

find_package(Boost COMPONENTS system filesystem REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

include(FindPkgConfig)
pkg_search_module(ZMQ REQUIRED libzmq)
include_directories(${ZMQ_INCLUDE_DIRS})

find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})

