#################
# CORE LIBRARIES #
#################


find_package(Boost COMPONENTS system filesystem serialization REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

if (ENABLE_ROS)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)
include_directories(${catkin_INCLUDE_DIRS})
# I don't know the proper way to include the header files for
# the built ROS custom messages, this probably
# isn't it.
include_directories(src ${CMAKE_BINARY_DIR}/devel/include)
endif()

include(FindPkgConfig)
pkg_search_module(ZMQ REQUIRED libzmq)
include_directories(${ZMQ_INCLUDE_DIRS})

set(PROTOBUF_FOLDER ${PROJECT_SOURCE_DIR}/lib/protobuf/cmake/build)
message(${PROTOBUF_FOLDER})
set(CMAKE_PREFIX_PATH
    ${CMAKE_PREFIX_PATH}
    ${PROTOBUF_FOLDER}
)
find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})

