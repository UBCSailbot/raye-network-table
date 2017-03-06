#################
# CORE LIBRARIES #
#################

set( CORE_LIBS )

list(APPEND CORE_LIBS ${CMAKE_THREAD_LIBS_INIT})

find_package(Boost 1.58 REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

# Generates source for shared message data types using protobuf
add_subdirectory(${CMAKE_SOURCE_DIR}/lib/protofiles)
include_directories(${ProtobufIncludePath})
list(APPEND CORE_LIBS protofiles)

