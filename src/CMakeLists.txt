###########
# SOURCES #
###########

set(LIB_SRCS
        network-table/NetworkTableServer.cpp
        network-table/NetworkTableConnection.cpp
        )

set(LIB_HDRS
        network-table/NetworkTableServer.h
        network-table/NetworkTableConnection.h
        )

# Define the src_core library target
add_library(src_core STATIC ${LIB_SRCS} ${LIB_HDRS})
