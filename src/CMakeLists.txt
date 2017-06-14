###########
# SOURCES #
###########

set(LIB_SRCS
        network-table/network-table.cpp
        )

set(LIB_HDRS
        network-table/network-table.h
        )

# Define the src_core library target
add_library(src_core STATIC ${LIB_SRCS} ${LIB_HDRS})
