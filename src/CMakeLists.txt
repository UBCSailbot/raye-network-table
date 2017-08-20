###########
# SOURCES #
###########

set(LIB_SRCS
        network-table/Server.cpp
        network-table/Connection.cpp
        )

set(LIB_HDRS
        network-table/Server.h
        network-table/Connection.h
        )

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS
    network-table/protofiles/Reply.proto
    network-table/protofiles/Request.proto
    network-table/protofiles/SetValueRequest.proto
    network-table/protofiles/GetValueReply.proto
    network-table/protofiles/GetValueRequest.proto
    network-table/protofiles/Value.proto
    )

# Define the src_core library target
add_library(src_core STATIC ${LIB_SRCS} ${LIB_HDRS} ${PROTO_SRCS} ${PROTO_HDRS})
