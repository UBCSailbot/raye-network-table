###########
# SOURCES #
###########

set(LIB_SRCS
        Server.cpp
        Connection.cpp
        )

set(LIB_HDRS
        Server.h
        Connection.h
        )

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS
    protofiles/KeyValuePair.proto
    protofiles/Reply.proto
    protofiles/Request.proto
    protofiles/SetValuesRequest.proto
    protofiles/GetValuesReply.proto
    protofiles/GetValuesRequest.proto
    protofiles/SubscribeRequest.proto
    protofiles/SubscribeReply.proto
    protofiles/UnsubscribeRequest.proto
    protofiles/Value.proto
    )

# Define the src_core library target
add_library(src_core STATIC ${LIB_SRCS} ${LIB_HDRS} ${PROTO_SRCS} ${PROTO_HDRS})
