###########
# SOURCES #
###########

set(LIB_SRCS
        Server.cpp
        Help.cpp
        Connection.cpp
        )

set(LIB_HDRS
        Server.h
        Help.h
        Connection.h
        )

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS
    protofiles/ActuationAngle.proto
    protofiles/Node.proto
    protofiles/Reply.proto
    protofiles/Request.proto
    protofiles/SetValuesRequest.proto
    protofiles/GetNodesReply.proto
    protofiles/GetNodesRequest.proto
    protofiles/Sensors.proto
    protofiles/SubscribeRequest.proto
    protofiles/SubscribeReply.proto
    protofiles/Uccms.proto
    protofiles/UnsubscribeRequest.proto
    protofiles/Value.proto
    protofiles/ErrorReply.proto
    )

# Define the src_core library target
add_library(src_core STATIC ${LIB_SRCS} ${LIB_HDRS} ${PROTO_SRCS} ${PROTO_HDRS})
