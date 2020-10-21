###########
# SOURCES #
###########

set(NT_SERVER_SRCS
        Server.cpp
        Help.cpp
        )

set(NT_SERVER_HDRS
        Server.h
        Help.h
        )

set(NT_CLIENT_SRCS
        Connection.cpp
        Help.cpp
        )

set(NT_CLIENT_HDRS
        Connection.h
        Help.h
        )

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS
    protofiles/ActuationAngle.proto
    protofiles/Node.proto
    protofiles/Reply.proto
    protofiles/Request.proto
    protofiles/SetValuesRequest.proto
    protofiles/GetNodesReply.proto
    protofiles/GetNodesRequest.proto
    protofiles/Satellite.proto
    protofiles/Sensors.proto
    protofiles/SubscribeRequest.proto
    protofiles/SubscribeReply.proto
    protofiles/Uccms.proto
    protofiles/UnsubscribeRequest.proto
    protofiles/Value.proto
    protofiles/ErrorReply.proto
    )

add_library(nt_server STATIC ${NT_SERVER_SRCS} ${NT_SERVER_HDRS} ${PROTO_SRCS} ${PROTO_HDRS})
add_library(nt_client STATIC ${NT_CLIENT_SRCS} ${NT_CLIENT_HDRS} ${PROTO_SRCS} ${PROTO_HDRS})

add_subdirectory(python)
