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
        NonProtoConnection.cpp
        )

set(NT_CLIENT_HDRS
        Connection.h
        Help.h
        NonProtoConnection.h
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
target_compile_definitions(nt_client PUBLIC WELCOME_DIRECTORY=${WELCOME_DIRECTORY})
target_compile_definitions(nt_server PUBLIC WELCOME_DIRECTORY=${WELCOME_DIRECTORY})

add_subdirectory(python)

# Executing network-table/scripts/generate_frameID_json.py
# Generated file will be in network-table/src/frame_ids.json
execute_process(COMMAND python3 generate_frameID_json.py WORKING_DIRECTORY "../scripts/utility")
