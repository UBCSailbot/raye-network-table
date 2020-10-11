# get access to protoc (protobuf compiler)
# so that the .proto files can be converted
# to python
find_program(protoc_executable protoc PATHS "${CMAKE_SOURCE_DIR}/lib")

# get a list of the .proto files
file(GLOB protofiles "${CMAKE_SOURCE_DIR}/src/protofiles/*.proto")

# create a directory to store the generated python
set(generated_python_folder "${CMAKE_CURRENT_SOURCE_DIR}/generated_python/")
file(MAKE_DIRECTORY "${generated_python_folder}")

# run protoc on each file
foreach(protofile ${protofiles})
    execute_process(COMMAND ${protoc_executable}
        -I=${CMAKE_SOURCE_DIR}/src/protofiles/
        --python_out=${generated_python_folder}
        ${protofile})
endforeach()

# the generated code has absolute imports,
# which causes problems when you are trying to use it
# and you arent in the "generated_python" folder.
# you can fix it with sys.path.append, but thats kind of hacky
# The fix has not yet been merged in (Oct 19 2020):
# https://github.com/protocolbuffers/protobuf/issues/1491
# https://github.com/protocolbuffers/protobuf/pull/7470
# so manually replace the imports with sed
execute_process(COMMAND bash -c "sed -i -E 's/^import.*_pb2/from . \\0/' *.py"
                WORKING_DIRECTORY "${generated_python_folder}")
