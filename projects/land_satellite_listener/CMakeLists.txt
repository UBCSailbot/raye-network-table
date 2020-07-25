# get access to protoc (protobuf compiler)
# so that the .proto files can be converted
# to python
find_program(protoc_executable protoc PATHS ../lib)

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
