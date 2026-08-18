cmake_minimum_required(VERSION 3.16)
project(litearm-cpp VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ── Protobuf ──────────────────────────────────────────────────────────────────
# Requires protobuf >= 3.19. A system install is found automatically.
# For a custom install prefix (e.g. a conda env), pass:
#     cmake .. -DPROTOBUF_ROOT=$CONDA_PREFIX
set(PROTOBUF_ROOT "" CACHE PATH "Protobuf install prefix (optional)")
if(PROTOBUF_ROOT)
    list(PREPEND CMAKE_PREFIX_PATH "${PROTOBUF_ROOT}")
endif()

find_path(PROTOBUF_INCLUDE_DIR google/protobuf/message.h
    HINTS "${PROTOBUF_ROOT}/include"
    REQUIRED)

find_library(PROTOBUF_LIBRARY protobuf
    HINTS "${PROTOBUF_ROOT}/lib"
    REQUIRED)

find_program(PROTOC_EXECUTABLE protoc
    HINTS "${PROTOBUF_ROOT}/bin"
    REQUIRED)

message(STATUS "Protobuf include: ${PROTOBUF_INCLUDE_DIR}")
message(STATUS "Protobuf library: ${PROTOBUF_LIBRARY}")
message(STATUS "Protoc: ${PROTOC_EXECUTABLE}")

# ── Proto generation ──────────────────────────────────────────────────────────
set(PROTO_FILE "${CMAKE_SOURCE_DIR}/proto/litearm.proto")
set(PROTO_GEN_DIR "${CMAKE_SOURCE_DIR}/generated")

add_custom_command(
    OUTPUT "${PROTO_GEN_DIR}/litearm.pb.cc" "${PROTO_GEN_DIR}/litearm.pb.h"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${PROTO_GEN_DIR}"
    COMMAND ${PROTOC_EXECUTABLE}
        --cpp_out="${PROTO_GEN_DIR}"
        --proto_path="${CMAKE_SOURCE_DIR}/proto"
        "${PROTO_FILE}"
    DEPENDS "${PROTO_FILE}"
    COMMENT "Generating protobuf C++ code from litearm.proto"
)

add_custom_target(proto_gen DEPENDS
    "${PROTO_GEN_DIR}/litearm.pb.cc"
    "${PROTO_GEN_DIR}/litearm.pb.h"
)

# ── Main library ──────────────────────────────────────────────────────────────
add_library(litearm
    src/exceptions.cpp
    src/types.cpp
    src/codec.cpp
    src/transport.cpp
    src/arm.cpp
    src/device.cpp
    generated/litearm.pb.cc
)

target_include_directories(litearm PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROTOBUF_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${PROTO_GEN_DIR}>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(litearm PUBLIC ${PROTOBUF_LIBRARY})
target_compile_options(litearm PRIVATE -Wall -Wextra -Wpedantic)

add_dependencies(litearm proto_gen)

# ── Tests ─────────────────────────────────────────────────────────────────────
# GoogleTest is downloaded automatically on first configure.
# For offline builds, point to a local source tree:
#     cmake .. -DGOOGLETEST_ROOT=/usr/src/googletest
include(FetchContent)
set(GOOGLETEST_ROOT "" CACHE PATH "Local GoogleTest source dir (optional)")
if(GOOGLETEST_ROOT)
    FetchContent_Declare(googletest SOURCE_DIR ${GOOGLETEST_ROOT})
else()
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.14.0
    )
endif()
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

function(add_litearm_test test_name test_source)
    add_executable(${test_name} ${test_source})
    target_link_libraries(${test_name} PRIVATE litearm gtest gtest_main)
    target_include_directories(${test_name} PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${PROTOBUF_INCLUDE_DIR}
        ${PROTO_GEN_DIR}
    )
    add_test(NAME ${test_name} COMMAND ${test_name})
endfunction()

add_litearm_test(test_protocol tests/test_protocol.cpp)
add_litearm_test(test_codec tests/test_codec.cpp)
add_litearm_test(test_transport tests/test_transport.cpp)
add_litearm_test(test_arm tests/test_arm.cpp)
add_litearm_test(test_device tests/test_device.cpp)
