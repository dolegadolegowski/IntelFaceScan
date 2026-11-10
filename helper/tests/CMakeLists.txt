# License: Apache 2.0. See LICENSE file in root directory.
#
# Capture engine tests, the helper's own tests and the synthetic recording generator (no camera needed). Standalone
# project:
#
#   cmake -S helper/tests -B build/helper-tests -DIFS_SDK_DIR=<librealsense install prefix>
#   cmake --build build/helper-tests -j
#   ctest --test-dir build/helper-tests --output-on-failure      (or build/helper-tests/capture-tests [filter])
#   build/helper-tests/make-test-recording -o head.db3 --frames 90
#
# helper-tests (helper_tests.cpp) covers the recording spool and runs the helper binary: its command line, exit
# status and a live session as the user. It gets the binary in IFS_HELPER_UNDER_TEST, which ctest sets when this
# folder is built as part of the helper build (below); standalone, those tests are skipped unless it is set by hand:
#   IFS_HELPER_UNDER_TEST=build/helper/IntelFaceScanHelper build/helper-tests/helper-tests
#
# The helper build can include it behind an option (helper/CMakeLists.txt, owned by the platform module):
#
#   option(IFS_HELPER_TESTS "Build the capture tests and make-test-recording" OFF)
#   if(IFS_HELPER_TESTS)
#       add_subdirectory(tests)
#   endif()
cmake_minimum_required(VERSION 3.20)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    project(IntelFaceScanHelperTests LANGUAGES C CXX)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_OSX_ARCHITECTURES arm64 CACHE STRING "")
    if(NOT CMAKE_OSX_DEPLOYMENT_TARGET)
        set(CMAKE_OSX_DEPLOYMENT_TARGET 27.2)
    endif()
    if(NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE Release)
    endif()
    set(IFS_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../build/sdk/install" CACHE PATH "librealsense install prefix")
    # Search IFS_SDK_DIR on every configure (find_package would keep a cached realsense2_DIR of another SDK).
    unset(realsense2_DIR CACHE)
    find_package(realsense2 REQUIRED CONFIG PATHS "${IFS_SDK_DIR}" NO_DEFAULT_PATH)
endif()

set(IFS_HELPER_SRC "${CMAKE_CURRENT_SOURCE_DIR}/../src")

# The capture module exactly as the helper compiles it (capture_engine.cpp, capture_*.cpp, probe.cpp).
file(GLOB IFS_CAPTURE_SOURCES CONFIGURE_DEPENDS "${IFS_HELPER_SRC}/capture_*.cpp")
add_library(ifs_capture STATIC ${IFS_CAPTURE_SOURCES} "${IFS_HELPER_SRC}/probe.cpp")
target_include_directories(ifs_capture PUBLIC "${IFS_HELPER_SRC}" "${IFS_SDK_DIR}/include")
target_compile_options(ifs_capture PRIVATE -Wall -Wextra -Wno-unused-parameter)
target_link_libraries(ifs_capture PUBLIC realsense2::realsense2)

add_library(ifs_test_recording STATIC test_recording.cpp)
target_include_directories(ifs_test_recording PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${IFS_SDK_DIR}/include")
target_compile_options(ifs_test_recording PRIVATE -Wall -Wextra -Wno-unused-parameter)
target_link_libraries(ifs_test_recording PUBLIC realsense2::realsense2)

add_executable(make-test-recording make_test_recording.cpp)
target_link_libraries(make-test-recording PRIVATE ifs_test_recording)

add_executable(capture-tests capture_tests.cpp)
target_compile_options(capture-tests PRIVATE -Wall -Wextra -Wno-unused-parameter)
target_link_libraries(capture-tests PRIVATE ifs_capture ifs_test_recording)

set_target_properties(make-test-recording capture-tests PROPERTIES
    BUILD_RPATH "${IFS_SDK_DIR}/lib"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}")

# The helper's own tests: the recording spool as the helper compiles it, and the helper process (no librealsense
# here; nlohmann/json comes with the SDK).
set(IFS_PROTOCOL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../Sources/IFSProtocol")
add_executable(helper-tests helper_tests.cpp
    "${IFS_HELPER_SRC}/security_recording.cpp"
    "${IFS_HELPER_SRC}/ipc_wire.cpp"
    "${IFS_PROTOCOL_DIR}/ifs_protocol.c")
target_include_directories(helper-tests PRIVATE "${IFS_HELPER_SRC}" "${IFS_PROTOCOL_DIR}/include")
target_include_directories(helper-tests SYSTEM PRIVATE "${IFS_SDK_DIR}/include")
target_compile_options(helper-tests PRIVATE -Wall -Wextra -Wno-unused-parameter)
set_target_properties(helper-tests PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}")

enable_testing()
add_test(NAME capture-tests COMMAND capture-tests)
add_test(NAME helper-tests COMMAND helper-tests)
if(TARGET IntelFaceScanHelper)
    set_tests_properties(helper-tests PROPERTIES
        ENVIRONMENT "IFS_HELPER_UNDER_TEST=$<TARGET_FILE:IntelFaceScanHelper>")
endif()
