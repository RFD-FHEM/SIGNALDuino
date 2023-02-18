cmake_minimum_required(VERSION 3.24)

project(TestProject)

message("PROJECT_SOURCE_DIR: ${PROJECT_SOURCE_DIR}")
# Set the output folder where your program will be created
#set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
#set(   LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_CXX_STANDARD 11)


if (POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/install)
set(CMAKE_PREFIX_PATH ${PROJECT_SOURCE_DIR}/install/include/rapidassst-0.10.2;${PROJECT_SOURCE_DIR}/install/include/win32arduino-2.4.0)
##############################################################################################################################################
# Add Library
##############################################################################################################################################
if (CMAKE_VERSION VERSION_LESS 3.2)
    set(UPDATE_DISCONNECTED_IF_AVAILABLE "")
else()
    set(UPDATE_DISCONNECTED_IF_AVAILABLE "UPDATE_DISCONNECTED 1")
    set(FETCHCONTENT_UPDATES_DISCONNECTED "ON")
endif()

# Required include
include(FetchContent)

# Fetching all the required repositories and making them available
FetchContent_Declare(googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG release-1.10.0
    FIND_PACKAGE_ARGS NAMES GTest
    )

FetchContent_Declare(arduino_mock
    GIT_REPOSITORY https://github.com/sidey79/arduino-mock.git
    GIT_TAG sidey79-patch-2)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(arduino_mock googletest)

file( GLOB_RECURSE ARDUINO_LIBRARY_SOURCE_FILES ${PROJECT_SOURCE_DIR}/../src/_micro-api/libraries/output/src/*.h ${PROJECT_SOURCE_DIR}/../src/_micro-api/libraries/output/src/*.cpp ${PROJECT_SOURCE_DIR}/../src/_micro-api/libraries/bitstore/*.cpp  ${PROJECT_SOURCE_DIR}/../src/_micro-api/libraries/fastdelegate/src/*.cpp ${PROJECT_SOURCE_DIR}/../src/_micro-api/libraries/signalDecoder/src/*.cpp  ${PROJECT_SOURCE_DIR}/../src/_micro-api/libraries/bitstore/src/*.h  ${PROJECT_SOURCE_DIR}/../src/_micro-api/libraries/fastdelegate/src/*.h ${PROJECT_SOURCE_DIR}/../src/_micro-api/libraries/signalDecoder/src/*.h 
 ${PROJECT_SOURCE_DIR}/../src/commands.h 
 ${PROJECT_SOURCE_DIR}/../src/functions.h 
 ${PROJECT_SOURCE_DIR}/../src/send.h)
file( GLOB ARDUINO_LIBRARY_TEST_FILES   ${PROJECT_SOURCE_DIR}/testSignalDecoder/tests.cpp  ${PROJECT_SOURCE_DIR}/testSignalDecoder/tests.h ${PROJECT_SOURCE_DIR}/testSignalDecoder/targetver.h )

# Adding the software and main test file as an executable
add_executable(TestProject
  ${ARDUINO_LIBRARY_SOURCE_FILES}
  ${ARDUINO_LIBRARY_TEST_FILES}
  ${PROJECT_SOURCE_DIR}/testSignalDecoder/main.cpp
)
# Link all libraries
target_link_libraries(TestProject
    gtest_main
    arduino_mock
    ${CMAKE_THREAD_LIBS_INIT}
)

include(GoogleTest)

# (Optional) Setting up the compiler
#target_compile_features(TestProject PUBLIC cxx_std_11)
#set_target_properties(TestProject PROPERTIES CXX_EXTENSIONS OFF)

include_directories(
    ../src/
    ../src/_micro-api/libraries/output/src/
    ../src/_micro-api/libraries/output/src/
    ../src/_micro-api/libraries/bitstore/
    ../src/_micro-api/libraries/fastdelegate/src/
    ../src/_micro-api/libraries/signalDecoder/src/
    ../src/_micro-api/libraries/bitstore/src/
    ../src/_micro-api/libraries/fastdelegate/src/
    ../src/_micro-api/libraries/signalDecoder/src/
)

enable_testing()
add_test(all_tests TestProject)
