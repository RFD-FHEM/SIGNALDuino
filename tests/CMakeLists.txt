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
    GIT_TAG sidey79-patch-3
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(arduino_mock googletest)

file( GLOB ARDUINO_LIBRARY_SOURCE_FILES
    ${PROJECT_SOURCE_DIR}/../src/_micro-api/libraries/*/src/*.h
    ${PROJECT_SOURCE_DIR}/../src/_micro-api/libraries/*/src/*.cpp
)


message("ARDUINO_LIBRARY_SOURCE_FILES: ${ARDUINO_LIBRARY_SOURCE_FILES}")
message("ARDUINO_LIBRARY_SOURCE_DIRS: ${ARDUINO_LIBRARY_SOURCE_DIRS}")
message("ARDUINO_MOCK_SOURCE_DIR: ${arduino_mock_SOURCE_DIR}")



# Adding the software and main test file as an executable
add_executable(TestProject
  ${ARDUINO_LIBRARY_SOURCE_FILES}
  ${PROJECT_SOURCE_DIR}/main.cpp
)
# Link all libraries
target_link_libraries(TestProject
    gtest_main
    arduino_mock
    ${CMAKE_THREAD_LIBS_INIT}
)

target_include_directories(TestProject PUBLIC 
    ${arduino_mock_SOURCE_DIR}/include
    ${googletest_SOURCE_DIR}/googletest/include
    ${googletest_SOURCE_DIR}/googlemock/include
)

include(GoogleTest)

# (Optional) Setting up the compiler
#target_compile_features(TestProject PUBLIC cxx_std_11)
#set_target_properties(TestProject PROPERTIES CXX_EXTENSIONS OFF)


include_directories(
    ${googletest_SOURCE_DIR}/googletest/include
    ${googletest_SOURCE_DIR}/googlemock/include
    ${arduino_mock_SOURCE_DIR}/include
    ../src/
    ../src/_micro-api/libraries/output/src/
    ../src/_micro-api/libraries/output/src/
    ../src/_micro-api/libraries/bitstore/
    ../src/_micro-api/libraries/fastdelegate/src/
    ../src/_micro-api/libraries/signalDecoder/src/
    ../src/_micro-api/libraries/bitstore/src/
    ../src/_micro-api/libraries/fastdelegate/src/
    ../src/_micro-api/libraries/signalDecoder/src/
    ../src/_micro-api/libraries/SimpleFIFO/src/
)

enable_testing()
gtest_discover_tests(TestProject)
#add_test(testSamesign TestProject)