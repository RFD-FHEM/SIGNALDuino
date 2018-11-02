cmake_minimum_required(VERSION 3.4.3 FATAL_ERROR)
project(TestProject)
message("PROJECT_SOURCE_DIR: ${PROJECT_SOURCE_DIR}")
# Set the output folder where your program will be created
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(   LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_CXX_STANDARD 11)

##############################################################################################################################################
# Dependencies
##############################################################################################################################################
find_package(GTest REQUIRED)
find_package(rapidassist 0.4.0 REQUIRED)
find_package(win32arduino 2.3.0 REQUIRED)


##############################################################################################################################################
# Project settings
##############################################################################################################################################

# Prevents annoying warnings on MSVC
if (WIN32)
  add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif()

# Find all library source and unit test files
file( GLOB_RECURSE ARDUINO_LIBRARY_SOURCE_FILES ${PROJECT_SOURCE_DIR}/../src/_micro-api/libraries/output/src/*.h ${PROJECT_SOURCE_DIR}/../src/_micro-api/libraries/output/src/*.cpp ${PROJECT_SOURCE_DIR}/../src/_micro-api/libraries/bitstore/*.cpp  ${PROJECT_SOURCE_DIR}/../src/_micro-api/libraries/fastdelegate/src/*.cpp ${PROJECT_SOURCE_DIR}/../src/_micro-api/libraries/signalDecoder/src/*.cpp  ${PROJECT_SOURCE_DIR}/../src/_micro-api/libraries/bitstore/src/*.h  ${PROJECT_SOURCE_DIR}/../src/_micro-api/libraries/fastdelegate/src/*.h ${PROJECT_SOURCE_DIR}/../src/_micro-api/libraries/signalDecoder/src/*.h 
 ${PROJECT_SOURCE_DIR}/../commands.h 
 ${PROJECT_SOURCE_DIR}/../functions.h 
 ${PROJECT_SOURCE_DIR}/../send.h)
file( GLOB ARDUINO_LIBRARY_TEST_FILES   ${PROJECT_SOURCE_DIR}/testSignalDecoder/tests.cpp  ${PROJECT_SOURCE_DIR}/testSignalDecoder/tests.h ${PROJECT_SOURCE_DIR}/testSignalDecoder/targetver.h )

# Create unit test executable
add_executable(TestProject
  ${ARDUINO_LIBRARY_SOURCE_FILES}
  ${ARDUINO_LIBRARY_TEST_FILES}
  ${PROJECT_SOURCE_DIR}/testSignalDecoder/main.cpp
)

include_directories(

)

#include directories
target_include_directories(TestProject PRIVATE
  #PRIVATE ${PROJECT_SOURCE_DIR}/testSignalDecoder       # Arduino Library folder
  #${ARDUINO_LIBRARY_SOURCE_FILES}
  ${GTEST_INCLUDE_DIR}
  win32arduino
  ${PROJECT_SOURCE_DIR}/../src/_micro-api/libraries/fastdelegate/src/
  ${PROJECT_SOURCE_DIR}/../src/_micro-api/libraries/output/src/
  ${PROJECT_SOURCE_DIR}/../src/_micro-api/libraries/bitstore/src/
  ${PROJECT_SOURCE_DIR}/../src/_micro-api/libraries/signalDecoder/src/
  ${PROJECT_SOURCE_DIR}/testSignalDecoder/
  ${PROJECT_SOURCE_DIR}/
  ${PROJECT_SOURCE_DIR}/../

)

# Unit test projects requires to link with pthread if also linking with gtest
if(NOT WIN32)
  set(PTHREAD_LIBRARIES -pthread)
endif()

#link libraries
target_link_libraries(TestProject PRIVATE win32arduino rapidassist ${PTHREAD_LIBRARIES} ${GTEST_LIBRARIES} )

