cmake_minimum_required(VERSION 3.24)

project(TestProject)

message("PROJECT_SOURCE_DIR: ${PROJECT_SOURCE_DIR}")
# Set the output folder where your program will be created
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(   LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_CXX_STANDARD 11)

cmake_policy(SET CMP0074 NEW)

if (POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()
enable_testing()

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

#include(ExternalProject)
#set_property(DIRECTORY PROPERTY EP_STEP_TARGETS build)

include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY      https://github.com/google/googletest.git
  GIT_TAG             release-1.8.1
  FIND_PACKAGE_ARGS   NAMES GTest 
  #OVERRIDE_FIND_PACKAGE
)
FetchContent_Declare(
  rapidassist
  GIT_REPOSITORY      https://github.com/sidey79/RapidAssist.git
  GIT_TAG             master
  FIND_PACKAGE_ARGS   NAMES rapidassist
)
FetchContent_Declare(
    win32arduino
    GIT_REPOSITORY      https://github.com/sidey79/win32arduino.git
    GIT_TAG             master
    FIND_PACKAGE_ARGS   NAMES win32arduino
)

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
set(BUILD_GTEST ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

if(NOT EXISTS ${CMAKE_FIND_PACKAGE_REDIRECTS_DIR}/gtest-config.cmake AND
   NOT EXISTS ${CMAKE_FIND_PACKAGE_REDIRECTS_DIR}/GTestConfig.cmake)
  file(WRITE ${CMAKE_FIND_PACKAGE_REDIRECTS_DIR}/gtest-config.cmake
[=[
include(CMakeFindDependencyMacro)
find_dependency(googletest)
]=])
endif()

if(NOT EXISTS ${CMAKE_FIND_PACKAGE_REDIRECTS_DIR}/gtest-config-version.cmake AND
   NOT EXISTS ${CMAKE_FIND_PACKAGE_REDIRECTS_DIR}/GTestConfigVersion.cmake)
  file(WRITE ${CMAKE_FIND_PACKAGE_REDIRECTS_DIR}/gtest-config-version.cmake
[=[
include(${CMAKE_FIND_PACKAGE_REDIRECTS_DIR}/googletest-config-version.cmake OPTIONAL)
if(NOT PACKAGE_VERSION_COMPATIBLE)
  include(${CMAKE_FIND_PACKAGE_REDIRECTS_DIR}/googletestConfigVersion.cmake OPTIONAL)
endif()
]=])
endif()

#find_package(GTest REQUIRED)

FetchContent_MakeAvailable(rapidassist)
FetchContent_MakeAvailable(win32arduino)

message(STATUS "Looking for RAPIDASSIST_HEADER_FILES=" ${rapidassist_SOURCE_DIR})
message(STATUS "Looking for RAPIDASSIST_FOUND=" ${rapidassist_FOUND})
message(STATUS "Looking for RAPIDASSIST_DIR=" ${rapidassist_DIR})
message(STATUS "Looking for PROJECT_SOURCE_DIR=" ${PROJECT_SOURCE_DIR})
##############################################################################################################################################
# Code coverage
##############################################################################################################################################
#SET(COVERAGE OFF CACHE BOOL "Coverage")
#if (COVERAGE)
#    target_compile_options(TestProject PRIVATE --coverage)
#endif()

##############################################################################################################################################
# Dependencies
##############################################################################################################################################
#find_package(GTest REQUIRED)
#find_package(rapidassist 0.4.0 REQUIRED)
#find_package(win32arduino 2.3.0 REQUIRED)
find_package(win32arduino)


##############################################################################################################################################
# Project settings
##############################################################################################################################################

# Prevents annoying warnings on MSVC
#if (WIN32)
#  add_definitions(-D_CRT_SECURE_NO_WARNINGS)
#endif()

# Find all library source and unit test files
file( GLOB_RECURSE ARDUINO_LIBRARY_SOURCE_FILES ${PROJECT_SOURCE_DIR}/../src/_micro-api/libraries/output/src/*.h ${PROJECT_SOURCE_DIR}/../src/_micro-api/libraries/output/src/*.cpp ${PROJECT_SOURCE_DIR}/../src/_micro-api/libraries/bitstore/*.cpp  ${PROJECT_SOURCE_DIR}/../src/_micro-api/libraries/fastdelegate/src/*.cpp ${PROJECT_SOURCE_DIR}/../src/_micro-api/libraries/signalDecoder/src/*.cpp  ${PROJECT_SOURCE_DIR}/../src/_micro-api/libraries/bitstore/src/*.h  ${PROJECT_SOURCE_DIR}/../src/_micro-api/libraries/fastdelegate/src/*.h ${PROJECT_SOURCE_DIR}/../src/_micro-api/libraries/signalDecoder/src/*.h 
 ${PROJECT_SOURCE_DIR}/../src/commands.h 
 ${PROJECT_SOURCE_DIR}/../src/functions.h 
 ${PROJECT_SOURCE_DIR}/../src/send.h)
file( GLOB ARDUINO_LIBRARY_TEST_FILES   ${PROJECT_SOURCE_DIR}/testSignalDecoder/tests.cpp  ${PROJECT_SOURCE_DIR}/testSignalDecoder/tests.h ${PROJECT_SOURCE_DIR}/testSignalDecoder/targetver.h )

# Create unit test executable
add_executable(TestProject
  ${ARDUINO_LIBRARY_SOURCE_FILES}
  ${ARDUINO_LIBRARY_TEST_FILES}
  ${PROJECT_SOURCE_DIR}/testSignalDecoder/main.cpp
)

#include_directories(
# install/include
#)


#include directories
target_include_directories(TestProject PRIVATE
  ${CMAKE_PREFIX_PATH}
  #${RAPIDASSIST_HEADER_FILES}
  #PRIVATE ${PROJECT_SOURCE_DIR}/testSignalDecoder       # Arduino Library folder
  #${ARDUINO_LIBRARY_SOURCE_FILES}
  GTest::gtest GTest::gtest_main
  #win32arduino
  #rapidassist
  ${PROJECT_SOURCE_DIR}/../src/
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

# link libraries
if (COVERAGE)
 	target_link_libraries(TestProject PRIVATE win32arduino rapidassist ${PTHREAD_LIBRARIES} gtest gtest_main --coverage)
 else()
 	target_link_libraries(TestProject PRIVATE win32arduino rapidassist ${PTHREAD_LIBRARIES} gtest gtest_main)
 endif()