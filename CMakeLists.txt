cmake_minimum_required(VERSION 3.16)
project(mavlink_ground_station VERSION 1.0.0 LANGUAGES CXX)

option(MAVGS_BUILD_TESTS "Build the unit tests" ON)
option(MAVGS_BUILD_APPS "Build the mavgcs and mavsim executables" ON)
option(MAVGS_WERROR "Treat compiler warnings as errors" OFF)
option(MAVGS_SANITIZE "Enable AddressSanitizer and UBSan" OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

add_library(mavgs_options INTERFACE)
target_compile_options(mavgs_options INTERFACE -Wall -Wextra -Wpedantic -Wshadow -Wconversion)
if(MAVGS_WERROR)
  target_compile_options(mavgs_options INTERFACE -Werror)
endif()
if(MAVGS_SANITIZE)
  target_compile_options(mavgs_options INTERFACE -fsanitize=address,undefined -fno-omit-frame-pointer
                         -fno-sanitize-recover=undefined)
  target_link_options(mavgs_options INTERFACE -fsanitize=address,undefined)
endif()

add_library(mavgs STATIC
  src/frame.cpp
  src/messages.cpp
)
target_include_directories(mavgs PUBLIC include PRIVATE src)
target_link_libraries(mavgs PUBLIC mavgs_options)

if(MAVGS_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
