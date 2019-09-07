cmake_minimum_required(VERSION 3.10)
project(slam-graphs VERSION 0.0.0 LANGUAGES CXX)

set(DEFAULT_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 14)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${DEFAULT_BUILD_TYPE}'.")
  set(CMAKE_BUILD_TYPE "${DEFAULT_BUILD_TYPE}")
endif()

include(GNUInstallDirs)

find_package(GTSAM REQUIRED)
find_package(Eigen3 3.3 REQUIRED)

# Dubins Car library
add_library(DubinsCar
  src/DubinsCar.cpp)
target_include_directories(DubinsCar
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE src)
target_link_libraries(DubinsCar
  PUBLIC
    gtsam
    Eigen3::Eigen)
set_target_properties(DubinsCar PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION 1)
target_compile_options(DubinsCar PRIVATE -W -Wall -Wextra -Wpedantic)
target_compile_features(DubinsCar PRIVATE cxx_std_14)



