cmake_minimum_required(VERSION 3.13)

find_package(Qt5 COMPONENTS Core Widgets Gui Network OpenGL REQUIRED PATHS ~/Qt5.13.2)
message("Hello world: ${Qt5_FIND_REQUIRED}")
message(STATUS ${Qt5_Core_INCLUDE_DIRS})
message(STATUS ${Qt5_FOUND})
message(STATUS ${Qt5_FIND_REQUIRED})
find_package(OpenGL REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

set(boxes-src
  main.cpp
  scene.cpp
  glbuffers.cpp
  glextensions.cpp
  qtbox.cpp
  roundedbox.cpp
  /home/tonio/Qt5.13.2/5.13.2/gcc_64/include/QtCore/qrandom.h
  trackball.cpp)
add_executable(boxes ${boxes-src})
target_include_directories(boxes PUBLIC "/home/tonio/Qt5.13.2/5.13.2/gcc_64/include/QtCore")
target_link_libraries(boxes Qt5::Core Qt5::Widgets Qt5::Gui Qt5::OpenGL OpenGL)
