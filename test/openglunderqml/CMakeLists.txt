cmake_minimum_required(VERSION 3.13)

find_package(Qt5 COMPONENTS Quick REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

add_executable(openglunderqml squircle.cpp main.cpp)
target_link_libraries(openglunderqml Qt5::Quick)
