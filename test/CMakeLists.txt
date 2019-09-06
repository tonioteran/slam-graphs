CMAKE_MINIMUM_REQUIRED(VERSION 3.10.1)

project(slam-graphs)

find_package(Qt5 COMPONENTS Widgets REQUIRED)

add_executable(helloworld main.cpp)

target_link_libraries(helloworld Qt5::Widgets)
