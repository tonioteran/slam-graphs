# Simple DubinsCar test.
add_executable(helloDubinsCar helloDubinsCar.cpp)
target_link_libraries(helloDubinsCar PRIVATE DubinsCar)
target_include_directories(helloDubinsCar
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Matplotlib plotting tools
add_executable(plotDubinsPath plotDubinsPath.cpp)
target_link_libraries(plotDubinsPath
  PRIVATE
    DubinsCar
    ${PYTHON_LIBRARIES})
target_include_directories(plotDubinsPath
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
  PRIVATE ${PYTHON_INCLUDE_DIRS})
