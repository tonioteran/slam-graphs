# Simple DubinsCar test.
add_executable(helloDubinsCar helloDubinsCar.cpp)
target_link_libraries(helloDubinsCar PRIVATE DubinsCar)
target_include_directories(helloDubinsCar
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
