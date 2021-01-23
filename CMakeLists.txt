cmake_minimum_required (VERSION 3.0)
project (rrt)
 

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

add_executable (main main.cpp)
add_library(geometry geometry.cpp)
add_library(rrt rrt.cpp)
target_link_libraries(rrt geometry)
target_link_libraries(main rrt ${PYTHON_LIBRARIES})
