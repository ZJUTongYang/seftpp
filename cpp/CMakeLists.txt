cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(seftpp)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -fPIC")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -g")

set(CMAKE_BUILD_TYPE "Release")

###########################################################################
# Find OpenCV
find_package(OpenCV REQUIRED)


include_directories(${OpenCV_INCLUDE_DIR})
link_directories(${OpenCV_LIBRARY_DIRS})
message("OpenCV Libraries: ${OpenCV_LIBS}")



include_directories(./include)

add_library(seftpp_lib ./src/tool.cpp ./src/solver.cpp ./src/planner.cpp ./src/map.cpp ./src/dijkstra_4dir.cpp)
target_link_libraries(seftpp_lib ${OpenCV_LIBS})

add_executable(seftpp_main ./src/main.cpp )
target_link_libraries(seftpp_main seftpp_lib ${OpenCV_LIBS})


