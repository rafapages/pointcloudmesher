cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(POINT_CLOUD_MESHER)

find_package(PCL 1.3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcMesher pcMesher.cpp camera.cpp main.cpp)
target_link_libraries(pcMesher ${PCL_LIBRARIES} freeimageplus)
