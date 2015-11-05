cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(POINT_CLOUD_MESHER)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

#set(FREEIMAGE_FIND_REQUIRED, TRUE)
find_package(FreeImagePlus REQUIRED)
include_directories(${FREEIMAGEPLUS_INCLUDE_DIRS})
link_directories(${FREEIMAGEPLUS_LIBRARY_DIRS})

find_package(PCL 1.3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(pcMesher pcMesher.cpp camera.cpp main.cpp)
target_link_libraries(pcMesher ${PCL_LIBRARIES} ${FREEIMAGEPLUS_LIBRARIES} boost_program_options)
