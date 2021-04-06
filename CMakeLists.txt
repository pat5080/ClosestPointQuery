cmake_minimum_required(VERSION 2.5)

project(closest_point_on_mesh LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories("${CMAKE_SOURCE_DIR}/source")
include_directories("${CMAKE_SOURCE_DIR}/include")
include_directories("${CMAKE_SOURCE_DIR}/thirdparty")
include_directories("${CMAKE_SOURCE_DIR}/test")

#add_subdirectory(test)

add_executable(project_sources
    source/main.cpp
    source/closestpointquery.cpp
    source/mesh.cpp
    source/tinyply.cpp
    )

target_link_libraries(project_sources)





