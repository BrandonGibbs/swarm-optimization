cmake_minimum_required(VERSION 3.8.0)
project(swarm_optimization)

set(CMAKE_BUILD_TYPE "Release") # we'll need to add debug build eventually

#set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_CXX_EXTENSIONS OFF)
#set(OpenGL_GL_PREFERENCE "GLVND")

find_package(ARGoS REQUIRED)
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS})
link_directories(${ARGOS_LIBRARY_DIR})
link_libraries(${ARGOS_LDFLAGS})

add_subdirectory(controllers)
