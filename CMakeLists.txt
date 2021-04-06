cmake_minimum_required(VERSION 3.8.0)
project(argos3_examples)

# Set the compilation type
if(CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel ..." FORCE)
endif(CMAKE_BUILD_TYPE STREQUAL "")

# Deactivate RPATH for MacOSX
set(CMAKE_MACOSX_RPATH 0)

# Set the path where CMake will find additional scripts
set(CMAKE_MODULE_PATH $ENV{CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)
set(CMAKE_PREFIX_PATH $ENV{CMAKE_PREFIX_PATH} ${CMAKE_SOURCE_DIR}/cmake)

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find the ARGoS package, make sure to save the ARGoS prefix
find_package(ARGOS REQUIRED COMPONENTS footbot OPTIONAL_COMPONENTS kheperaiv)

find_package(PkgConfig REQUIRED)
pkg_check_modules(ARGOS REQUIRED argos3_simulator)
# message(STATUS "ARGOS_LIBRARIES = ${ARGOS_LIBRARIES}")
# message(STATUS "ARGOS_LINK_LIBRARIES = ${ARGOS_LINK_LIBRARIES}")
# message(STATUS "ARGOS_LIBRARY_DIRS = ${ARGOS_LIBRARY_DIRS}")
# message(STATUS "ARGOS_LDFLAGS = ${ARGOS_LDFLAGS}")
# message(STATUS "ARGOS_LDFLAGS_OTHER = ${ARGOS_LDFLAGS_OTHER}")
set(ARGOS_PREFIX ${ARGOS_PREFIX} CACHE INTERNAL "ARGoS installation prefix")
set(CMAKE_MODULE_PATH $ENV{CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake ${ARGOS_PREFIX}/share/argos3/cmake)
set(CMAKE_PREFIX_PATH $ENV{CMAKE_PREFIX_PATH} ${CMAKE_SOURCE_DIR}/cmake ${ARGOS_PREFIX}/share/argos3/cmake)
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS})
link_libraries(${ARGOS_LDFLAGS})

# Check whether all the necessary libs have been installed to compile the
# code that depends on Qt and OpenGL
find_package(ARGoSQTOpenGL)
if(ARGOS_QTOPENGL_FOUND)
  include_directories(${ARGOS_QTOPENGL_INCLUDE_DIRS})
endif(ARGOS_QTOPENGL_FOUND)

include_directories(${CMAKE_SOURCE_DIR})

# Look for the (optional) galib library
find_package(GALIB)
if(GALIB_FOUND)
  include_directories(${GALIB_INCLUDE_DIRS})
endif(GALIB_FOUND)

# Descend into the controllers directory
add_subdirectory(controllers)

# Descend into the loop_functions directory
add_subdirectory(loop_functions)

# Descend into the embedding directory
add_subdirectory(embedding)
