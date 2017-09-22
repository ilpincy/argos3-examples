cmake_minimum_required(VERSION 2.8.12)
project(argos3_examples)

if(APPLE)
  option(ASAN "Use Clang Address Sanitizer in Debug builds")
  if(ASAN)
    # Add address sanitizer support for CLang
    set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -fsanitize=address -fno-optimize-sibling-calls -fno-omit-frame-pointer")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=address -fno-optimize-sibling-calls -fno-omit-frame-pointer")
    set(CMAKE_MODULE_LINKER_FLAGS_DEBUG "${CMAKE_MODULE_LINKER_FLAGS_DEBUG} -fsanitize=address")
    set(CMAKE_SHARED_LINKER_FLAGS_DEBUG "${CMAKE_MODULE_LINKER_FLAGS_DEBUG} -fsanitize=address")
    set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_MODULE_LINKER_FLAGS_DEBUG} -fsanitize=address")
  endif(ASAN)
endif(APPLE)

# Deactivate RPATH for MacOSX
set(CMAKE_MACOSX_RPATH 0)

# Set the path where CMake will find additional scripts
set(CMAKE_MODULE_PATH $ENV{CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)

# Find the ARGoS package, make sure to save the ARGoS prefix
find_package(PkgConfig)
pkg_check_modules(ARGOS REQUIRED argos3_simulator)
set(ARGOS_PREFIX ${ARGOS_PREFIX} CACHE INTERNAL "")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ARGOS_PREFIX}/share/argos3/cmake)

# Check whether all the necessary libs have been installed to compile the
# code that depends on Qt and OpenGL
include(ARGoSCheckQTOpenGL)

# Look for the (optional) galib library
find_package(GALIB)
if(GALIB_FOUND)
  include_directories(${GALIB_INCLUDE_DIRS})
endif(GALIB_FOUND)

# Find Lua
find_package(Lua52 REQUIRED)

# Set ARGoS include dir
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS} ${LUA_INCLUDE_DIR})

# Set ARGoS link dir
link_directories(${ARGOS_LIBRARY_DIRS})

# Descend into the controllers directory
add_subdirectory(controllers)

# Descend into the loop_functions directory
add_subdirectory(loop_functions)

# Descend into the embedding directory
add_subdirectory(embedding)
