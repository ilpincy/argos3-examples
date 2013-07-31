include_directories(${CMAKE_SOURCE_DIR})

# Descend into the foraging_loop_functions directory
add_subdirectory(foraging_loop_functions)

# If GALIB was found, also descend into evolution_loop_functions
if(GALIB_FOUND)
  add_subdirectory(evolution_loop_functions)
endif(GALIB_FOUND)
