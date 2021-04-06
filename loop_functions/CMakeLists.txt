include_directories(${CMAKE_SOURCE_DIR})

# Descend into the foraging_loop_functions directory
add_subdirectory(foraging_loop_functions)

# Descend into the mpga_phototaxis_loop_functions directory
add_subdirectory(mpga_loop_functions)

# Descend into the custom_distributions_loop_functions directory
add_subdirectory(custom_distributions_loop_functions)

# If Qt+OpenGL dependencies were found, descend into these directories
if(ARGOS_QTOPENGL_FOUND)
  add_subdirectory(trajectory_loop_functions)
  add_subdirectory(id_loop_functions)
  add_subdirectory(manualcontrol_loop_functions)
endif(ARGOS_QTOPENGL_FOUND)

# If GALIB was found, also descend into galib_phototaxis_loop_functions
if(GALIB_FOUND)
  add_subdirectory(galib_phototaxis_loop_functions)
endif(GALIB_FOUND)
