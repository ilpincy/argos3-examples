include_directories(${CMAKE_SOURCE_DIR}/controllers)

add_subdirectory(footbot_diffusion)
add_subdirectory(footbot_flocking)
add_subdirectory(footbot_foraging)
add_subdirectory(footbot_synchronization)

# Compile the neural network controllers only if GALIB was found
if(GALIB_FOUND)
  add_subdirectory(footbot_nn)
endif(GALIB_FOUND)
