link_directories(
  ${CMAKE_BUILD_DIR}/build/loop_functions/evolution_loop_functions
  ${CMAKE_BINARY_DIR}/controllers/footbot_nn
)
add_executable(argos_galib main.cpp)

target_link_libraries(argos_galib
  ${GALIB_LIBRARIES}
  footbot_nn
  evolution_loop_functions
  argos3core_simulator)

