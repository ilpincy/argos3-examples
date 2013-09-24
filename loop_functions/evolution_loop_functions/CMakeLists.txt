link_directories(${CMAKE_BINARY_DIR}/controllers/footbot_nn)

add_library(evolution_loop_functions SHARED evolution_loop_functions.cpp)

target_link_libraries(evolution_loop_functions
  footbot_nn
  argos3core_simulator
  argos3plugin_simulator_actuators
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_media
  argos3plugin_simulator_sensors
  ${GALIB_LIBRARIES})
