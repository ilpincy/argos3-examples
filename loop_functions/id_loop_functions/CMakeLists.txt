include_directories(${ARGOS_QTOPENGL_INCLUDE_DIRS})
add_library(id_loop_functions MODULE 
  id_qtuser_functions.h
  id_qtuser_functions.cpp)

target_link_libraries(id_loop_functions
  argos3core_simulator
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_qtopengl
  ${ARGOS_QTOPENGL_LIBRARIES})
