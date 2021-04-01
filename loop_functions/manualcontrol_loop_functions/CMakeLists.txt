include_directories(${ARGOS_QTOPENGL_INCLUDE_DIRS})

add_library(manualcontrol_loop_functions MODULE 
  manualcontrol_qtuser_functions.h
  manualcontrol_qtuser_functions.cpp)

target_link_libraries(manualcontrol_loop_functions
  footbot_manualcontrol
  argos3core_simulator
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_qtopengl
  ${ARGOS_QTOPENGL_LIBRARIES})
