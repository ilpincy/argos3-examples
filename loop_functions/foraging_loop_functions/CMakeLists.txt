link_directories(${CMAKE_BINARY_DIR}/controllers/footbot_foraging)
set(foraging_loop_functions_SOURCES foraging_loop_functions.cpp)

if(ARGOS_COMPILE_QTOPENGL)
  set(foraging_loop_functions_SOURCES
    ${foraging_loop_functions_SOURCES}
    foraging_qt_user_functions.cpp)
endif(ARGOS_COMPILE_QTOPENGL)

add_library(foraging_loop_functions MODULE ${foraging_loop_functions_SOURCES})
target_link_libraries(foraging_loop_functions
  footbot_foraging
  argos3core_simulator
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_media)

if(ARGOS_COMPILE_QTOPENGL)
  target_link_libraries(foraging_loop_functions argos3plugin_simulator_qtopengl)
endif(ARGOS_COMPILE_QTOPENGL)
