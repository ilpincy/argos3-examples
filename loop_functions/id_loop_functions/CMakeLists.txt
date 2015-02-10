add_library(id_loop_functions MODULE 
  id_qtuser_functions.h
  id_qtuser_functions.cpp)

target_link_libraries(id_loop_functions
  argos3core_simulator
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_qtopengl
  ${QT_LIBRARIES} ${GLUT_LIBRARY} ${OPENGL_LIBRARY})
