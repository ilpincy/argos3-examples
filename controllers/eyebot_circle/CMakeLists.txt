add_library(eyebot_circle MODULE eyebot_circle.h eyebot_circle.cpp footbot_circle.h footbot_circle.cpp)
target_link_libraries(eyebot_circle
  argos3core_simulator
  argos3plugin_simulator_eyebot
  argos3plugin_simulator_genericrobot)
