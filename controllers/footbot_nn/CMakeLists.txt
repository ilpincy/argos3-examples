add_library(footbot_nn SHARED
  nn/neural_network.cpp
  nn/perceptron.cpp
  nn/ctrnn_multilayer.cpp
  footbot_nn_controller.cpp
)
target_link_libraries(footbot_nn
  argos3core_simulator
  argos3plugin_simulator_actuators
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_sensors)
