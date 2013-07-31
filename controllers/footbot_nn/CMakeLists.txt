add_library(footbot_nn SHARED
  nn/neural_network.cpp
  nn/perceptron.cpp
  nn/ctrnn_multilayer.cpp
  footbot_nn_controller.cpp
)
target_link_libraries(footbot_nn argos2_simulator_dynamic_linking)