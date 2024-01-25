add_library(lubot_controller MODULE 
  lubot-controller.h lubot-controller.cpp
)
target_link_libraries(lubot_controller
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
)
