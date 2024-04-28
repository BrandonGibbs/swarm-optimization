add_library(pathfindbot_controller SHARED
  pathfindbot-controller.h pathfindbot-controller.cpp
)
target_link_libraries(pathfindbot_controller
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
)
