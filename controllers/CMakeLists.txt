add_library(lubot_controller MODULE 
  lubot-controller.h lubot-controller.cpp
)
add_library(pathfindbot_controller MODULE 
  pathfindbot-controller.h pathfindbot-controller.cpp
  GridModel.h GridModel.cpp
  PQueue.h
)
target_link_libraries(lubot_controller
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
)
