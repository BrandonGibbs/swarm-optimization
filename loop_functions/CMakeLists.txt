link_directories(${CMAKE_BINARY_DIR}/controllers/pathfindbot_controller)
set(flocking_loop_functions_SOURCES flocking_loop_functions.cpp)

if(ARGOS_QTOPENGL_FOUND)
  include_directories(${ARGOS_QTOPENGL_INCLUDE_DIRS})
  set(flocking_loop_functions_SOURCES
    ${flocking_loop_functions_SOURCES}
    flocking_qt_user_functions.cpp)
endif(ARGOS_QTOPENGL_FOUND)

add_library(flocking_loop_functions MODULE ${flocking_loop_functions_SOURCES})
target_link_libraries(flocking_loop_functions
  pathfindbot_controller
  argos3core_simulator
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_media)

if(ARGOS_QTOPENGL_FOUND)
  target_link_libraries(flocking_loop_functions argos3plugin_simulator_qtopengl)
endif(ARGOS_QTOPENGL_FOUND)
