#include "flocking_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <controllers/pathfindbot-controller.h>

CFlockingLoopFunctions::CFlockingLoopFunctions(){}
void CFlockingLoopFunctions::PreStep(){}
void CFlockingLoopFunctions::Init(TConfigurationNode& t_tree){}
void CFlockingLoopFunctions::Reset(){}
void CFlockingLoopFunctions::Destroy(){}

REGISTER_LOOP_FUNCTIONS(CFlockingLoopFunctions, "flocking_loop_functions")
