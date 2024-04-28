#ifndef FLOCKING_LOOP_FUNCTIONS_H
#define FLOCKING_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>
#include <set>
using namespace argos;
using namespace std;

class CFlockingLoopFunctions : public CLoopFunctions {

public:

   CFlockingLoopFunctions();
   virtual ~CFlockingLoopFunctions() {}

   virtual void Init(TConfigurationNode& t_tree);
   virtual void Reset();
   virtual void Destroy();
   /*
   virtual CColor GetFloorColor(const CVector2& c_position_on_plane);
   virtual void IncreaseTargetDistance();
   virtual UInt32 GetTargetDistance();
   */
   virtual void PreStep();
   /*
   virtual Real getSimTimeInSeconds();
   virtual void CheckConnectivity();
   virtual map<string, CVector2> GetRobotPosition();

	size_t DrawIDs;

private:

   Real m_fFoodSquareRadius;
   CRange<Real> m_cFlockingArenaSideX, m_cFlockingArenaSideY;
   std::vector<CVector2> m_cFoodPos;
   CFloorEntity* m_pcFloor;
   CRandom::CRNG* m_pcRNG;

   std::string m_strOutput;
   std::ofstream m_cOutput;
   
    
   Real curr_time_in_seconds; 
   Real last_time_in_seconds; 
   
   map<string, CVector2> robotPos;
   map<string, vector<string>> connectedList;

   //UInt32 m_unCollectedFood;
   //SInt64 m_nEnergy;
   //UInt32 m_unEnergyPerFoodItem;
   //UInt32 m_unEnergyPerWalkingRobot;
   UInt32 target_distance; 
   */
};

#endif
