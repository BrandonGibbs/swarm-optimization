/**
 * This robot is going to model its environment using a grid state space which
 * it will use to make decisions about what areas to explore.
 *
 * Once a single robot is able to arrive at the target, we can consider how to
 * get a team of robots to a target efficiently.
 */
#ifndef CPATHFINDBOT_CONTROLLER_H
#define CPATHFINDBOT_CONTROLLER_H
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>

#include "GridModel.h"

using namespace argos;

class CPathFindBot : public CCI_Controller {

public:
	CPathFindBot();
	virtual ~CPathFindBot(){}
	virtual void Init(TConfigurationNode& t_node);
	virtual void ControlStep();
	virtual void Reset();
	virtual void Destroy();
	
	/**
	 * When the robot has reached a dead end, a state with no new paths 
	 * available, it will have to go back to a position it has already 
	 * explored to look for a new path. This will happen when an item 
	 * popped from the priority queue says to go back to a state which is 
	 * not near the robot.
	 *
	 * To retrace our steps, we will follow the parent pointers of the 
	 * preceding nodes in the Grid.
	 */
	//enum EExploreState {
	//	EXPLORING = 0,
	//	BACKTRACKING
	//};
	
	struct SMovementParams {
		enum EMovementState {
			STATIONARY = 0,
			ROTATING,
			MOVING_FORWARD
		} m_eMovementState;
		
		/**
		 * These will remain constant but I haven't figured out how to compile 
		 * with const keyword
		 */
		double m_dMaxVelocity,
		       m_dMaxRotnSpeed,
		       m_dSimSecPerTick; // time delta for velocity calculations
		
		double m_dLeftWheelSpeed, 
		       m_dRightWheelSpeed;
			   
		/**
		 * the amount of time steps we'll be going the max linear or angular 
		 * velocity
		 */
		int    m_nTimes;
		/**
		 * the remaining linear or angular displacement we need to travel to 
		 * arrive at destination
		 */
		double m_dRemainDisp;
		
		
		void Init (TConfigurationNode& t_node);
	};

private:
	/**
	 * robot's hardware
	 */
	CCI_DifferentialSteeringActuator* m_pcWheels;
	CCI_FootBotProximitySensor      * m_pcProximity;
	CCI_RangeAndBearingActuator     * m_pcCommsTransmr;
	CCI_RangeAndBearingSensor       * m_pcCommsRecvr;
	CCI_PositioningSensor           * m_pcPosSensor;

	/**
	 * This is the global target which if the robot reaches, the simulation 
	 * terminates
	 */
	CVector2 m_cTarget;
	
	/**
	 * An instance of the struct defined above. An explanation of how the state
	 * machine works is in ControlStep.
	 */
	SMovementParams m_sMovement;
	
	/**
	 * the robot needs to provide the previous position to the grid so that it
	 * can know where it came from and the grid will return the next local 
	 * target
	 */
	CVector2 m_cPrevPos;
	CVector2 m_cLocalTarget;
	
	/**
	 * the grid will store the states the robot has traversed and will also
	 * tell it which state to explore next
	 */
	Grid m_cGrid;
};


#endif