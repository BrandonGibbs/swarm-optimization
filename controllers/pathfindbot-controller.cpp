#include "pathfindbot-controller.h"

#include <cmath> /* INFINITY, fmod */
#include <vector> /* vector<CRange<CRadians>> are used in GetAvailablePaths */
#include <argos3/core/simulator/simulator.h> /* CSimulator::Terminate, CSimulator::GetInstance */
#include <argos3/core/simulator/physics_engine/physics_engine.h>  /* CPhysicsEngine::GetSimulationClockTick */
#include <argos3/core/utility/logging/argos_log.h> /* LOG */

/**
 * This value is needed to calculate angular velocity. I got it from here:
 *   argos3/src/plugins/robots/foot-bot/simulator/dynamics2d_footbot_model.cpp
 */
const double FOOTBOT_INTERWHEEL_DIST = 0.14;

CPathFindBot::CPathFindBot() :
	m_pcWheels(NULL),
	m_pcProximity(NULL),
	m_pcCommsTransmr(NULL),
	m_pcCommsRecvr(NULL),
	m_pcPosSensor(NULL),

	m_cTarget(7.0, 7.0),
	m_cPrevPos(0,0),
	m_cLocalTarget(CVector2 (0,0))
	{}
	
void CPathFindBot::SMovementParams::Init(TConfigurationNode& t_node){
	m_eState = STATIONARY;
	m_dMaxVelocity   = 0.2;               // 0.2 m/s
	m_dMaxRotnSpeed  = 2.857142857142857; // rad/s - both wheels rotating in opposite directions at 20 cm/s (163.7 degrees/s)
	
	GetNodeAttributeOrDefault(t_node, "maxvelocity", m_dMaxVelocity, m_dMaxVelocity);
	GetNodeAttributeOrDefault(t_node, "maxrotationvelocity", m_dMaxRotnSpeed, m_dMaxRotnSpeed);
	
	m_dSimSecPerTick = CPhysicsEngine::GetSimulationClockTick(); // time delta for velocity calculations
}

void CPathFindBot::Init(TConfigurationNode& t_node){
	m_pcWheels       = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
	m_pcProximity    = GetSensor  <CCI_FootBotProximitySensor>      ("footbot_proximity");
	m_pcCommsTransmr = GetActuator<CCI_RangeAndBearingActuator>     ("range_and_bearing");
	m_pcCommsRecvr   = GetSensor  <CCI_RangeAndBearingSensor>       ("range_and_bearing");
	m_pcPosSensor    = GetSensor  <CCI_PositioningSensor>           ("positioning");


	GetNodeAttributeOrDefault(t_node, "target", m_cTarget, m_cTarget);
	m_sMovement.Init(GetNode(t_node, "movementparams"));
	m_cGrid.setTarget(m_cTarget);
}

/**
 * Get a set of angle ranges where the robot will be able to travel.
 *
 * First we'll use the KISS method and disregard distances. If there's any 
 * reading, we'll treat it as a range where there's no path available.
 *
 * The sensor angles start at 7.5 degrees and the subsequent sensors are 
 * 15 degrees apart. In the constructor it says that angles are 
 * 'SignedNormalize'd, so they're in the range [-pi,pi].
 * Source: 
 *   argos3/src/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.cpp
 *
 * We'll get the index to the relevant sensors by truncating the angle to the
 * target to an integer in this list and check to see if there are readings in 
 * in that range.
 */
bool isPathReachable(
	const CCI_FootBotProximitySensor::TReadings tProxReads, 
	CRadians angleToTarget
){
	LOG << "angle to target: " << angleToTarget;
	angleToTarget.SignedNormalize();
	LOG << " -> " << angleToTarget;
	int angleInd = (int) angleToTarget.GetValue() / (CRadians::PI_OVER_SIX.GetValue()/4);
	double sumReads = 0;
	LOG << "-> angle index: " << angleInd << " ->";
	
	for (int i = angleInd - 1; i < angleInd + 3; i++){
		int nonNegInd = (i < 0)? 24 + i : i;
		LOG << " " << nonNegInd;
		sumReads += tProxReads[nonNegInd].Value;
	}
	LOG << " -> sum: " << sumReads << std::endl;
	return sumReads == 0;
}

void CPathFindBot::ControlStep(){

	const CCI_PositioningSensor::SReading & 
		sPosSensRead = m_pcPosSensor->GetReading();
	
    const CCI_FootBotProximitySensor::TReadings & 
		tProxSensReads = m_pcProximity->GetReadings();
	
	CVector2 currentPos (
		sPosSensRead.Position[0],
		sPosSensRead.Position[1]
	);

	CQuaternion orientation = sPosSensRead.Orientation;
	
	/**
	 * This value is 0 degrees when facing north, positive 90 when facing west,
	 * negative 90 when facing east, so confusing af
	 */
	CRadians angleAboutZ, y, x; // we only care about the angle about z
	orientation.ToEulerAngles(  // but this method requires y and x
		angleAboutZ,  // now our angles are in range [-pi, pi) radians
		y,            // or [-180, 180) degrees
		x             // instead of [0, 1)
	);
	
	//angleAboutZ.UnsignedNormalize(); // change range to [0, 2*PI)
	
	LOG << "Position: " << currentPos << "; Orientation: " 
	    << angleAboutZ << std::endl;
	
	/**
	 * This rudimentary state machine allows the robot to get to a target with
	 * no barriers on the way. Here's the state-transition table:
	 *
	 *     current state  | next state     | condition
	 *     ------------------------------------------------------------------
	 *     STATIONARY     | ROTATING       | when angle delta > 2 degrees
	 *     ------------------------------------------------------------------
	 *     STATIONARY     | MOVING_FORWARD | when distance delta > 8 cm
	 *     ------------------------------------------------------------------
	 *     ROTATING       | STATIONARY     | when (m_nTimes * max velocity) 
	 *                    |                | distance has been travelled (as 
	 *                    |                | well as the remaining distance)
	 *     ------------------------------------------------------------------
	 *     MOVING_FORWARD | STATIONARY     | same as this^ except with linear
	 *                    |                | distance/velocity instead of 
	 *                    |                | angular
	 *
	 * Since the maximum velocity of footbots is 20 cm/s,
	 * (source: https://www.argos-sim.info/forum/viewtopic.php?p=597&hilit=footbot+max+velocity#p597)
	 * I decided to divide the distance to the target by the maximum velocity 
	 * and have the footbot travel at that speed until it needs to slow down
	 * to cover the remaining distance. Because the distance is a double, I 
	 * used fmod instead of the '%' operator to get the remainder. I used the 
	 * same idea to calculate the angular velocity.
	 *
	 * Note: The linear and angular displacement travelled are necessarily 
	 * underestimates since we're ignoring acceleration/forces. We can ignore 
	 * this issue since footbots don't have that much mass, but we can burn 
	 * that bridge when we get to it.
	 */
	LOG << "Current State: ";
	switch (m_sMovement.m_eState){
		case m_sMovement.STATIONARY: {
			// then we're deciding what to do
			LOG << "STATIONARY" << std::endl;
			m_sMovement.m_dLeftWheelSpeed = 0;
			m_sMovement.m_dRightWheelSpeed = 0;
			
			if (!m_cGrid.contains(currentPos)){
				// if transitioned grid states, insert new grid state into grid
				// and pop new target from PQ
				m_cLocalTarget = 
					m_cGrid.insert(currentPos, m_cPrevPos, 1); // cost will probably 
															   // be time or distance
				LOG << "Transitioned states. New target state: " 
					<< m_cLocalTarget << std::endl;
			}
			
			bool verified = false;
			CVector2 vecToTarget;
			CRadians diffVecAngle; // this is the angle which determines which 
			                       // direction we will rotate depending on the
			                       // sign
			
			do {
				if (verified){
					LOG << "discarded local target: " << m_cLocalTarget << std::endl;
					// forget last target popped from PQ, get another
					m_cLocalTarget = 
						m_cGrid.getNextState();
					LOG << "new local target: " << m_cLocalTarget << std::endl;
				}
				
				vecToTarget = m_cLocalTarget - currentPos;
				diffVecAngle = angleAboutZ - vecToTarget.Angle();
			
				verified = true;
				
				// check prox sensors to see if we can get to m_cLocalTarget
			} while (!isPathReachable(tProxSensReads, diffVecAngle));
			
			m_sMovement.m_nTimes      = (int)(diffVecAngle.GetValue() / m_sMovement.m_dMaxRotnSpeed);
			m_sMovement.m_dRemainDisp = fmod (diffVecAngle.GetValue(),  m_sMovement.m_dMaxRotnSpeed);
			
			if (m_sMovement.m_nTimes == 0 && Abs(m_sMovement.m_dRemainDisp) < (CRadians::PI / 90).GetValue()){
				// our angle is close enough, now we check if we need to move forward
				m_sMovement.m_nTimes      = (int) (vecToTarget.Length() / m_sMovement.m_dMaxVelocity);
				m_sMovement.m_dRemainDisp = fmod (vecToTarget.Length(),  m_sMovement.m_dMaxVelocity);
				
				if (m_sMovement.m_nTimes == 0 && Abs(m_sMovement.m_dRemainDisp) < 0.08){
					LOG << "Arrived at local target?" << std::endl;
				} else {
					m_sMovement.m_eState = m_sMovement.MOVING_FORWARD;
				}
			} else {
				m_sMovement.m_eState = m_sMovement.ROTATING;
			}
			break;
		} case m_sMovement.ROTATING: {
			LOG << "ROTATING" << std::endl;
			if (m_sMovement.m_nTimes != 0){
				// then the velocity will be the max for this time step
				// I just rearranged this equation which is used below to calculate
				// angular velocity:
				// angVel = (m_dRightWheelSpeed - m_dLeftWheelSpeed) / FOOTBOT_INTERWHEEL_DIST
				double wheelVelDiff = m_sMovement.m_dMaxRotnSpeed * FOOTBOT_INTERWHEEL_DIST;
				double wheelVelocity = wheelVelDiff / 2;
				if (m_sMovement.m_nTimes > 0){
					m_sMovement.m_dLeftWheelSpeed  =  wheelVelocity;
					m_sMovement.m_dRightWheelSpeed = -wheelVelocity;
					m_sMovement.m_nTimes -= 1;
				} else if (m_sMovement.m_nTimes < 0){
					m_sMovement.m_dLeftWheelSpeed  = -wheelVelocity;
					m_sMovement.m_dRightWheelSpeed =  wheelVelocity;
					m_sMovement.m_nTimes += 1;
				}
			} else {
				// the velocity needs to be less than the max, so calculate the
				// velocity needed for the last time step before changing 
				// states
				double angVel = m_sMovement.m_dRemainDisp / m_sMovement.m_dSimSecPerTick;
				double wheelVelDiff = angVel * FOOTBOT_INTERWHEEL_DIST;
				double wheelVelocity = wheelVelDiff / 2;
				m_sMovement.m_dLeftWheelSpeed  =  wheelVelocity;
				m_sMovement.m_dRightWheelSpeed = -wheelVelocity;
				
				m_sMovement.m_eState = m_sMovement.STATIONARY;
			}
			break;
		} case m_sMovement.MOVING_FORWARD: {
			LOG << "MOVING_FORWARD" << std::endl;
			if (m_sMovement.m_nTimes > 0){
				// go the max velocity
				m_sMovement.m_dLeftWheelSpeed = m_sMovement.m_dRightWheelSpeed = m_sMovement.m_dMaxVelocity;
				m_sMovement.m_nTimes -= 1;
			} else {
				// calculate velocity needed to cover the remaining 
				// displacement/distance before changing states
				double velocity = m_sMovement.m_dRemainDisp;// / m_dSimSecPerTick;
				m_sMovement.m_dRightWheelSpeed = m_sMovement.m_dLeftWheelSpeed = velocity;
				m_sMovement.m_eState = m_sMovement.STATIONARY;
			}
			break;
		}
	}
	
	m_pcWheels->SetLinearVelocity(
		100 * m_sMovement.m_dLeftWheelSpeed,  // convert from m/s to cm/s
		100 * m_sMovement.m_dRightWheelSpeed
	);
	
	/**
	 * These calculations are from here:
	 *   argos3/src/plugins/simulator/physics_engines/dynamics2d/dynamics2d_differentialsteering_control.cpp
	 */
	double angVel = (m_sMovement.m_dRightWheelSpeed - m_sMovement.m_dLeftWheelSpeed) / FOOTBOT_INTERWHEEL_DIST;
	double temp   = (m_sMovement.m_dRightWheelSpeed + m_sMovement.m_dLeftWheelSpeed) / 2;
	CVector2 linVel (temp * ARGOS_COS(angleAboutZ.GetValue()), temp * ARGOS_SIN(angleAboutZ.GetValue()));
	
	LOG << "linear speed = " << linVel.Length() << " m/s, angular speed = " << angVel << " rad/s" << std::endl;
	LOG << "______________________________" << std::endl;
	
	m_cPrevPos = currentPos;
	
	if ((m_cTarget - currentPos).Length() < 0.1){
		CSimulator::GetInstance().Terminate();
		LOG << "Arrived at target. Simulation terminated." << std::endl;
	}
}

void CPathFindBot::Reset(){}

void CPathFindBot::Destroy(){}

REGISTER_CONTROLLER(CPathFindBot, "pathfindbot")