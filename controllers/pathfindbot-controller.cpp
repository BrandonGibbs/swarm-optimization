#include "pathfindbot-controller.h"

#include <cmath> /* INFINITY */
#include <cstring> /* memcpy */
#include <argos3/core/simulator/simulator.h> /* CSimulator::GetInstance */
#include <argos3/core/simulator/space/space.h> /* CSpace */
#include <argos3/core/utility/logging/argos_log.h> /* LOG */

#define AVG_WEIGHT      0.01  // used to calculate weighted average; must be in range [0, 1)
			      // making this smaller leads to the average converging more slowly

#define LOCAL_MIN_THRESH 0.1  // when our weighted average velocity toward the target reaches this value,
			      // we've decided we're in a local minimum

#define SPREAD_RATE     0.1   // the rate at which TargetDistance parameter is increased to spread robots out
#define CONTRACT_RATE   0.05  // the same as ^this but to contract as robots approach green LED

#define MIN_TARGET_DIST 75    // these will be our minimum and maximum TargetDistance values for expansion/contraction
#define MAX_TARGET_DIST 180

#define TARGET_DIST_DIFF 30   // when setting the TargetDistance of robots further from the lost neighbor, this value
			      // will be subtracted from the neighbor's

CPathFindBot::CPathFindBot() :
	m_pcWheels(NULL),
	m_pcProximity(NULL),
	m_pcPosSensor(NULL),
	m_pcLEDs(NULL),
	m_pcCamera(NULL),
	m_pcRABSensor(NULL),
	m_pcRABActuator(NULL),

	m_cTarget(9, 3)
	{}

/****************************************/
/****************************************/
	
void CPathFindBot::SWheelTurningParams::Init(TConfigurationNode& t_node) {
   try {
      TurningMechanism = NO_TURN;
      CDegrees cAngle;
      GetNodeAttribute(t_node, "hard_turn_angle_threshold", cAngle);
      HardTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "soft_turn_angle_threshold", cAngle);
      SoftTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "no_turn_angle_threshold", cAngle);
      NoTurnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "max_speed", MaxSpeed);
      
      GetNodeAttributeOrDefault(t_node, "alpha", m_cAlpha, m_cAlpha);
      m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
      GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
      GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);
   
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller wheel turning parameters.", ex);
   }
}

/****************************************/
/****************************************/

void CPathFindBot::SFlockingInteractionParams::Init(TConfigurationNode& t_node) {
   try {
      GetNodeAttribute(t_node, "target_distance", TargetDistance);
      GetNodeAttribute(t_node, "gain", Gain);
      GetNodeAttribute(t_node, "exponent", Exponent);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller flocking parameters.", ex);
   }
}

/****************************************/
/****************************************/

void CPathFindBot::Init(TConfigurationNode& t_node){
	m_pcWheels       = GetActuator <CCI_DifferentialSteeringActuator>("differential_steering");
	m_pcLEDs         = GetActuator <CCI_LEDsActuator>                ("leds");
	m_pcProximity    = GetSensor   <CCI_FootBotProximitySensor>      ("footbot_proximity");
	m_pcPosSensor    = GetSensor   <CCI_PositioningSensor>           ("positioning");
	m_pcCamera       = GetSensor   <CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
	m_pcRABSensor    = GetSensor   <CCI_RangeAndBearingSensor>       ("range_and_bearing");
	m_pcRABActuator  = GetActuator <CCI_RangeAndBearingActuator>     ("range_and_bearing");

	/*
	* Parse the config file
	*/
	try {
		/* Wheel turning */
		m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));
		/* Flocking-related */
		m_sFlockingParams.Init(GetNode(t_node, "flocking"));
	}
	catch(CARGoSException& ex) {
		THROW_ARGOSEXCEPTION_NESTED("Error parsing the controller parameters.", ex);
	}

	GetNodeAttributeOrDefault(t_node, "target", m_cTarget, m_cTarget);

	controllerID = std::stoi(GetId().substr(2, std::string::npos));
	Reset();
}

/****************************************/
/****************************************/

Real CPathFindBot::SFlockingInteractionParams::GeneralizedLennardJones(Real f_distance){
	Real fNormDistExp = ::pow (TargetDistance / f_distance, Exponent);
	return -Gain / f_distance * (fNormDistExp * fNormDistExp - fNormDistExp);
}

/****************************************/
/****************************************/

void CPathFindBot::ControlStep(){

	m_dWeightAvgSpeedTarget = updateWeightedAvgSpeed (m_dWeightAvgSpeedTarget);
	announceState();
	updateLOSNeighbors();
	m_pcLEDs->SetSingleColor(12, m_cLEDColor);

	/* Get readings from proximity sensor */
	const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
	/* Sum them together */
	CVector2 cAccumulator;

	for(size_t i = 0; i < tProxReads.size(); ++i) {
		cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
	}
	cAccumulator /= tProxReads.size();

	CRadians cAngle = cAccumulator.Angle();
	CVector2 targetVec = VectorToTarget(),
	         flockVec  = FlockingVector();

	if (m_cLEDColor == CColor::GREEN){
		targetVec = flockVec = CVector2(0, 0);
	} else if (m_cLEDColor == CColor::ORANGE){
		targetVec = CVector2 (0, 0);
	}

	if(m_sWheelTurningParams.m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
	   cAccumulator.Length() < m_sWheelTurningParams.m_fDelta ) {
		/* Go straight to the target location*/
		SetWheelSpeedsFromVector(targetVec + flockVec);
	} else {

		if (m_bReadyToBreakAway && m_cLEDColor == CColor::ORANGE){
			m_sFlockingParams.TargetDistance = MAX_TARGET_DIST - 20;
			m_cLEDColor = CColor::WHITE;
			LOG << "fb" << controllerID << ": orange -> white (change in angle W.R.T. green > pi & sensed barrier)" << std::endl;
		}
		/* Turn, depending on the sign of the angle */
		if(cAngle.GetValue() > 0.0f) {
			 m_pcWheels->SetLinearVelocity(m_sWheelTurningParams.m_fWheelVelocity, 0.0f);
		}
		else {
			m_pcWheels->SetLinearVelocity(0.0f, m_sWheelTurningParams.m_fWheelVelocity);
		}
	}
}

/****************************************/
/****************************************/

CVector2 CPathFindBot::VectorToTarget(){
	CVector2 c, currPos;
	//const CCI_PositioningSensor::SReadings& sReadings = m_compassSensor->GetReadings();
	argos::CVector3 position3D = m_pcPosSensor->GetReading().Position;
	const argos::CQuaternion orientation = m_pcPosSensor->GetReading().Orientation;
	/* convert the quaternion to euler angles */
	argos::CRadians z_angle, y_angle, x_angle;
	orientation.ToEulerAngles(z_angle, y_angle, x_angle);
	
	
	//LOG<<"angle="<< z_angle<< endl;
	float x = position3D.GetX();
	float y = position3D.GetY();
	currPos = CVector2(x, y);
	//LOG<<"robot ID="<< controllerID <<std::endl;
	////LOG<<"currPos = "<< currPos<<std::endl;
	//LOG<<"Target = "<< Target<<std::endl;
	//LOG<<"length = "<< (currPos - Target).Length()<<endl;
	
	if ((m_cTarget - currPos).Length() > 0.05){
		//argos::CRadians heading(orientation.GetW()); 
		//LOG<<"heading = "<< z_angle<<endl;
		//LOG<<"(currPos - Target).Angle()="<<(currPos - Target).Angle()<< endl;
		//LOG<<"(Target - currPos).Angle()="<<(Target - currPos).Angle()<< endl;
		
		//LOG<< "(currPos - Target).Angle()-heading="<< (currPos - Target).Angle()-z_angle << endl;
		//c = CVector2((Target - currPos).Length(), (Target - currPos).Angle()-z_angle);
		c = CVector2(1.0, (m_cTarget - currPos).Angle()-z_angle);
		
		c.Normalize();
		c *= 0.25f * m_sWheelTurningParams.MaxSpeed;
		//LOG << "c="<< c<<std::endl;
		}
		else c = CVector2(0, 0);
	//LOG << "VectorToTarget()="<< c<< std::endl;
    return c;
	
}

/****************************************/
/****************************************/

CVector2 CPathFindBot::FlockingVector() {
	
	CVector2 cAccum;
	Real fLJ;
	size_t unBlobsSeen  = 0;
	bool redBlobSeen    = false,
	     blueBlobSeen   = false,
	     orangeBlobSeen = false,
	     yellowBlobSeen = false,
	     greenBlobSeen  = false;

	const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings & sCameraReadings = m_pcCamera->GetReadings();

	for(CCI_ColoredBlobOmnidirectionalCameraSensor::TBlobList::const_iterator it = sCameraReadings.BlobList.begin();
	it != sCameraReadings.BlobList.end(); ++it){
		//if ((*it)->Distance < 1.8 * m_sFlockingParams.TargetDistance){
			if ((m_cLEDColor == CColor::BLUE || m_cLEDColor == CColor::YELLOW) && (*it)->Color == CColor::ORANGE){
				// if we see orange, we need to get that robot's id so we can determine our own TargetDistance
				m_bFollowingNeighbor = true;
				m_nNeighborToFollow = getLOSNeighborID ((*it)->Distance, (*it)->Angle);
				LOG << "fb" << controllerID << " now following fb" << m_nNeighborToFollow << std::endl;
			}


			if (m_cLEDColor == CColor::ORANGE && (*it)->Color == CColor::GREEN){
				// when we see green, we'll more than likely be very close, so we need to reduce its influence
				// so we don't shoot away too abruptly
				Real prevTD = m_sFlockingParams.TargetDistance;
				m_sFlockingParams.TargetDistance = MIN_TARGET_DIST;

				// add a slight push perpendicular to green LED's position to get us around the edge of the barrier
				CVector2 perpendicularVec;
				if ((*it)->Angle.GetValue() > 0){
					perpendicularVec = CVector2 ((*it)->Distance, (*it)->Angle - CRadians::PI_OVER_TWO);
				} else {
					perpendicularVec = CVector2 ((*it)->Distance, (*it)->Angle + CRadians::PI_OVER_TWO);
				}
				perpendicularVec.Normalize();
				perpendicularVec *= 0.1 * m_sWheelTurningParams.MaxSpeed;

				cAccum += perpendicularVec;

				// change force due to green LED to one which 
				//   - pulls when getting too far and 
				//   - pushes when too close
				fLJ = m_sFlockingParams.GeneralizedLennardJones ((*it)->Distance) 
				    + m_sFlockingParams.GeneralizedLennardJones ((*it)->Distance - 200);
				m_sFlockingParams.TargetDistance = prevTD;

				if (m_bSeenGreenLED && !m_bReadyToBreakAway){
					CVector3 position3D = m_pcPosSensor->GetReading().Position;
					CVector2 absPos (position3D.GetX(), position3D.GetY()),
						 greenAbsPos = getAbsolutePosition (
							CVector2 ((*it)->Distance / 100, (*it)->Angle)
						 );
					CRadians currentAngleWRTGreen = (greenAbsPos - absPos).Angle();

					if ((currentAngleWRTGreen - m_cAngleWRTGreenFirstSeen).GetAbsoluteValue() > CRadians::PI.GetValue()){
						m_bReadyToBreakAway = true;
					}
				}
			} else if (m_cLEDColor == CColor::ORANGE && (*it)->Color == CColor::WHITE){
				Real prevTD = m_sFlockingParams.TargetDistance;
				m_sFlockingParams.TargetDistance = MIN_TARGET_DIST;
				fLJ = m_sFlockingParams.GeneralizedLennardJones ((*it)->Distance);
				m_sFlockingParams.TargetDistance = prevTD;
			} else if (m_cLEDColor == CColor::WHITE && (*it)->Color != CColor::WHITE){
				Real prevTD = m_sFlockingParams.TargetDistance;
				m_sFlockingParams.TargetDistance = MAX_TARGET_DIST + 20;
				fLJ = m_sFlockingParams.GeneralizedLennardJones ((*it)->Distance);
				m_sFlockingParams.TargetDistance = prevTD;
			} else if (m_cLEDColor == CColor::GREEN && (*it)->Color == CColor::WHITE){
				UInt32 whiteRobotID = getLOSNeighborID ((*it)->Distance, (*it)->Angle);
				m_vWhiteLEDsSeen.insert (whiteRobotID);
			} else {
				fLJ = m_sFlockingParams.GeneralizedLennardJones ((*it)->Distance);
			}
			
			cAccum += CVector2 (fLJ, (*it)->Angle);
			unBlobsSeen ++;

			   redBlobSeen |= (*it)->Color == CColor::RED;
			  blueBlobSeen |= (*it)->Color == CColor::BLUE;
			orangeBlobSeen |= (*it)->Color == CColor::ORANGE;
			yellowBlobSeen |= (*it)->Color == CColor::YELLOW;
			 greenBlobSeen |= (*it)->Color == CColor::GREEN;

			
			if ((*it)->Color == CColor::GREEN && !m_bSeenGreenLED){
				CVector3 position3D = m_pcPosSensor->GetReading().Position;
				CVector2 absPos (position3D.GetX(), position3D.GetY());
				CVector2 greenAbsPos = getAbsolutePosition (
					CVector2 ((*it)->Distance / 100, (*it)->Angle)
				);
				m_cAngleWRTGreenFirstSeen = (greenAbsPos - absPos).Angle();
				m_bSeenGreenLED = true;
			}
		//}
	}

	if (m_cLEDColor == CColor::RED){

		if(m_dWeightAvgSpeedTarget < LOCAL_MIN_THRESH){
			// tell neighbors that we're in a local minimum
			m_cLEDColor = CColor::BLUE;
			LOG << "fb" << controllerID << ": red -> blue (local minimum detected)" << std::endl;
		}

		if (blueBlobSeen){
			// one of our neighbors senses that we're in a local minimum
			m_cLEDColor = CColor::BLUE;
			LOG << "fb" << controllerID << ": red -> blue (see blue neighbor) TD: " << m_sFlockingParams.TargetDistance << std::endl;
		}
	} else if (m_cLEDColor == CColor::BLUE){
		
		// spread out to find an escape from local minimum
		m_sFlockingParams.TargetDistance += SPREAD_RATE;

		if (yellowBlobSeen || m_sFlockingParams.TargetDistance >= MAX_TARGET_DIST){
			// stop expanding because we have reached the max communication range
			m_sFlockingParams.TargetDistance = MAX_TARGET_DIST; // set explicitly to prevent slight differences
			m_cLEDColor = CColor::YELLOW;
			LOG << "fb" << controllerID << ": blue -> yellow (reached max TD or see yellow LED) " 
			    << m_sFlockingParams.TargetDistance << std::endl;
		}

		if (m_bNeighborLeftSwarm){
			// let neighbors know one of our neighbors left the flock
			m_sFlockingParams.TargetDistance = MAX_TARGET_DIST;
			m_bFollowingNeighbor = false;
			m_cLEDColor = CColor::ORANGE;
			LOG << "fb" << controllerID << ": blue -> orange (neighbor left flock) TD: " << m_sFlockingParams.TargetDistance << std::endl;
		} else if (orangeBlobSeen){
			if (m_sFlockingParams.TargetDistance - TARGET_DIST_DIFF >= MIN_TARGET_DIST){
				m_sFlockingParams.TargetDistance = m_vLOSNeighbors [m_nNeighborToFollow].TargetDistance - TARGET_DIST_DIFF;
			}
			m_cLEDColor = CColor::ORANGE;
			LOG << "fb" << controllerID << ": blue -> orange (see orange neighbor fb" 
			    << m_nNeighborToFollow << ") TD: " << m_sFlockingParams.TargetDistance << std::endl;
		} else if (unBlobsSeen == 0){
			// we either escaped the local minimum or are no longer in the communication range of our neighbors
			m_nTotalNumRobots = CSimulator::GetInstance().GetSpace().GetEntitiesByType("foot-bot").size();
			m_cLEDColor = CColor::GREEN;
			LOG << "fb" << controllerID << ": blue -> green (no neighbors) TD: " << m_sFlockingParams.TargetDistance << std::endl;
		}
	} else if (m_cLEDColor == CColor::YELLOW){

		if (m_bNeighborLeftSwarm){
			// let neighbors know one of our neighbors left the flock
			m_sFlockingParams.TargetDistance = MAX_TARGET_DIST;
			m_bFollowingNeighbor = false;
			m_cLEDColor = CColor::ORANGE;
			LOG << "fb" << controllerID << ": yellow -> orange (neighbor left flock) TD: " << m_sFlockingParams.TargetDistance << std::endl;
		} else if (orangeBlobSeen){
			if (m_sFlockingParams.TargetDistance - TARGET_DIST_DIFF >= MIN_TARGET_DIST){
				m_sFlockingParams.TargetDistance = m_vLOSNeighbors [m_nNeighborToFollow].TargetDistance - TARGET_DIST_DIFF;
			}
			m_cLEDColor = CColor::ORANGE;
			LOG << "fb" << controllerID << ": yellow -> orange (see orange neighbor fb" 
			    << m_nNeighborToFollow << ") TD: " << m_sFlockingParams.TargetDistance << std::endl;
		
		} else if (unBlobsSeen == 0){
			// we either escaped the local minimum or are no longer in the communication range of our neighbors
			m_nTotalNumRobots = CSimulator::GetInstance().GetSpace().GetEntitiesByType("foot-bot").size();
			m_cLEDColor = CColor::GREEN;
			LOG << "fb" << controllerID << ": yellow -> green (no neighbors) TD: " << m_sFlockingParams.TargetDistance << std::endl;
		}
	} else if (m_cLEDColor == CColor::ORANGE){

		if (unBlobsSeen == 0){
			// we either escaped the local minimum or are no longer in the communication range of our neighbors
			m_nTotalNumRobots = CSimulator::GetInstance().GetSpace().GetEntitiesByType("foot-bot").size();
			m_cLEDColor = CColor::GREEN;
			LOG << "fb" << controllerID << ": orange -> green (no neighbors) TD: " << m_sFlockingParams.TargetDistance << std::endl;
		} else if (!greenBlobSeen &&  m_sFlockingParams.TargetDistance >= MIN_TARGET_DIST){
			// then we're not near green  -> reduce spread to maintain a stable structure
			//LOG << "fb" << controllerID << " contracting" << std::endl;
			m_sFlockingParams.TargetDistance -= CONTRACT_RATE;
		} else if (greenBlobSeen && m_sFlockingParams.TargetDistance < MAX_TARGET_DIST){
			// we're near green, increase TargetDistance to propel ourselves out of the local minimum
			//LOG << "fb" << controllerID << " spreading" << std::endl;
			m_sFlockingParams.TargetDistance += SPREAD_RATE;
		}

		if (redBlobSeen){
			Reset();
			m_cLEDColor = CColor::RED;
			LOG << "fb" << controllerID << ": orange -> red (see red neighbor) TD: " << m_sFlockingParams.TargetDistance << std::endl;
		}

	} else if (m_cLEDColor == CColor::GREEN){
		// once we've seen every robot escape the local minimum, we're ready to keep descending gradient
		if (m_vWhiteLEDsSeen.size() == m_nTotalNumRobots - 1){
			Reset();
			m_cLEDColor = CColor::RED;
			LOG << "fb" << controllerID << ": green -> red (no robots left to wait for) TD: " << m_sFlockingParams.TargetDistance << std::endl;

			for (std::set<UInt32>::iterator it = m_vWhiteLEDsSeen.begin(); it != m_vWhiteLEDsSeen.end(); ++it){
				LOG << " " << *it;
			}
			LOG << std::endl;
		}

	} else if (m_cLEDColor == CColor::WHITE){
		if (m_sFlockingParams.TargetDistance > MIN_TARGET_DIST){
			m_sFlockingParams.TargetDistance -= SPREAD_RATE;
		}

		if (redBlobSeen){
			Reset();
			m_cLEDColor = CColor::RED;
			LOG << "fb" << controllerID << ": white -> red (see red neighbor) TD: " << m_sFlockingParams.TargetDistance << std::endl;
		}
	}

	m_nNumNeighbors = unBlobsSeen;
	return cAccum;
}

/****************************************/
/****************************************/

Real CPathFindBot::updateWeightedAvgSpeed(Real prevWeightedAvg){
	Real newWeightedAvg;
	Real timeDelta = CSimulator::GetInstance().GetPhysicsEngine("dyn2d").GetInverseSimulationClockTick();
	CVector2 currentPosition = {
		m_pcPosSensor->GetReading().Position.GetX(),
		m_pcPosSensor->GetReading().Position.GetY()
	};

	CVector2 vecToTarget = m_cTarget - currentPosition;
	CVector2 stepVelocity = (currentPosition - m_cPrevPosition) * 10000 / timeDelta;

	Real speedToTarget = ARGOS_COS((stepVelocity.Angle() - vecToTarget.Angle()).GetValue()) * stepVelocity.Length();
	newWeightedAvg = prevWeightedAvg + AVG_WEIGHT * (speedToTarget - prevWeightedAvg);
	m_cPrevPosition = currentPosition;

	return newWeightedAvg;
}

/****************************************/
/****************************************/

void CPathFindBot::updateLOSNeighbors(){

	const CCI_RangeAndBearingSensor::TReadings & tRABReadings = m_pcRABSensor->GetReadings();
	std::vector <UInt32> loneNeighborIDs;               // IDs of neighbors with only one neighbor
	UInt8 prevNumLOSNeighbors = m_vLOSNeighbors.size(); // the number of LOS neighbors we had last time step

	// save the location of the neighbor we were following in case we need to update m_nNeighborToFollow
	CVector2 prevNeighborToFollowLoc;

	if (m_bFollowingNeighbor){
		prevNeighborToFollowLoc = CVector2(
			m_vLOSNeighbors [m_nNeighborToFollow].distance, 
			m_vLOSNeighbors [m_nNeighborToFollow].angle
		);
	}

	for (std::map <UInt32, SLOSNeighborState>::iterator it = m_vLOSNeighbors.begin(); it != m_vLOSNeighbors.end(); ++it){
		if (it->second.numNeighbors == 1){
			loneNeighborIDs.push_back(it->first);
		}
	}
	m_vLOSNeighbors.clear();

	for (CCI_RangeAndBearingSensor::TReadings::const_iterator it = tRABReadings.begin(); it != tRABReadings.end(); ++it){
		// extract numNeighbors and TargetDistance fields from message buffer
		const UInt8 *messageBuffer = it->Data.ToCArray();

		UInt32 neighborID;
		UInt8 numNeighbors;
		float TargetDistance;

		memcpy (&neighborID, messageBuffer, 4);
		numNeighbors = messageBuffer [4];
		memcpy (&TargetDistance, messageBuffer + 5, 4);

		m_vLOSNeighbors [neighborID] = (
			SLOSNeighborState(
				numNeighbors,
				it->Range,
				it->HorizontalBearing,
				TargetDistance
			)
		);
	}

	
	/*
	if (controllerID == 4 && CSimulator::GetInstance().GetSpace().GetSimulationClock() >= 1864
	&& CSimulator::GetInstance().GetSpace().GetSimulationClock() <= 1870){
		LOG << "fb" << controllerID << "'s LOS neighbors:" << std::endl;
		for (std::map <UInt32, SLOSNeighborState>::iterator it = m_vLOSNeighbors.begin(); it != m_vLOSNeighbors.end(); ++it){
			LOG << "fb" << it->first << ": " << it->second.numNeighbors << ", "
			    << it->second.TargetDistance << std::endl;
		}
		if (loneNeighborIDs.size() > 0){
			LOG << "lone neighbors: " << std::endl;
			for (std::vector<UInt32>::iterator it = loneNeighborIDs.begin(); it != loneNeighborIDs.end(); ++it){
				LOG << " fb" << *it;
			}
			LOG << std::endl;
		}
	}*/
	
	// check to see if the neighbor we were following is still in line of sight
	if (m_bFollowingNeighbor && m_vLOSNeighbors.find (m_nNeighborToFollow) == m_vLOSNeighbors.end()){
		// if it's not, the neighbor nearest to the one we were following is now the one we're following
		m_nNeighborToFollow = getLOSNeighborID (prevNeighborToFollowLoc.Length(), prevNeighborToFollowLoc.Angle());
		// the TargetDistance has to be the same, otherwise all TargetDistances reduce until there's no distance between the robots
		//m_sFlockingParams.TargetDistance = m_vLOSNeighbors [m_nNeighborToFollow].TargetDistance;
		//LOG << "fb" << controllerID << " now following fb" << m_nNeighborToFollow << std::endl;
	}

	if (loneNeighborIDs.size() > 0){
		for (std::vector<UInt32>::iterator it = loneNeighborIDs.begin(); it != loneNeighborIDs.end(); ++it){
			if (m_vLOSNeighbors.find (*it) == m_vLOSNeighbors.end()){
				m_bNeighborLeftSwarm |= true;
				LOG << "fb" << controllerID << ": lost lone neighbor fb" << *it << std::endl;
			}
		}
	}
}

/****************************************/
/****************************************/

UInt32 CPathFindBot::getLOSNeighborID (Real distance, CRadians angle){
	UInt32 id;
	Real minDistance = INFINITY;
	CVector2 robotOfInterestLoc = {distance, angle};

	m_bFollowingNeighbor &= !m_vLOSNeighbors.empty();

	for (std::map <UInt32, SLOSNeighborState>::iterator it = m_vLOSNeighbors.begin(); it != m_vLOSNeighbors.end(); ++it){
		Real diff = (robotOfInterestLoc - CVector2(it->second.distance, it->second.angle)).Length();
		if (diff < minDistance){
			minDistance = diff;
			id = it->first;
		}
	}
	return id;
}

/**
 * Announce the number of neighbors we have and our current TargetDistance
 *
 * The message buffer size is 10 bytes, and our data is:
 *   - our controller ID: 4 bytes (in case we want to have 4294967296 robots;)
 *   - the number of neighbors: 1 byte (since we can safely assume this will be less than 255 due to the LJP)
 *   - the TargetDistance: 4 bytes (a double truncated to a single precision floating point number)
 *   - one left over byte we don't need: '\0'
 *
 * "<controller ID><number of neighbors><TargetDistance>"
 */
void CPathFindBot::announceState(){
	UInt32 id = controllerID;
	UInt8 numNeighbors = m_nNumNeighbors;
	float truncTD = m_sFlockingParams.TargetDistance;

	UInt8 TDBuffer [4];
	CByteArray message;

	memcpy(TDBuffer, &truncTD, 4);

	message.AddBuffer((const UInt8 *) &id, 4);
	message.AddBuffer(&numNeighbors, 1);
	message.AddBuffer(TDBuffer, 4);
	message.AddBuffer((const UInt8 *) "\0", 1);

	m_pcRABActuator->SetData(message);
}

/****************************************/
/****************************************/

CVector2 CPathFindBot::getAbsolutePosition (CVector2 relativePos){
	CVector3 position3D = m_pcPosSensor->GetReading().Position;
	CVector2 position (position3D.GetX(), position3D.GetY());
	CQuaternion orientation = m_pcPosSensor->GetReading().Orientation;
	CRadians z_angle, y_angle, x_angle;
	orientation.ToEulerAngles(z_angle, y_angle, x_angle);

	return position + CVector2 (relativePos.Length(), relativePos.Angle() + z_angle);
}

/****************************************/
/****************************************/

void CPathFindBot::SetWheelSpeedsFromVector(const CVector2& c_heading) {
   /* Get the heading angle */
   CRadians cHeadingAngle = c_heading.Angle().SignedNormalize();
   /* Get the length of the heading vector */
   Real fHeadingLength = c_heading.Length();
   /* Clamp the speed so that it's not greater than MaxSpeed */
   Real fBaseAngularWheelSpeed = Min<Real>(fHeadingLength, m_sWheelTurningParams.MaxSpeed);
   /* State transition logic */
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::HARD_TURN) {
      if(Abs(cHeadingAngle) <= m_sWheelTurningParams.SoftTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
      }
   }
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::SOFT_TURN) {
      if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
      }
      else if(Abs(cHeadingAngle) <= m_sWheelTurningParams.NoTurnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::NO_TURN;
      }
   }
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::NO_TURN) {
      if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
      }
      else if(Abs(cHeadingAngle) > m_sWheelTurningParams.NoTurnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
      }
   }
   /* Wheel speeds based on current turning state */
   Real fSpeed1, fSpeed2;
   switch(m_sWheelTurningParams.TurningMechanism) {
      case SWheelTurningParams::NO_TURN: {
         /* Just go straight */
         fSpeed1 = fBaseAngularWheelSpeed;
         fSpeed2 = fBaseAngularWheelSpeed;
         break;
      }
      case SWheelTurningParams::SOFT_TURN: {
         /* Both wheels go straight, but one is faster than the other */
         Real fSpeedFactor = (m_sWheelTurningParams.HardTurnOnAngleThreshold - Abs(cHeadingAngle)) / m_sWheelTurningParams.HardTurnOnAngleThreshold;
         fSpeed1 = fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
         fSpeed2 = fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
         break;
      }
      case SWheelTurningParams::HARD_TURN: {
         /* Opposite wheel speeds */
         fSpeed1 = -m_sWheelTurningParams.MaxSpeed;
         fSpeed2 =  m_sWheelTurningParams.MaxSpeed;
         break;
      }
   }
   /* Apply the calculated speeds to the appropriate wheels */
   Real fLeftWheelSpeed, fRightWheelSpeed;
   if(cHeadingAngle > CRadians::ZERO) {
      /* Turn Left */
      fLeftWheelSpeed  = fSpeed1;
      fRightWheelSpeed = fSpeed2;
   }
   else {
      /* Turn Right */
      fLeftWheelSpeed  = fSpeed2;
      fRightWheelSpeed = fSpeed1;
   }
   /* Finally, set the wheel speeds */
   m_pcWheels->SetLinearVelocity(fLeftWheelSpeed, fRightWheelSpeed);
}

/****************************************/
/****************************************/

void CPathFindBot::Reset(){
	if (!m_vLOSNeighbors.empty()){
		m_vLOSNeighbors.clear();
	}
	if (!m_vWhiteLEDsSeen.empty()){
		m_vWhiteLEDsSeen.clear();
	}
	m_bNeighborLeftSwarm = false;
	m_bFollowingNeighbor = false;
	m_bSeenGreenLED = false;
	m_bReadyToBreakAway = false;
	m_nNumNeighbors = 0;
	m_dWeightAvgSpeedTarget = 20;
	m_sFlockingParams.TargetDistance = MIN_TARGET_DIST;
	m_cPrevPosition = CVector2 (0, 0);
	/* Enable camera filtering */
	m_pcCamera->Enable();
	/* Set beacon color to all red to be visible for other robots */
	m_cLEDColor = CColor::RED;
	//m_pcLEDs->SetSingleColor(12, m_cLEDColor);
}

/****************************************/
/****************************************/

void CPathFindBot::Destroy(){}

REGISTER_CONTROLLER(CPathFindBot, "pathfindbot")
