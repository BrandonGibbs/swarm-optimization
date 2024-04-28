#include "pathfindbot-controller.h"

#include <cmath> /* INFINITY */
#include <cstring> /* memcpy */
#include <argos3/core/simulator/simulator.h> /* CSimulator::GetInstance */
#include <argos3/core/utility/logging/argos_log.h> /* LOG */

#define AVG_WEIGHT      0.01 // used to calculate weighted average; must be in range [0, 1)
			     // smaller makes value more stable/less responsive
#define SPEED_LOCAL_MIN 0.1  // when our weighted average reaches this value,
			     // we've decided we're in a local minima
#define SPREAD_RATE     0.1  // the rate at which TargetDistance parameter is increased/decreased

#define MIN_TARGET_DIST 75   // these will be our minimum and maximum TargetDistance values for expansion/contraction
#define MAX_TARGET_DIST 190


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

	if (m_cLEDColor == CColor::RED){
		if(m_dWeightAvgSpeedTarget < SPEED_LOCAL_MIN){
			// tell neighbors that we're in a local minimum
			m_cLEDColor = CColor::BLUE;
			LOG << controllerID << ": red -> blue (local minima detected)" << std::endl
			    << "weighted average velocity toward target: " << m_dWeightAvgSpeedTarget << std::endl;
		}
	}
	
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
	}

	if(m_sWheelTurningParams.m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
	   cAccumulator.Length() < m_sWheelTurningParams.m_fDelta ) {
		/* Go straight to the target location*/
		SetWheelSpeedsFromVector(targetVec + flockVec);
	} else {
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
	size_t unBlobsSeen = 0;
	bool redBlobSeen    = false,
	     blueBlobSeen   = false,
	     yellowBlobSeen = false,
	     greenBlobSeen  = false;

	const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings & sCameraReadings = m_pcCamera->GetReadings();

	for(CCI_ColoredBlobOmnidirectionalCameraSensor::TBlobList::const_iterator it = sCameraReadings.BlobList.begin();
	it != sCameraReadings.BlobList.end(); ++it){
		
		if ((*it)->Distance < 1.8 * m_sFlockingParams.TargetDistance){
			if (m_cLEDColor == CColor::BLUE && (*it)->Color == CColor::YELLOW){
				// if we see yellow, we need to get that robot's id so we can determine our own TargetDistance
				m_nNeighborToFollow = getLOSNeighborID ((*it)->Distance, (*it)->Angle);
				if (m_bFollowingNeighbor){
					LOG << "fb" << controllerID << " now following fb" << m_nNeighborToFollow << std::endl;
				}
			}
			fLJ = m_sFlockingParams.GeneralizedLennardJones ((*it)->Distance);
			cAccum += CVector2 (fLJ, (*it)->Angle);
			unBlobsSeen ++;

			redBlobSeen    |= (*it)->Color == CColor::RED;
			blueBlobSeen   |= (*it)->Color == CColor::BLUE;
			yellowBlobSeen |= (*it)->Color == CColor::YELLOW;
			greenBlobSeen  |= (*it)->Color == CColor::GREEN;
		}
	}

	if (m_cLEDColor == CColor::RED){
		if (blueBlobSeen){
			// one of our neighbors senses that we're in a local minima
			m_cLEDColor = CColor::BLUE;
			LOG << controllerID << ": red -> blue (see blue neighbor)" << std::endl;
		}
	} else if (m_cLEDColor == CColor::BLUE){
		if (!yellowBlobSeen && !m_bNeighborLeftSwarm && m_sFlockingParams.TargetDistance < MAX_TARGET_DIST){
			// spread out to find escape from local minima
			m_sFlockingParams.TargetDistance += SPREAD_RATE;
		} else {
			// stop expanding because 
			//   - we either lost a neighbor with no other neighbors, 
			//   - have reached the max communication range, 
			//   - or one of these conditions applies to our neighbors
			m_cLEDColor = CColor::YELLOW;
			LOG << controllerID << ": blue -> yellow " << m_sFlockingParams.TargetDistance << std::endl;
		}

		if (unBlobsSeen == 0){
			// we either escaped the local minima or are no longer in the communication range of our neighbors
			m_cLEDColor = CColor::GREEN;
			LOG << controllerID << ": blue -> green (no neighbors)" << std::endl;
		}
	} else if (m_cLEDColor == CColor::YELLOW){
		if (m_bFollowingNeighbor){
			m_sFlockingParams.TargetDistance = m_vLOSNeighbors [m_nNeighborToFollow].TargetDistance + 0.1;
		}
		if (unBlobsSeen == 0){
			// we either escaped the local minima or are no longer in the communication range of our neighbors
			m_cLEDColor = CColor::GREEN;
			LOG << controllerID << ": yellow -> green (no neighbors)" << std::endl;
		}
	} else if (m_cLEDColor == CColor::GREEN){
		if (unBlobsSeen != 0){
			if (redBlobSeen && unBlobsSeen == 5){
				// our neighbors are ready to start descending to minima
				m_cLEDColor = CColor::RED;
				LOG << controllerID << ": green -> red (red neighbors)" << std::endl;
			}
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
	m_cPrevPosition = currentPosition; // set for the next time step

	return newWeightedAvg;
}

/****************************************/
/****************************************/

void CPathFindBot::updateLOSNeighbors(){

	const CCI_RangeAndBearingSensor::TReadings & tRABReadings = m_pcRABSensor->GetReadings();
	bool hadLoneNeighbor = false;
	UInt32 loneNeighborID;
	UInt8 prevNumLOSNeighbors = m_vLOSNeighbors.size();

	// save in case we need to update m_nNeighborToFollow
	CVector2 prevNeighborToFollowLoc (m_vLOSNeighbors [m_nNeighborToFollow].distance, m_vLOSNeighbors[m_nNeighborToFollow].angle); 

	for (int i = 0; i< prevNumLOSNeighbors; i++){
		if (m_vLOSNeighbors[i].numNeighbors == 1){
			loneNeighborID = m_vLOSNeighbors[i].controllerID;
			hadLoneNeighbor = true;
			break;
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
				neighborID,
				numNeighbors,
				it->Range,
				it->HorizontalBearing,
				TargetDistance
			)
		);
	}

	// check to see if the neighbor we were following is still in line of sight
	if (m_vLOSNeighbors.find (m_nNeighborToFollow) == m_vLOSNeighbors.end()){
		// if it's not, the neighbor nearest to the one we were following is now the one we're following
		m_nNeighborToFollow = getLOSNeighborID (prevNeighborToFollowLoc.Length(), prevNeighborToFollowLoc.Angle());
	}

	m_bNeighborLeftSwarm = hadLoneNeighbor && m_vLOSNeighbors.find (loneNeighborID) == m_vLOSNeighbors.end();
}

/****************************************/
/****************************************/

UInt32 CPathFindBot::getLOSNeighborID (Real distance, CRadians angle){
	UInt32 id;
	Real minDistance = INFINITY;
	CVector2 robotOfInterestLoc = {distance, angle};

	m_bFollowingNeighbor = !m_vLOSNeighbors.empty();

	for (std::map <UInt32, SLOSNeighborState>::iterator it = m_vLOSNeighbors.begin(); it != m_vLOSNeighbors.end(); ++it){
		Real diff = (robotOfInterestLoc - CVector2(it->second.distance, it->second.angle)).Length();
		if (diff < minDistance){
			id = it->second.controllerID;
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
   m_dWeightAvgSpeedTarget = 0;
   m_sFlockingParams.TargetDistance = 75;
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
