#include "lubot-controller.h"
#include <iostream>
#include <argos3/core/utility/configuration/argos_configuration.h>

CLubot::CLubot() :
		m_pcWheels(NULL),
		m_pcProximity(NULL) {}
		
void CLubot::Init(TConfigurationNode& t_node){
  /**
   *  All actuators/sensors are initilized here. They need to match the
   *  specifications in the <constrollers section in the config.
   */
  m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
  m_pcProximity = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
  m_pcCommsTransmr = GetActuator<CCI_RangeAndBearingActuator>("range_and_bearing");
  m_pcCommsRecvr = GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");
  /* setting the velocity of each wheel */
  m_pcWheels->SetLinearVelocity(3, 3);
  std::cout << "initialized Lubot" << std::endl;
}

void CLubot::ControlStep(){
	/** 
	 * CCI_FootBotProximitySensor has a struct SReading
	 * which has a Value and Angle
   *
   * The hungarian notation for TReadings indicates
   * T is for typedef. The verbose type is vector<SReading>
	 */
	const CCI_FootBotProximitySensor::TReadings& 
    tProxReads = m_pcProximity->GetReadings();
  /* I still need to figure out how to use these readings */
  //std::cout << tProxReads << std::endl;
}

void CLubot::Reset(){
  std::cout << "reset Lubot" << std::endl;
}

void CLubot::Destroy(){
  std::cout << "destroyed Lubot" << std::endl;
}

REGISTER_CONTROLLER(CLubot, "lubot_controller")
