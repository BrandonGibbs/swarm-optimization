#include "lubot-controller.h"
#include <iostream>
#include <string>
#include <cmath> // logarithm, INFINITY
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/datatypes/byte_array.h>

CLubot::CLubot() :
		m_pcWheels(NULL),
		m_pcProximity(NULL),
		m_pcCommsTransmr(NULL),
		m_pcCommsRecvr(NULL),
		m_pcPosSensor(NULL){}
		
void CLubot::Init(TConfigurationNode& t_node){

  // get id declared in the config file
  m_nID = std::stoi(GetId().substr(2, std::string::npos));
  /**
   *  All actuators/sensors are initilized here. They need to match the
   *  specifications in the <constrollers> section of the .argos file.
   */
  m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
  m_pcProximity = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
  m_pcCommsTransmr = GetActuator<CCI_RangeAndBearingActuator>("range_and_bearing");
  m_pcCommsRecvr = GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");
  m_pcPosSensor = GetSensor<CCI_PositioningSensor>("positioning");

  /* setting the velocity of each wheel of Lubot 1 */
  if (m_nID == 1){
    m_pcWheels->SetLinearVelocity(50, 50);
  }
  std::cout << "initialized Lubot " << m_nID << std::endl;
}

void CLubot::ControlStep(){
  /**
   * Position and orientation fields are basically the same as the ones set in 
   * the arena section of the .argos file: 
   *   position = (x coordinate, y coordinate, z coordinate), 
   *   orientation = rotation angle about given axis in degrees (z, y, x)
   */
  const CCI_PositioningSensor::SReading & sPosReading = m_pcPosSensor->GetReading();
  std::cout << "---------------- fb" << m_nID << ": Position = " 
			<< sPosReading.Position << ", Orientation = " 
			<< sPosReading.Orientation << std::endl;
			
  // designate lubot 0 to be a listener and lubots 1 and 2 to be transmitters
  if (m_nID == 0){
	  
	/**
	 * CCI_RangeAndBearingSensor has a TReading, which is a vector<SPacket>.
	 * SPacket is a struct with fields:
	 *   - Range
	 *   - Horizontal/Vertical Bearing
	 *   - Data
	 *
	 * There are a few things to note about this example:
	 *   - there are only readings when there's no barrier between the robots
	 *   - the readings stop when the robots are no longer in range
	 *   - the vertical bearing is always 0
	 *   - the 10 byte data size limits
	 *   - transmissions by lubot 2 do not interfere with lubot 0 receiving transmissions from 
	 *     lubot 1
	 *
	 * The range can be set with the rab_range property and the data size can be set with the 
	 * rab_data_size property in the .argos file in the <arena> section.
	 */
    const CCI_RangeAndBearingSensor::TReadings &
      tRABReadings = m_pcCommsRecvr->GetReadings();
    
    for (int i = 0; i < tRABReadings.size(); i++){
	  std::string sData((const char *)tRABReadings[i].Data.ToCArray());
	  
      std::cout << "received packet:\n"
                << "          range:\t" << tRABReadings[i].Range               << std::endl
                << "       bearing:\t(" << tRABReadings[i].HorizontalBearing
                <<                 ", " << tRABReadings[i].VerticalBearing     << ")" << std::endl
                << "           data:\t" << tRABReadings[i].Data
				<< " encoded data:\t\"" << sData << "\"" << std::endl;
    }
  } else if (m_nID == 1) {
	UInt8 d = 'E';
    m_pcCommsTransmr->SetData(0, d);
	
   /** 
    * CCI_FootBotProximitySensor has a TReading, which is a vector<SReading>.
    * The SReading struct has a Value and Angle
    *
	* The footbot is 17 cm in diameter as described here:
	* https://www.swarmanoid.org/swarmanoid_hardware.php.html
	* (click on the tab that says "Foot-bots")
	*
	* and '$ argos3 -q proximity' says that this sensor returns readings 
	* using exp(-d), so we can get the distance by doing the inverse:
	* 	-ln(value)
	*
	* Using the pythagorean theorem and some high school physics, we can see
	* that fb1 collides with the top-left corner at t=3.86s 
	* (between t=38 and t=39 in the argos logs since it's running at 10 ticks
	*  per second as defined in the .argos file).
	*
	* You'll notice that at t=38, the readings from sensors 0-5 
	* (the front-left) and sensors 18-23 (front-right) are all showing a 
	* reading (as they should), but at t>=39 sensors 2, 3, 20, and 21 
	* incorrectly indicate that there's nothing in range. Ideally the robots
	* shouldn't collide with surroundings, but it's something we should keep 
	* in mind.
    */
    const CCI_FootBotProximitySensor::TReadings& 
      tProxReads = m_pcProximity->GetReadings();
	
	// used iterators here, but could have used subscript notation like in RAB sensor readings
	for (CCI_FootBotProximitySensor::TReadings::const_iterator it = tProxReads.begin(); 
	     it != tProxReads.end(); 
		 ++it){
		double distance = INFINITY; // nothing in range
		if (it->Value == 1){
			// object is touching the sensor
			distance = 0;
		} else if (it->Value > 0){
			// the distance in centimeters to the object
			distance = -log(it->Value);
		} 
		
		std::cout << "Prox: Value: " << it->Value << " Angle: " << it->Angle << std::endl;
		std::cout << "-log(Value) -> d: " << distance << std::endl;
	}
  } else {
	std::string s("I'm fb2!!");
	const CByteArray d((const UInt8 *)s.c_str(), s.size() + 1);
    m_pcCommsTransmr->SetData(d);
  }
}

void CLubot::Reset(){
  /* setting the velocity of each wheel of Lubot 1 */
  if (m_nID == 1){
    m_pcWheels->SetLinearVelocity(50, 50);
  }
  std::cout << "reset Lubot" << std::endl;
}

void CLubot::Destroy(){
  std::cout << "destroyed Lubot " << m_nID << std::endl;
}

REGISTER_CONTROLLER(CLubot, "lubot_controller")
