/**
 *  Our controller will be similar to the diffusion example due to
 *  the 
 *    - differential steering actuator (to move)
 *    - and proximity sensor (to sense nearby objects)
 *
 *  In addition to these modules, our robot will need to communicate 
 *  with other robots, so we will add
 *    - a range_and_bearing actuator (comms transmitter)
 *    - a range_and_bearing sensor (comms receiver)
 *
 *  The range_and_bearing transmissions are broadcast to all robots
 *  in line of sight, so knowledge of the arena and all communications
 *  would have to be collective. If this is an issue, we can also use
 *  the simple_radios module. None of the examples use it though, and
 *  I don't know if the communication range is beyond what we need.
 *
 *  For examples using the range_and_bearing module, check these out:
 *    - foraging
 *    - eyebot_circle
 *    - and eyebot_flocking
 *  
 *  and here's a helpful thread:
 *    https://argos-sim.info/forum/viewtopic.php?t=201
 *
 *  TODO:
 *  Since we don't know if our robots are going to know where the 
 *  target is, we may need to have an area on the ground with a 
 *  different color which our robot can sense with a 
 *  footbot_motor_ground sensor. The only example using this sensor 
 *  is foraging.
 *
 *  It'll be easier if we assume our robots know the target 
 *  coordinates until Professor Lu tells us otherwise.
 */
#ifndef CLUBOT_CONTROLLER_H
#define CLUBOT_CONTROLLER_H
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>

using namespace argos;

class CLubot : public CCI_Controller {

public:
	CLubot();
	virtual ~CLubot(){}
	virtual void Init(TConfigurationNode& t_node);
	virtual void ControlStep();
	virtual void Reset();
	virtual void Destroy();

private:
  /**
   *  I'm following the Hungarian naming convention of argos which
   *  says that letters before the underscore denote the scope of
   *  the variable (m for member), and letters after denote type
   *  (p for pointer, c for class instance I assume).
   */
  CCI_DifferentialSteeringActuator* m_pcWheels;
  CCI_FootBotProximitySensor      * m_pcProximity;
  CCI_RangeAndBearingActuator     * m_pcCommsTransmr;
  CCI_RangeAndBearingSensor       * m_pcCommsRecvr;
  int                               m_nID;
};


#endif
