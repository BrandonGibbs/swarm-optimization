/**
 * Our project borrows from the argos3-examples repository along with some contributions by our professor.
 *
 * The goal of this project is to minimize the distance between a flock of robots and a target location, so it's a
 * variation of gradient descent where every robot must arrive at the target together and overcome local minima along
 * the way. 
 *
 * The motion of each robot is modelled as a Lennard-Jones fluid where the distance to its neighbors is governed by the
 * Lennard-Jones potential so that robots repel each other when too close, attract each other when in the potential well,
 * and do not affect each other when far apart. In addition, they are repelled from barriers.
 *
 * So in order to arrive at the target, the robots must be able to detect when they have reached a local minimum. This is 
 * done by calculating a weighted average of the velocity toward the target. When this average drops below a threshold,
 * the robots increase their Brownian motion so that the flock expands until a robot escapes the local minimum. When this 
 * robot no longer has neighbors, it stops completely until it's seen.
 *
 * From the flock's perspective, the robot which has lost a neighbor with no other neighbors announces this and sets its
 * Lennard-Jones potential well very high and other robots set theirs slightly below this value. This leads to a feedback
 * loop where the robots with a higher potential well are repelled, and robots with a lower potential well are attracted,
 * so that their collective trajectory is toward the robot which got separated from the flock.
 *
 *****************************************************************************************************************************
 *
 * For this to work, every robot keeps track of its neighbors with LEDs and camera sensors as well as range and bearing (RAB)
 * sensors/actuators. With RAB hardware the robots are able to transmit 10 bytes of data per time step, but these messages 
 * are susceptible to being blocked by other robots or barriers. This means that unifying neighbors' state is not a simple task.
 * Bear in mind that variables, data structures, and methods with "LOS" in the name refer to data obtained from robots in 
 * "line of sight".
 *
 * With the camera sensor, we can get a list of LEDs that we see with the following fields:
 *   - color
 *   - relative distance
 *   - relative angle
 *
 * With range and bearing sensors, the robots announce:
 *   - controller ID
 *   - the number of neighbors they see with the camera sensor
 *   - the value of their TargetDistance variable (defined in SFlockingInteractionParams struct)
 * 
 * and the RAB sensor readings also provide the relative distance and relative angle of the robot which announced it.
 * The distance and angle are used to tie information obtained from the camera sensors to messages received through
 * the RAB sensor.
 */
#ifndef CPATHFINDBOT_CONTROLLER_H
#define CPATHFINDBOT_CONTROLLER_H
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#include <argos3/core/utility/math/vector2.h>
#include <set>

using namespace argos;

class CPathFindBot : public CCI_Controller {

public:
	/*
	* The following variables are used as parameters for the
	* diffusion algorithm. You can set their value in the <parameters>
	* section of the XML configuration file, under the
	* <controllers><footbot_flocking_controller><parameters><diffusion>
	* section.
	*/
	struct SDiffusionParams {
		/*
		* Maximum tolerance for the proximity reading between
		* the robot and the closest obstacle.
		* The proximity reading is 0 when nothing is detected
		* and grows exponentially to 1 when the obstacle is
		* touching the robot.
		*/
		Real Delta;
		/* Angle tolerance range to go straight. */
		CRange<CRadians> GoStraightAngleRange;

		/* Constructor */
		SDiffusionParams();

		/* Parses the XML section for diffusion */
		void Init(TConfigurationNode& t_tree);
	};

	/*
	* The following variables are used as parameters for
	* turning during navigation. You can set their value
	* in the <parameters> section of the XML configuration
	* file, under the
	* <controllers><footbot_flocking_controller><parameters><wheel_turning>
	* section.
	*/
	struct SWheelTurningParams {
		/*
		* The turning mechanism.
		* The robot can be in three different turning states.
		*/
		enum ETurningMechanism
		{
			 NO_TURN = 0, // go straight
			 SOFT_TURN,   // both wheels are turning forwards, but at different speeds
			 HARD_TURN    // wheels are turning with opposite speeds
		} TurningMechanism;
		/*
		* Angular thresholds to change turning state.
		*/
		CRadians HardTurnOnAngleThreshold;
		CRadians SoftTurnOnAngleThreshold;
		CRadians NoTurnAngleThreshold;
		/* Maximum wheel speed */
		Real MaxSpeed;
		/* Maximum tolerance for the angle between
		* the robot heading direction and
		* the closest obstacle detected. */
		CDegrees m_cAlpha;

		/* Maximum tolerance for the proximity reading between
		* the robot and the closest obstacle.
		* The proximity reading is 0 when nothing is detected
		* and grows exponentially to 1 when the obstacle is
		* touching the robot.
		*/
		Real m_fDelta;
		/* Wheel speed. */
		Real m_fWheelVelocity;

		/* Angle tolerance range to go straight.
		* It is set to [-alpha,alpha]. */
		CRange<CRadians> m_cGoStraightAngleRange;
		void Init(TConfigurationNode& t_tree);
	};

	/*
	* The following variables are used as parameters for
	* flocking interaction. You can set their value
	* in the <parameters> section of the XML configuration
	* file, under the
	* <controllers><footbot_flocking_controller><parameters><flocking>
	* section.
	*/
	struct SFlockingInteractionParams {
		/* Target robot-robot distance in cm */
		Real TargetDistance;
		/* Gain of the Lennard-Jones potential */
		Real Gain;
		/* Exponent of the Lennard-Jones potential */
		Real Exponent;

		void Init(TConfigurationNode& t_node);
		Real GeneralizedLennardJones(Real f_distance);
	};

	/**
	 * Line of sight neighbors' state
	 *
	 * See definition of m_vLOSNeighbors below for more info
	 */
	struct SLOSNeighborState {
		UInt8    numNeighbors;
		Real     distance;
		CRadians angle;
		float    TargetDistance;

		SLOSNeighborState()=default;
		SLOSNeighborState(UInt8 nn, Real d, CRadians a, float td):
			numNeighbors(nn), distance(d), angle(a), TargetDistance(td){}
	};

	CPathFindBot();
	virtual ~CPathFindBot(){}
	virtual void Init(TConfigurationNode& t_node);
	virtual void ControlStep();
	virtual void Reset();
	virtual void Destroy();
	
protected:

	virtual CVector2 VectorToTarget();

	virtual CVector2 FlockingVector();

	void SetWheelSpeedsFromVector(const CVector2& c_heading);

private:
	/**
	 * Update the weighted average of the speed to the target
	 *
	 * The parameter and value returned are m_dWeightAvgSpeedTarget
	 */
	Real updateWeightedAvgSpeed (Real prevWeightedAvg);

	/**
	 * This will update map <UInt32, SLOSNeighborState> m_vLOSNeighbors
	 * so it will always contain neighbors in line of sight at every time step
	 *
	 * As a side effect, m_bNeighborLeftSwarm flag will be flipped if a neighbor with
	 * no other neighbors is no longer visible. 
	 *
	 * Note: There's a small probability that
	 *   - a neighbor with no other neighbors disconnectes from us and connects to
	 *     another robot in the same time step, or
	 *   - a neighbor connected to 2 robots or more disconnects from all its neighbors
	 *     on the same time step
	 *
	 * but we will ignore these issues for now.
	 */
	void updateLOSNeighbors ();

	/**
	 * When our LED is blue or yellow and we see an orange LED, we need to tie the orange  
	 * blob seen with the robot making announcements on RAB actuator so that we can calculate 
	 * our TargetDistance based on that robot's TargetDistance in later time steps. This value
	 * is saved in m_nNeighborToFollow. If we have no neighbors, m_bFollowingNeighbor is false.
	 *
	 * The default parameter maxThreshold will be a maximum distance error allowed.
	 */
	UInt32 getLOSNeighborID (Real distance, CRadians angle);

	/**
	 * Announce 
	 *   - our controller ID
	 *   - the number of neighbors we have and  
	 *   - our current TargetDistance
	 *
	 *   on RAB actuator
	 */
	void announceState ();

	/**
	 * Convert a position relative to this robot to an absolute position.
	 */
	CVector2 getAbsolutePosition (CVector2 relativePos);


	CCI_DifferentialSteeringActuator          * m_pcWheels;
	CCI_FootBotProximitySensor                * m_pcProximity;
	CCI_PositioningSensor                     * m_pcPosSensor;
	CCI_LEDsActuator                          * m_pcLEDs;
	CCI_ColoredBlobOmnidirectionalCameraSensor* m_pcCamera;
	CCI_RangeAndBearingSensor                 * m_pcRABSensor;
	CCI_RangeAndBearingActuator               * m_pcRABActuator;


	SWheelTurningParams        m_sWheelTurningParams;
	SFlockingInteractionParams m_sFlockingParams;

	UInt32 controllerID;
	CVector2 m_cTarget;

	/**
	 * The LED color will announce our flock state to others:
	 * 	RED:    descending gradient to the target
	 * 	BLUE:   reached a local minimum => expand flock
	 * 	YELLOW: stop expanding flock (expanding more would break up the flock because of 
	 * 	                              LED/camera communication range limit)
	 * 	ORANGE: neighbor left the flock (for robots still in the flock; the robot which 
	 *	                                 left turns green)
	 * 	GREEN:  guide flock out of LM
	 * 	WHITE:  escaped LM; waiting for the rest of the flock
	 */
	CColor m_cLEDColor;
	UInt8 m_nNumNeighbors; // the number of blobs we can see with camera sensor


	/**
	 * Line of sight neighbors who we can communicate with over RAB sensors/actuators
	 *
	 * Keep in mind that this list will almost always be smaller than number of blobs
	 * we can see with the camera sensor because other robots will block the line of sight
	 * needed for communication over RAB sensors/actuators.
	 */
	std::map <UInt32, SLOSNeighborState> m_vLOSNeighbors;
	bool m_bNeighborLeftSwarm;
	UInt32 m_nNeighborToFollow; // "following" in this case means we're basing our TargetDistance
	bool m_bFollowingNeighbor;  // on our neighbor's; having a smaller TargetDistance creates a 
				    // feedback loop leading to collective movement in the direction
				    // of the robot with the larger TargetDistance
	
	/**
	 * The robot with the green LED will store the IDs of the robots with white LEDs it has seen
	 * here. Once it's full, green is ready to turn its LED red to continue descent toward target.
	 */
	std::set <UInt32> m_vWhiteLEDsSeen;
	int m_nTotalNumRobots;

	/**
	 * Our angle with respect to the robot with a green LED when it was first seen.
	 *
	 * This will be needed to get our change in angle with respect to green LED to determine if we've
	 * been guided out of local minimum.
	 */
	CRadians m_cAngleWRTGreenFirstSeen;
	bool m_bSeenGreenLED,
	     m_bReadyToBreakAway; // this will be true when our angle WRT green LED is greater than some
				  // threshold

	/* 
	 * weighted average of the speed to the target: the more recent the speed measurement, 
	 * the greater the weight
	 *
	 * this is used to determine whether the flock is in a local minimum
	 */
	Real m_dWeightAvgSpeedTarget;
	CVector2 m_cPrevPosition; // this is needed to compute the weighted average speed
};


#endif
