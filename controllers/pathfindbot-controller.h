/**
 * This is an example which borrows from the argos3-examples repository along with some modifications by our professor.
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
		UInt32   controllerID;
		UInt8    numNeighbors;
		Real     distance;
		CRadians angle;
		float    TargetDistance;
		SLOSNeighborState()=default;
		SLOSNeighborState(UInt32 id, UInt8 nn, Real d, CRadians a, float td):
			controllerID(id), numNeighbors(nn), distance(d), angle(a), TargetDistance(td){}
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
	 * Look at the definition of the m_dWeightAvgSpeedTarget definition below for more info
	 */
	Real updateWeightedAvgSpeed (Real prevWeightedAvg);

	/**
	 * This will update map <UInt32, SLOSNeighborState> m_vLOSNeighbors
	 * so it will always contain neighbors in line of sight at every time step
	 *
	 * As a side effect, m_bNeighborLeftSwarm flag will be flipped if a neighbor with
	 * no other neighbors is no longer visible. There's a small probability that
	 *   - a neighbor with no other neighbors disconnectes from us and connects to
	 *     another robot in the same time step, or
	 *   - a neighbor connected to 2 robots or more disconnects from all its neighbors
	 *     on the same time step
	 *
	 * but we will ignore these issues for now.
	 */
	void updateLOSNeighbors ();

	/**
	 * When our LED is blue and we see a yellow LED, we need to tie the yellow blob seen 
	 * with the robot making announcements on RAB actuator so that we can calculate our
	 * TargetDistance based on that robot's TargetDistance in later time steps. This value
	 * is saved in m_nNeighborToFollow. If we have no neighbors, m_bFollowingNeighbor is false.
	 */
	UInt32 getLOSNeighborID (Real distance, CRadians angle);

	/**
	 * Get the TargetDistance of a neighbor.
	 *
	 * First, see if m_nNeighborToFollow is in m_vLOSNeighbors. If it's not, get the TargetDistance
	 * nearest to that robot's position when we last saw them. This other robot's id will be stored in
	 * m_nNeighborToFollow.
	 */
	float getTargetDistance();

	/**
	 * Announce 
	 *   - our controller ID
	 *   - the number of neighbors we have and  
	 *   - our current TargetDistance
	 *
	 *   on RAB actuator
	 */
	void announceState ();

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
	 * 	BLUE:   reached a local minima => expand flock
	 * 	YELLOW: stop expanding flock
	 * 	GREEN:  wait for neighbors to arrive, then guide flock out of LM
	 */
	CColor m_cLEDColor;
	UInt8 m_nNumNeighbors; // the number of blobs we can see with camera sensor

	/**
	 * Line of sight neighbors who we can communicate with over RAB sensors/actuators
	 *
	 * Keep in mind that this list will almost always be smaller than number of blobs
	 * we can see with the camera sensor because other robots will block the line of sight
	 * needed for communication over RAB sensors/actuators.
	 *
	 * This structure is useful for 
	 *   - determining if a neighbor has left the swarm and
	 *   - getting neighbors' TargetDistance
	 */
	std::map <UInt32, SLOSNeighborState> m_vLOSNeighbors;
	bool m_bNeighborLeftSwarm;
	UInt32 m_nNeighborToFollow; // id of LOS neighbor whose TargetDistance we need
	bool m_bFollowingNeighbor;

	/* 
	 * weighted average of the speed to the target: the more recent the speed measurement, 
	 * the greater the weight this is used to determine whether flock is in a local minima
	 */
	Real m_dWeightAvgSpeedTarget;
	CVector2 m_cPrevPosition; // this is needed to compute the weighted average speed
};


#endif
