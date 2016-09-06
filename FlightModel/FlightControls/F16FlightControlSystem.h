#ifndef _F16FLIGHTCONTROLSYSTEM_H_
#define _F16FLIGHTCONTROLSYSTEM_H_

#include <cmath>

#include "F16Constants.h"
#include "../UtilityFunctions.h"

//#include "../include/general_filter.h"
#include "DummyFilter.h"

#include "Inputs/F16AnalogInput.h"
#include "Atmosphere/F16Atmosphere.h"

#include "F16Actuator.h"

#include "F16FcsCommon.h"

#include "F16FcsPitchController.h"
#include "F16FcsRollController.h"
#include "F16FcsYawController.h"
#include "F16FcsLeadingEdgeController.h"
#include "F16FcsTrailingFlapController.h"
#include "F16FcsAirbrakeController.h"

/*
sources:
- NASA TP 1538
- NASA TN D-8176
- NASA TP 2857
- NASA TM 74097
- AD-A055 417
- AD-A274 057
- AD-A202 599
- AD-A230 517
- NASA Technical Paper 3355
- NASA Technical Memorandum 86026 (AFTI Control laws)
- WL-TR-97-3091
*/

// TODO! combine controllers with real actuator support


class F16FlightControls
{
public:
	F16Atmosphere *pAtmos;
	F16BodyState bodyState;
	F16FlightSurface flightSurface;
	F16TrimState trimState;

protected:

	// TODO: get rid of this
	bool		simInitialized;



	// note: airbrake limit different when landing gear down (prevent strike to runway)
	// cx_brk = 0.08, --coefficient, drag, breaks <- for airbrake?
	bool airbrakeSwitch; // switch status
	F16Actuator airbrakeActuator;
	double airbrakeDrag;

	bool isGearDown; // is landing gear down
	// replace with:
	//F16LandingGear *landingGear;

	// Pitch controller variables
	AnalogInput		longStickInput; // pitch normalized
	AnalogInput		latStickInput; // bank normalized
	AnalogInput		pedInput;		// Pedal input command normalized (-1 to 1)

	F16Actuator		rudderActuator;
	/*
	F16Actuator		elevatorActuatorLeft;
	F16Actuator		elevatorActuatorRight;
	F16Actuator		flaperonActuatorLeft;
	F16Actuator		flaperonActuatorRight;
	F16Actuator		leadingedgeActuatorLeft;
	F16Actuator		leadingedgeActuatorRight;
	*/

	F16FcsPitchController pitchControl;
	F16FcsRollController rollControl;
	F16FcsYawController yawControl;
	F16FcsLeadingEdgeController leadingedgeControl;
	F16FcsTrailingFlapController flapControl;
	F16FcsAirbrakeController airbrakeControl;

	// when MPO pressed down, override AOA/G-limiter and direct control of horiz. tail
	bool manualPitchOverride;

	// flap position logic according to when landing gears are down:
	// flaps are controlled with landing gear lever as well,
	// gears go down -> trailing edge flaps go down
	// gear go up -> trailing edge flaps go up
	bool gearRelatedFlaps;


public:
	F16FlightControls(F16Atmosphere *atmos)
		: pAtmos(atmos)
		, bodyState()
		, flightSurface()
		//, trimState(-0.3, 0, 0) // <- -0.3 pitch trim, RSS compensation?
		, trimState(0, 0, 0)
		, simInitialized(false)
		, airbrakeSwitch(false)
		, airbrakeActuator(1.0, 0, 1.0) //
		, airbrakeDrag(0)
		, isGearDown(true)
		, longStickInput(-1.0, 1.0)
		, latStickInput(-1.0, 1.0)
		, pedInput(-1.0, 1.0)
		, rudderActuator(1.0, -30.0, 30.0)
		, pitchControl(&bodyState, &flightSurface)
		, rollControl(&bodyState, &flightSurface)
		, yawControl(&bodyState, &flightSurface)
		, leadingedgeControl(&bodyState, &flightSurface)
		, flapControl(&bodyState, &flightSurface)
		, airbrakeControl(&bodyState, &flightSurface)
		, manualPitchOverride(false)
		, gearRelatedFlaps(false)
	{
		// just do this once when constructing
		initialize(0);
	}
	~F16FlightControls() {}

	void setLatStickInput(double value) 
	{
		latStickInput = value;
	}
	void setLongStickInput(double value) 
	{
		longStickInput = -value;
	}
	void setPedInput(double value)
	{
		pedInput = -value;
	}

	void setManualPitchOverride(bool aoa_override)
	{
		manualPitchOverride = aoa_override;
	}

	// landing gear related flaps:
	// flaps are controlled with landing gear lever as well,
	// gears go down -> trailing edge flaps go down
	// gear go up -> trailing edge flaps go up
	void setGearRelatedFlaps(bool up)
	{
		gearRelatedFlaps = up;
	}

	void initAirBrakeOff()
	{
		airbrakeDrag = 0;
		airbrakeSwitch = false;
		airbrakeActuator.m_current = 0;
		airbrakeActuator.m_commanded = 0;
	}
	void setAirbrakeON()
	{
		airbrakeSwitch = true;
		airbrakeActuator.m_commanded = 1;
	}
	void setAirbrakeOFF()
	{
		airbrakeSwitch = false;
		airbrakeActuator.m_commanded = 0;
	}
	void switchAirbrake()
	{
		airbrakeSwitch = !airbrakeSwitch;
	}
	void setIsGearDown(bool gearDown)
	{
		isGearDown = gearDown;
	}

	// right-side
	float getAirbrakeRSAngle() const
	{
		// TODO: value from degrees to percentages here

		// use same for both sides for now
		return (float)airbrakeActuator.m_current;
	}
	// left-side
	float getAirbrakeLSAngle() const
	{
		// TODO: value from degrees to percentages here

		// use same for both sides for now
		return (float)airbrakeActuator.m_current;
	}

	void updateAirBrake(const double totalVelocity_FPS, const double dynamicPressure_LBFT2, const double ps_LBFT2, const double frameTime)
	{
		// TODO: change values to degrees here

		// for now, just use frametime for rate of movement
		// (multiplier 1)

		// note: airbrake limit 60 degrees normally, 
		// 43 deg when landing gear down (prevent strike to runway)
		double maxAnglePCT = 1.0; // 60 deg
		if (isGearDown == true)
		{
			maxAnglePCT = 0.71; // ~43 deg
		}

		// TODO: if weight on wheel -> max opening
		// if gear down but no weight on wheel -> restricted
		// controlled by additional switch in cockpit?

		if (airbrakeSwitch == true)
		{
			// open to max allowed by limit
			airbrakeActuator.m_commanded = maxAnglePCT;
		}
		else
		{
			// close it
			airbrakeActuator.m_commanded = 0;
		}
		airbrakeActuator.updateFrame(frameTime);


		// after actuator move, calculate new drag at new position

		// TODO: switch to actual angles instead of percentages
		//double angle = cos(airbrakeActuator.m_current);

		// TEST!
		// just use full now for testing
		//double force = dynamicPressure_LBFT2 * 16.0 * cos(60) * 0.7;

		/* source: http://www.f-16.net/forum/viewtopic.php?t=11398
		Because landing is such a low speed, I did not bother to calculate those forces. 
		But to estimate the force on the speedbrake at landing, you can use the dynamic pressure at landing speed x speedbrake area x cos 60 deg x Cd

		dynamic pressure q ~ 125 lb/sq ft (from q/M^2 = 1480 lb/sq ft)
		area ~ 4 sq ft x 4
		cos 60 deg = .866
		Cd ~ .7

		total drag force ~ 1212 lb

		That is the total force (parallel to the fuselage centerline) on all four panels at landing speed. It is much less than 3 tons.
		*/

		//double force = dynamicPressure_LBFT2 * 16.0 * cos(60) * 0.7;

		if (airbrakeActuator.m_current > 0)
		{
			double CDAirbrake = airbrakeActuator.m_current * 0.7;
			airbrakeDrag = -(CDAirbrake * cos(F16::degtorad));

			//double pressureAreaFT2 = airbrakeArea_FT2 * dynamicPressure_LBFT2;
			//double airbrake_DEG = (airbrakeActuator.m_current * 60); // <- PCT to DEG
			//airbrakeDrag = -(0.7 * cos(airbrake_DEG));
		}
		else
		{
			airbrakeDrag = 0;
		}

	}

	/*
	bool initializeLeadingEdgeFlapController()
	{
	}
	*/



public:

	bool initialize(double dt)
	{
		if (simInitialized == true)
		{
			return true;
		}

		bool res = true;

		res &= pitchControl.initialize(dt);
		res &= rollControl.initialize(dt);
		res &= yawControl.initialize(dt);

		pitchControl.reset(dt);
		rollControl.reset(dt);
		yawControl.reset(dt);
		return res;
	}

	// before simulation starts
	void setCurrentState(double ax, double ay, double az)
	{
		bodyState.ay_world = ay;
	}


	// gather some values for use later
	void setBodyAxisState(double common_angle_of_attack, double common_angle_of_slide, double omegax, double omegay, double omegaz, double ax, double ay, double az)
	{
		//-------------------------------
		// Start of setting F-16 states
		//-------------------------------
		bodyState.alpha_DEG = common_angle_of_attack * F16::radiansToDegrees;
		bodyState.beta_DEG = common_angle_of_slide * F16::radiansToDegrees;
		bodyState.rollRate_RPS = omegax;
		bodyState.pitchRate_RPS = omegaz;
		bodyState.yawRate_RPS = -omegay;

		// note the change in coordinate system here..
		bodyState.accz = ay;
		bodyState.accy = az;
	}

	// after first frame is done
	void setInitialized()
	{
		simInitialized = true;
	}

	//---------------------------------------------
	//-----CONTROL DYNAMICS------------------------
	//---------------------------------------------
	void updateFrame(double frametime)
	{
		const double totalVelocity_FPS = pAtmos->getTotalVelocityFPS();

		// only place this is needed for now..
		const double ps_LBFT2 = pAtmos->getAmbientPressureLBFTSQ(); // (N/m^2) to (lb/ft^2)
		const double dynamicPressure_LBFT2 = pAtmos->getDynamicPressureLBFTSQ(); // LB/FT^2


		//if (airbrakeExtended != airbrakeSwitch)
		// -> actuator movement by frame step
		updateAirBrake(totalVelocity_FPS, dynamicPressure_LBFT2, ps_LBFT2, frametime);

		// Call the leading edge flap dynamics controller, this controller is based on dynamic pressure and angle of attack
		// and is completely automatic
		// Leading edge flap deflection (deg)
		flightSurface.leadingEdgeFlap_DEG = leadingedgeControl.leading_edge_flap_controller(simInitialized, dynamicPressure_LBFT2, ps_LBFT2, frametime);
		flightSurface.leadingEdgeFlap_PCT = limit(flightSurface.leadingEdgeFlap_DEG / 25.0, 0.0, 1.0);

		// Call the longitudinal (pitch) controller.  Takes the following inputs:
		// -Normalize long stick input
		// -Trimmed G offset
		// -Angle of attack (deg)
		// -Pitch rate (rad/sec)
		// -Differential command (from roll controller, not quite implemented yet)

		// TODO: roll controller should also calculate elevator angle for combined effects?
		// or pitch controller should calculate roll effect too?
		// -> check control laws, in addition to handling supersonic flutter

		double elevator_DEG_commanded = -(pitchControl.fcs_pitch_controller(longStickInput.getValue(), trimState.trimPitch, 0.0, dynamicPressure_LBFT2, frametime));
		// Call the servo dynamics model (not used as it causes high flutter in high speed situations, related to filtering and dt rate)
		flightSurface.elevator_DEG = elevator_DEG_commanded; //F16::ACTUATORS::elevator_actuator(F16::elevator_DEG_commanded,dt);
		flightSurface.elevator_DEG = limit(flightSurface.elevator_DEG, -25.0, 25.0);

		double aileron_DEG_commanded = (rollControl.fcs_roll_controller(latStickInput.getValue(), pitchControl.getLongStickForce(), trimState.trimRoll, dynamicPressure_LBFT2, frametime));
		flightSurface.aileron_DEG = aileron_DEG_commanded; //F16::ACTUATORS::aileron_actuator(F16::aileron_DEG_commanded,dt);
		flightSurface.aileron_DEG = limit(flightSurface.aileron_DEG, -21.5, 21.5);

		double rudder_DEG_commanded = yawControl.fcs_yaw_controller(pedInput.getValue(), trimState.trimYaw, pitchControl.getAlphaFiltered(), aileron_DEG_commanded, frametime);
		flightSurface.rudder_DEG = rudder_DEG_commanded; //F16::ACTUATORS::rudder_actuator(F16::rudder_DEG_commanded,dt);
		flightSurface.rudder_DEG = limit(flightSurface.rudder_DEG, -30.0, 30.0);

		// reuse in drawargs
		//flightSurface.leadingEdgeFlap_PCT = 
		flightSurface.aileron_PCT = flightSurface.aileron_DEG / 21.5;
		flightSurface.elevator_PCT = flightSurface.elevator_DEG / 25.0;
		flightSurface.rudder_PCT = flightSurface.rudder_DEG / 30.0;

		// Trailing edge flap deflection (deg)
		// Note that flaps should be controlled by landing gear level:
		// when gears go down flaps go down as well
		flightSurface.flap_DEG = flapControl.fcs_flap_controller(totalVelocity_FPS);
		flightSurface.flap_PCT = flightSurface.flap_DEG / 20.0;
	}

	double getAirbrakeDrag()
	{
		return airbrakeDrag;
	}

};

#endif // ifndef _F16FLIGHTCONTROLSYSTEM_H_
