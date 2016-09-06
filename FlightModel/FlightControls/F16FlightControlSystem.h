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

#include "LandingGear/F16LandingGear.h"

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
	F16LandingGear *landingGear; // affecting some logic

	F16BodyState bodyState;
	F16FlightSurface flightSurface;
	F16TrimState trimState;

protected:

	// TODO: get rid of this
	bool		simInitialized;


	bool isGearDown; // is landing gear down


	// Pitch controller variables
	AnalogInput		longStickInput; // pitch normalized
	AnalogInput		latStickInput; // bank normalized
	AnalogInput		pedInput;		// Pedal input command normalized (-1 to 1)

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
	bool gearLevelStatus;


public:
	F16FlightControls(F16Atmosphere *atmos, F16LandingGear *lgear)
		: pAtmos(atmos)
		, landingGear(lgear)
		, bodyState()
		, flightSurface()
		//, trimState(-0.3, 0, 0) // <- -0.3 pitch trim, RSS compensation?
		, trimState(0, 0, 0)
		, simInitialized(false)
		, isGearDown(true)
		, longStickInput(-1.0, 1.0)
		, latStickInput(-1.0, 1.0)
		, pedInput(-1.0, 1.0)
		, pitchControl(&bodyState, &flightSurface)
		, rollControl(&bodyState, &flightSurface)
		, yawControl(&bodyState, &flightSurface)
		, leadingedgeControl(&bodyState, &flightSurface)
		, flapControl(&bodyState, &flightSurface)
		, airbrakeControl(&bodyState, &flightSurface)
		, manualPitchOverride(false)
		, gearLevelStatus(false)
	{}
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

	void initAirBrakeOff()
	{
		airbrakeControl.initAirBrakeOff();
	}
	void setAirbrakeON()
	{
		airbrakeControl.setAirbrake(true);
	}
	void setAirbrakeOFF()
	{
		airbrakeControl.setAirbrake(false);
	}
	void switchAirbrake()
	{
		airbrakeControl.toggleAirbrake();
	}

	// right-side
	float getAirbrakeRSAngle() const
	{
		// TODO: value from degrees to percentages here

		// use same for both sides for now
		return (float)airbrakeControl.airbrakeActuator.m_current;
	}
	// left-side
	float getAirbrakeLSAngle() const
	{
		// TODO: value from degrees to percentages here

		// use same for both sides for now
		return (float)airbrakeControl.airbrakeActuator.m_current;
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

	//---------------------------------------------
	//-----CONTROL DYNAMICS------------------------
	//---------------------------------------------
	void updateFrame(double frametime)
	{
		//const double totalVelocity_FPS = pAtmos->getTotalVelocityFPS();

		// only place this is needed for now..
		const double ps_LBFT2 = pAtmos->getAmbientPressureLBFTSQ(); // (N/m^2) to (lb/ft^2)
		const double dynamicPressure_LBFT2 = pAtmos->getDynamicPressureLBFTSQ(); // LB/FT^2

		// landing gear "down&locked" affects some logic
		isGearDown = landingGear->isGearDownLocked();
		gearLevelStatus = landingGear->getGearLevelStatus();

		//if (airbrakeExtended != airbrakeSwitch)
		// -> actuator movement by frame step
		airbrakeControl.updateAirBrake(isGearDown, dynamicPressure_LBFT2, ps_LBFT2, frametime);

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

		flightSurface.elevator_DEG = -(pitchControl.fcs_pitch_controller(longStickInput.getValue(), trimState.trimPitch, 0.0, dynamicPressure_LBFT2, frametime));
		flightSurface.elevator_DEG = limit(flightSurface.elevator_DEG, -25.0, 25.0);

		flightSurface.aileron_DEG = (rollControl.fcs_roll_controller(latStickInput.getValue(), pitchControl.getLongStickForce(), trimState.trimRoll, dynamicPressure_LBFT2, frametime));
		flightSurface.aileron_DEG = limit(flightSurface.aileron_DEG, -21.5, 21.5);

		flightSurface.rudder_DEG = yawControl.fcs_yaw_controller(pedInput.getValue(), trimState.trimYaw, pitchControl.getAlphaFiltered(), flightSurface.aileron_DEG, frametime);
		flightSurface.rudder_DEG = limit(flightSurface.rudder_DEG, -30.0, 30.0);

		// reuse in drawargs
		//flightSurface.leadingEdgeFlap_PCT = 
		flightSurface.aileron_PCT = flightSurface.aileron_DEG / 21.5;
		flightSurface.elevator_PCT = flightSurface.elevator_DEG / 25.0;
		flightSurface.rudder_PCT = flightSurface.rudder_DEG / 30.0;

		// Trailing edge flap deflection (deg)
		// Note that flaps should be controlled by landing gear level:
		// when gears go down flaps go down as well
		flightSurface.flap_DEG = flapControl.fcs_flap_controller(gearLevelStatus, pAtmos->getTotalVelocityKTS());
		flightSurface.flap_PCT = flightSurface.flap_DEG / 20.0;
	}

	double getAirbrakeDrag()
	{
		return airbrakeControl.airbrakeDrag;
	}

	// after first frame is done
	// TODO: get rid of this
	void setInitialized()
	{
		simInitialized = true;
	}

};

#endif // ifndef _F16FLIGHTCONTROLSYSTEM_H_
