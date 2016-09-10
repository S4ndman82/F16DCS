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
#include "Airframe/F16Airframe.h"

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

// TODO:
// - separate command channels with different processing rates?
// -> asynchronous logic of the actual control system? "skew" of channels
//
class F16FlightControls
{
public:
	F16Atmosphere *pAtmos;
	F16LandingGear *landingGear; // affecting some logic
	F16Airframe *airframe;

	F16BodyState bodyState;
	F16FlightSurface flightSurface;
	F16TrimState trimState;

protected:

	// TODO: get rid of this
	bool		simInitialized;


	bool isGearDown; // is landing gear down&locked
	bool isGearUp; // is lg up&locked


	// Pitch controller variables
	AnalogInput		longStickInput; // pitch normalized
	AnalogInput		latStickInput; // bank normalized
	AnalogInput		pedInput;		// Pedal input command normalized (-1 to 1)

	F16FcsPitchController pitchControl;
	F16FcsRollController rollControl;
	F16FcsYawController yawControl;
	F16FcsLeadingEdgeController leadingedgeControl;
	F16FcsTrailingFlapController flapControl;
	F16FcsAirbrakeController airbrakeControl;

	// 20 rad/sec min (lowest frequency filter)
	// flaperon: aileron and flap functionality (trailing edge)
	F16Actuator		flaperonActuatorLeft;
	F16Actuator		flaperonActuatorRight;
	// elevon: elevator and aileron functionality (stabilizers)
	F16Actuator		elevonActuatorLeft;
	F16Actuator		elevonActuatorRight;
	F16Actuator		rudderActuator;

	Limiter<double>		flaperonLimiter;
	Limiter<double>		htailLimiter;
	Limiter<double>		rudderLimiter;


	// when MPO pressed down, override AOA/G-limiter and direct control of horiz. tail
	bool manualPitchOverride;

	// flap position logic according to when landing gears are down:
	// flaps are controlled with landing gear lever as well,
	// gears go down -> trailing edge flaps go down
	// gear go up -> trailing edge flaps go up
	bool gearLevelStatus;

	/*
	enum EGainConstants
	{
		N2 = 0.38,
		N3 = 0.70,
		N5 = 10.00,
		N8 = 14.40,
		N14 = 7.20,
		N23 = 0.50,
		N24 = 0.67,
		N25 = 2.50,
		N30 = 20.00
	};
	*/

public:
	F16FlightControls(F16Atmosphere *atmos, F16LandingGear *lgear, F16Airframe *aframe)
		: pAtmos(atmos)
		, landingGear(lgear)
		, airframe(aframe)
		, bodyState()
		, flightSurface()
		//, trimState(-0.3, 0, 0) // <- -0.3 pitch trim, RSS compensation?
		, trimState(0, 0, 0)
		, simInitialized(false)
		, isGearDown(true)
		, isGearUp(false)
		, longStickInput(-1.0, 1.0)
		, latStickInput(-1.0, 1.0)
		, pedInput(-1.0, 1.0)
		, pitchControl(&bodyState, &flightSurface, &trimState)
		, rollControl(&bodyState, &flightSurface, &trimState)
		, yawControl(&bodyState, &flightSurface, &trimState)
		, leadingedgeControl(&bodyState, &flightSurface)
		, flapControl(&bodyState, &flightSurface)
		, airbrakeControl(&bodyState, &flightSurface)
		, flaperonActuatorLeft(1) // <- placeholder value
		, flaperonActuatorRight(1) // <- placeholder value
		, elevonActuatorLeft(1) // <- placeholder value
		, elevonActuatorRight(1) // <- placeholder value
		, rudderActuator(1.0, -30.0, 30.0)
		, flaperonLimiter(-20, 20) // deflection limit for both sides
		, htailLimiter(-25, 25) // stab. deflection limits
		, rudderLimiter(-30, 30) // deflection limit
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

	void setManualPitchOverride(float aoa_override)
	{
		if (aoa_override > 0)
		{
			manualPitchOverride = true;
		}
		else
		{
			manualPitchOverride = false;
		}
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

	// right-side leading-edge flap draw argument
	float getLefRSDraw() const
	{
		return (float)flightSurface.leadingEdgeFlap_Right_PCT;
	}
	// left-side leading-edge flap draw argument
	float getLefLSDraw() const
	{
		return (float)flightSurface.leadingEdgeFlap_Left_PCT;
	}

	// right-side trailing-edge flap draw argument
	float getFlapRSDraw() const
	{
		return (float)flightSurface.flap_Right_PCT;
	}
	// left-side trailing-edge flap draw argument
	float getFlapLSDraw() const
	{
		return (float)flightSurface.flap_Left_PCT;
	}

	// right-side aileron draw argument
	float getAileronRSDraw() const
	{
		return (float)-flightSurface.aileron_Right_PCT;
	}

	// left-side aileron draw argument
	float getAileronLSDraw() const
	{
		return (float)flightSurface.aileron_Left_PCT;
	}

	// right-side elevator draw argument
	float getElevatorRSDraw() const
	{
		return (float)-flightSurface.elevator_Right_PCT;
	}

	// left-side elevator draw argument
	float getElevatorLSDraw() const
	{
		return (float)-flightSurface.elevator_Left_PCT;
	}

	// rudder draw argument
	float getRudderDraw() const
	{
		return (float)flightSurface.rudder_PCT;
	}

	// right-side draw argument
	float getAirbrakeRSDraw() const
	{
		return (float)flightSurface.airbrake_Right_PCT;
	}
	// left-side draw argument
	float getAirbrakeLSDraw() const
	{
		return (float)flightSurface.airbrake_Left_PCT;
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
		double dynamicPressure_kNM2 = pAtmos->dynamicPressure / 1000.0; //for kN/m^2
		// stagnation pressure? (used to detect transonic speeds?)
		double qbarOverPs = pAtmos->dynamicPressure / pAtmos->ambientPressure;

		// landing gear "down&locked" affects some logic
		isGearDown = landingGear->isGearDownLocked();
		isGearUp = landingGear->isGearUpLocked();
		gearLevelStatus = landingGear->getGearLevelStatus();

		// TODO: affecting flaps logic when air refuel triggered
		//bool refuelingDoor = (airframe->getRefuelingDoorAngle() ? true : false);

		//if (airbrakeExtended != airbrakeSwitch)
		// -> actuator movement by frame step
		airbrakeControl.updateAirBrake(isGearDown, pAtmos->dynamicPressure, frametime);

		// Call the leading edge flap dynamics controller, this controller is based on dynamic pressure and angle of attack
		// and is completely automatic
		// Leading edge flap deflection (deg)
		leadingedgeControl.updateFrame(qbarOverPs, frametime);

		// Call the longitudinal (pitch) controller.  Takes the following inputs:
		// -Normalize long stick input
		// -Trimmed G offset
		// -Angle of attack (deg)
		// -Pitch rate (rad/sec)
		// -Differential command (from roll controller, not quite implemented yet)

		// TODO: roll controller should also calculate elevator angle for combined effects?
		// or pitch controller should calculate roll effect too?
		// -> check control laws, in addition to handling supersonic flutter

		pitchControl.fcs_pitch_controller(longStickInput.getValue(), dynamicPressure_kNM2, manualPitchOverride, frametime);
		rollControl.fcs_roll_controller(latStickInput.getValue(), pitchControl.getLongStickForce(), pAtmos->dynamicPressure, frametime);
		yawControl.fcs_yaw_controller(pedInput.getValue(), pitchControl.getAlphaFiltered(), frametime);

		// TODO: combine flap control with aileron control commands

		// Trailing edge flap deflection (deg)
		// Note that flaps should be controlled by landing gear level:
		// when gears go down flaps go down as well
		flapControl.updateFrame(isGearUp, pAtmos->getTotalVelocityKTS(), qbarOverPs, frametime);

		// combinations and differential commands in mixer (to actuators)
		fcsMixer(frametime);
	}

	// combined and differential commands of flight surfaces:
	// aileron assist from asymmetric stabilizer movement etc.
	void fcsMixer(double frametime)
	{
		// rough test only now, need to fix later
		flightSurface.elevon_DEG = flightSurface.aileron_DEG*0.294;

		// opposite direction in hta (compared to fl)

		// pitch was already calculated? -> just modify
		//flightSurface.elevator_DEG += flightSurface.elevon_DEG;

		// determine roll command for
		// ailerons (flaperons) and elevons (differential stabilizers),
		// actuator/servo dynamics
		//flightSurface->roll_Command = rollCommandGained;

		// if trailing edge flaps are used, only adjust instead of full control?
		if (flightSurface.flap_Command > 0)
		{
			// one side stays at maximum, other side can lift
		}


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
		leadingedgeControl.setInitialized();
	}

};

#endif // ifndef _F16FLIGHTCONTROLSYSTEM_H_
