#ifndef _F16FLIGHTCONTROLSYSTEM_H_
#define _F16FLIGHTCONTROLSYSTEM_H_

#include <cmath>

#include "F16Constants.h"
#include "UtilityFunctions.h"

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
// -> unique to AFTI modified version of F-16?
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

	// 25(deg)/sec
	F16Actuator			lefActuator; // symmetric

	// 20 rad/sec min (lowest frequency filter)
	// flaperon: aileron and flap functionality (trailing edge)
	F16Actuator		flaperonActuatorLeft;
	F16Actuator		flaperonActuatorRight;
	// elevon: elevator and aileron functionality (stabilizers)
	F16Actuator		elevonActuatorLeft;
	F16Actuator		elevonActuatorRight;

	F16Actuator		rudderActuator;
	F16Actuator		airbrakeActuator;

	// temporary until problem with integration is solved..
	// 
	F16Actuator		flapActuator;

	//Limiter<double>		flaperonLimiter;
	//Limiter<double>		htailLimiter;


	// when MPO pressed down, override AOA/G-limiter and direct control of horiz. tail
	bool manualPitchOverride;

	//bool isGearDown; // is landing gear down&locked
	//bool isGearUp; // is lg up&locked

	// flap position logic according to when landing gears are down:
	// flaps are controlled with landing gear lever as well,
	// gears go down -> trailing edge flaps go down
	// gear go up -> trailing edge flaps go up
	//bool gearLevelStatus;

	// if alternate flaps switch is used
	bool isAltFlaps;

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
		, longStickInput(-1.0, 1.0)
		, latStickInput(-1.0, 1.0)
		, pedInput(-1.0, 1.0)
		, pitchControl(&bodyState, &flightSurface, &trimState)
		, rollControl(&bodyState, &flightSurface, &trimState)
		, yawControl(&bodyState, &flightSurface, &trimState)
		, leadingedgeControl(&bodyState, &flightSurface)
		, flapControl(&bodyState, &flightSurface)
		, airbrakeControl(&bodyState, &flightSurface)
		, lefActuator(25, -2, 25) // <- FLCS diag
		, flaperonActuatorLeft(80, -23, 20) // <- FLCS diag, 21.5 flap limit in old code
		, flaperonActuatorRight(80, -23, 20) // <- FLCS diag, 21.5 flap limit in old code
		, elevonActuatorLeft(60, -25, 25) // <- FLCS diag
		, elevonActuatorRight(60, -25, 25) // <- FLCS diag
		, rudderActuator(120.0, -30.0, 30.0) // <- FLCS diag
		, airbrakeActuator(30.0, 0, 60.0) // <- check actuator rate
		, flapActuator(10.0, 0, 20.0) // temporary only until fixing integration problem
		//, flaperonLimiter(-20, 20) // deflection limit for both sides
		//, htailLimiter(-25, 25) // stab. deflection limits
		, manualPitchOverride(false)
		//, isGearDown(true)
		//, isGearUp(false)
		//, gearLevelStatus(false)
		, isAltFlaps(false)
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

	void toggleAltFlaps()
	{
		isAltFlaps = !isAltFlaps;
	}
	void setAltFlaps(bool status)
	{
		isAltFlaps = status;
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
		return (float)-flightSurface.flaperon_Right_PCT;
	}

	// left-side aileron draw argument
	float getAileronLSDraw() const
	{
		return (float)flightSurface.flaperon_Left_PCT;
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
	void controlCommands(double frametime)
	{
		double dynamicPressure_kNM2 = pAtmos->dynamicPressure / 1000.0; //for kN/m^2
		// stagnation pressure? (used to detect transonic speeds?)
		// dynamic pressure to static pressure ratio
		double qbarOverPs = pAtmos->getQcOverPs();

		// landing gear "down&locked" affects some logic
		bool isWoW = landingGear->isWoW();
		bool isGearDown = landingGear->isGearDownLocked();
		bool isGearUp = landingGear->isGearUpLocked();
		bool gearLevelStatus = landingGear->getGearLevelStatus();

		// TODO: affecting flaps logic when air refuel triggered
		//bool refuelingDoor = (airframe->getRefuelingDoorAngle() ? true : false);

		//if (airbrakeExtended != airbrakeSwitch)
		// -> actuator movement by frame step
		airbrakeControl.fcsCommand(isGearDown);

		// Call the leading edge flap dynamics controller, this controller is based on dynamic pressure and angle of attack
		// and is completely automatic
		// Leading edge flap deflection (deg)
		leadingedgeControl.fcsCommand(qbarOverPs, isWoW, frametime);

		// Call the longitudinal (pitch) controller.  Takes the following inputs:
		// -Normalize long stick input
		// -Trimmed G offset
		// -Angle of attack (deg)
		// -Pitch rate (rad/sec)
		// -Differential command (from roll controller, not quite implemented yet)

		// TODO: roll controller should also calculate elevator angle for combined effects?
		// or pitch controller should calculate roll effect too?
		// -> check control laws, in addition to handling supersonic flutter

		pitchControl.fcsCommand(longStickInput.getValue(), dynamicPressure_kNM2, manualPitchOverride, frametime);
		rollControl.fcsCommand(latStickInput.getValue(), pitchControl.getLongStickForce(), pAtmos->dynamicPressure, isGearUp, isAltFlaps);

		yawControl.fcsCommand(pedInput.getValue(), pitchControl.getAlphaFiltered());

		// TODO: combine flap control with aileron control commands

		// Trailing edge flap deflection (deg)
		// Note that flaps should be controlled by landing gear level:
		// when gears go down flaps go down as well
		flapControl.fcsCommand(isGearUp, isAltFlaps, pAtmos->getTotalVelocityKTS(), qbarOverPs);
	}


	// combined and differential commands of flight surfaces:
	// aileron assist from asymmetric stabilizer movement etc.
	void fcsMixer(double frametime)
	{
		// opposite direction in hta (compared to fl)

		// pitch was already calculated? -> just modify
		//flightSurface.elevator_DEG += flightSurface.elevon_DEG;

		// determine roll command for
		// ailerons (flaperons) and elevons (differential stabilizers),
		// actuator/servo dynamics
		//flightSurface->roll_Command = rollCommandGained;

		// this is symmetric command, adjusted by aileron command,
		// positive values -> more flaps (increase downwards)
		if (flightSurface.flap_Command > 0)
		{
			flightSurface.flap_Left_DEG = flightSurface.flap_Command;
			flightSurface.flap_Right_DEG = flightSurface.flap_Command;
			flightSurface.flap_Left_PCT = flightSurface.flap_Left_DEG / 20.0;
			flightSurface.flap_Right_PCT = flightSurface.flap_Right_DEG / 20.0;

			/*
			// one side stays at maximum, other side can lift
			flightSurface.flaperon_Left_Command = flightSurface.flap_Command;
			flightSurface.flaperon_Right_Command = flightSurface.flap_Command;

			// adjust by roll command
			flightSurface.flaperon_Left_Command += flightSurface.roll_Command;
			flightSurface.flaperon_Right_Command += flightSurface.roll_Command;
			*/
			flightSurface.flaperon_Left_Command = flightSurface.roll_Command;
			flightSurface.flaperon_Right_Command = flightSurface.roll_Command;
		}
		else
		{
			flightSurface.flap_Left_DEG = 0;
			flightSurface.flap_Right_DEG = 0;
			flightSurface.flap_Left_PCT = flightSurface.flap_Left_DEG / 20.0;
			flightSurface.flap_Right_PCT = flightSurface.flap_Right_DEG / 20.0;

			flightSurface.flaperon_Left_Command = flightSurface.roll_Command;
			flightSurface.flaperon_Right_Command = flightSurface.roll_Command;
		}

		// TODO:
		// aileron-rudder interconnect handling
		// 

		// start with symmetric command
		flightSurface.elevon_Left_Command = -flightSurface.pitch_Command;
		flightSurface.elevon_Right_Command = -flightSurface.pitch_Command;

		// TODO: combine with aileron effect, try with equal constant now (modify pitch)
		//flightSurface.elevon_Left_Command += flightSurface.flaperon_Left_Command*0.294;
		//flightSurface.elevon_Right_Command += flightSurface.flaperon_Right_Command*0.294;

	}

	// when preparing to land (wheels out),
	// counteract increased drag
	void landingGains(double frametime) {}
	// gains for gun usage
	void gunCompensation(double frametime) {}
	// gains for bombing mode
	void bombingMode(double frametime) {}

	void updateFrame(double frametime)
	{
		// determine controller commands
		controlCommands(frametime);

		// combinations and differential commands in mixer (to actuators)
		fcsMixer(frametime);

		updateCommand(frametime);
	}

	void updateCommand(double frametime)
	{
		lefActuator.commandMove(flightSurface.leadingEdgeFlap_Command);
		lefActuator.updateFrame(frametime);
		flightSurface.leadingEdgeFlap_DEG = lefActuator.m_current; // <- lef symmetric
		flightSurface.leadingEdgeFlap_Left_PCT = flightSurface.leadingEdgeFlap_Right_PCT = lefActuator.getCurrentPCT();

		// TODO: integrated command
		elevonActuatorLeft.commandMove(flightSurface.elevon_Left_Command);
		elevonActuatorRight.commandMove(flightSurface.elevon_Right_Command);
		elevonActuatorLeft.updateFrame(frametime);
		elevonActuatorRight.updateFrame(frametime);
		flightSurface.elevator_Left_DEG = elevonActuatorLeft.m_current;
		flightSurface.elevator_Right_DEG = elevonActuatorRight.m_current;
		flightSurface.elevator_Left_PCT = elevonActuatorLeft.getCurrentPCT();
		flightSurface.elevator_Right_PCT = elevonActuatorRight.getCurrentPCT();

		// TODO: 
		flaperonActuatorLeft.commandMove(flightSurface.flaperon_Left_Command);
		flaperonActuatorRight.commandMove(flightSurface.flaperon_Right_Command);
		flaperonActuatorLeft.updateFrame(frametime);
		flaperonActuatorRight.updateFrame(frametime);
		flightSurface.flaperon_Left_DEG = flaperonActuatorLeft.m_current;
		flightSurface.flaperon_Right_DEG = flaperonActuatorRight.m_current;

		flightSurface.flaperon_Left_PCT = flaperonActuatorLeft.getCurrentPCT();
		flightSurface.flaperon_Right_PCT = flaperonActuatorRight.getCurrentPCT();


		// TODO: roll combination in mixer
		// TODO: figure out problem with integration (command controller vs. aero code)
		flapActuator.commandMove(flightSurface.flap_Command);
		flapActuator.updateFrame(frametime);
		flightSurface.flap_Left_DEG = flightSurface.flap_Right_DEG = flapActuator.m_current;
		flightSurface.flap_Left_PCT = flightSurface.flap_Right_PCT = flapActuator.getCurrentPCT();


		rudderActuator.commandMove(flightSurface.rudder_Command);
		rudderActuator.updateFrame(frametime);
		flightSurface.rudder_DEG = rudderActuator.m_current;
		flightSurface.rudder_PCT = rudderActuator.getCurrentPCT();

		airbrakeActuator.m_commanded = flightSurface.airbrake_Command;
		airbrakeActuator.updateFrame(frametime);
		flightSurface.airbrake_Left_DEG = flightSurface.airbrake_Right_DEG = airbrakeActuator.m_current;
		flightSurface.airbrake_Left_PCT = flightSurface.airbrake_Right_PCT = airbrakeActuator.getCurrentPCT();

#if 0
		outputDebug();
#endif

	}

	void outputDebug()
	{
		wchar_t dbgmsg[1024] = { 0 };
		swprintf(dbgmsg, 1024, L"F16:: flLp: %f flRp: %f flLd: %f flRd: %f \r\n", 
			flightSurface.flaperon_Left_PCT, flightSurface.flaperon_Right_PCT, 
			flightSurface.flaperon_Left_DEG, flightSurface.flaperon_Right_DEG);
		::OutputDebugString(dbgmsg);
	}

};

#endif // ifndef _F16FLIGHTCONTROLSYSTEM_H_
