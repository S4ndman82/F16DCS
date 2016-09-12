#ifndef _F16FCSAIRBRAKECONTROLLER_H_
#define _F16FCSAIRBRAKECONTROLLER_H_

#include <cmath>

#include "../UtilityFunctions.h"

#include "F16FcsCommon.h"
#include "F16Actuator.h"


class F16FcsAirbrakeController
{
protected:
	F16BodyState *bodyState;
	F16FlightSurface *flightSurface;

public:
	// note: airbrake limit different when landing gear down (prevent strike to runway)
	// cx_brk = 0.08, --coefficient, drag, breaks <- for airbrake?
	bool airbrakeSwitch; // switch status
	F16Actuator airbrakeActuator;

public:
	F16FcsAirbrakeController(F16BodyState *bs, F16FlightSurface *fs) :
		bodyState(bs),
		flightSurface(fs),
		airbrakeSwitch(false),
		airbrakeActuator(30.0, 0, 60.0) // <- check actuator rate
	{}
	~F16FcsAirbrakeController() {}

	void initAirBrakeOff()
	{
		airbrakeSwitch = false;
		airbrakeActuator.m_current = 0;
		airbrakeActuator.m_commanded = 0;
	}
	void setAirbrake(bool status)
	{
		airbrakeSwitch = status;
	}
	void toggleAirbrake()
	{
		airbrakeSwitch = !airbrakeSwitch;
	}

	bool initialize(double dt)
	{
		return true;
	}
	void reset(double dt)
	{
	}

	void fcsCommand(bool isGearDown)
	{
		// TODO: change values to degrees here

		// for now, just use frametime for rate of movement
		// (multiplier 1)

		// note: airbrake limit 60 degrees normally, 
		// 43 deg when landing gear down (prevent strike to runway)
		double maxAnglePCT = 60.0; // 60 deg
		if (isGearDown == true)
		{
			maxAnglePCT = 43.0; // ~43 deg
		}

		// TODO: if weight on wheel -> max opening
		// if gear down but no weight on wheel -> restricted
		// controlled by additional switch in cockpit?

		flightSurface->airbrake_Command = maxAnglePCT;
		if (airbrakeSwitch == true)
		{
			// open to max allowed by limit
			flightSurface->airbrake_Command = maxAnglePCT;
		}
		else
		{
			// close it
			flightSurface->airbrake_Command = 0;
		}
	}

	void updateFrame(double frametime) 
	{
		airbrakeActuator.m_commanded = flightSurface->airbrake_Command;
		airbrakeActuator.updateFrame(frametime);

		// just use same for both sides for now
		flightSurface->airbrake_Left_DEG = flightSurface->airbrake_Right_DEG = airbrakeActuator.m_current;

		flightSurface->airbrake_Right_PCT = flightSurface->airbrake_Right_DEG / 60.0;
		flightSurface->airbrake_Left_PCT = flightSurface->airbrake_Left_DEG / 60.0;
	}
};

#endif // ifndef _F16FCSAIRBRAKECONTROLLER_H_
