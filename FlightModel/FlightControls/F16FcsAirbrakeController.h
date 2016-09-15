#ifndef _F16FCSAIRBRAKECONTROLLER_H_
#define _F16FCSAIRBRAKECONTROLLER_H_

#include <cmath>

#include "UtilityFunctions.h"

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

public:
	F16FcsAirbrakeController(F16BodyState *bs, F16FlightSurface *fs) :
		bodyState(bs),
		flightSurface(fs),
		airbrakeSwitch(false)
	{}
	~F16FcsAirbrakeController() {}

	void initAirBrakeOff()
	{
		airbrakeSwitch = false;
	}
	void setAirbrake(bool status)
	{
		airbrakeSwitch = status;
	}
	void toggleAirbrake()
	{
		airbrakeSwitch = !airbrakeSwitch;
	}

	void fcsCommand(bool isGearDown)
	{
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
};

#endif // ifndef _F16FCSAIRBRAKECONTROLLER_H_
