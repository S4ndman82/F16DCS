#ifndef _F16FCSTRAILINGFLAPCONTROLLER_H_
#define _F16FCSTRAILINGFLAPCONTROLLER_H_

#include <cmath>

#include "../UtilityFunctions.h"

#include "F16FcsCommon.h"
#include "F16Actuator.h"

class F16FcsTrailingFlapController
{
protected:
	F16BodyState *bodyState;
	F16FlightSurface *flightSurface;

	// TEF actually does not have own actuators 
	// but use the "flaperon" actuators.
	// This is only to estimate some "gain" of position movement.
	// Bit of a hack now really, might need to be in roll controller..
	F16Actuator actuator;

	// is alternate flaps mode (extend regardless of landing gear lever)
	bool isAltFlaps;

	// Passive flap schedule for the F-16...nominal for now from flight manual comments
	// below specific dynamic pressure (q) -> function as flaps,
	// otherwise only as ailerons
	double fcs_flap_controller(bool gearLevelUp, double airspeed_KTS)
	{
		const double tef_min = 0.0;
		const double tef_max = 20.0;

		// if gear lever is down -> max flaps
		// if alt flap switch -> max flaps
		if (isAltFlaps == true || gearLevelUp == false)
		{
			return tef_max;
		}

		if (airspeed_KTS < 240.0)
		{
			return tef_max;
		}
		else if ((airspeed_KTS >= 240.0) && (airspeed_KTS <= 370.0))
		{
			double trailing_edge_flap_deflection = (1.0 - ((airspeed_KTS - 240.0) / (370.0 - 240.0))) * 20.0;
			return limit(trailing_edge_flap_deflection, tef_min, tef_max);
		}
		else
		{
			return tef_min;
		}
	}

public:
	F16FcsTrailingFlapController(F16BodyState *bs, F16FlightSurface *fs) :
		bodyState(bs),
		flightSurface(fs),
		actuator(10.0, 0, 20.0),
		isAltFlaps(false)
	{}
	~F16FcsTrailingFlapController() {}

	bool initialize(double dt)
	{
		return true;
	}
	void reset(double dt)
	{
	}

	// Trailing edge flap deflection (deg)
	// Note that flaps should be controlled by landing gear level:
	// when gears go down flaps go down as well.
	//
	// In normal flight, flaps are used like normal ailerons.
	//
	void updateFrame(bool gearLevelUp, double airspeed_KTS, double frametime)
	{
		flightSurface->flap_Command = fcs_flap_controller(gearLevelUp, airspeed_KTS);
		actuator.commandMove(flightSurface->flap_Command);
		actuator.updateFrame(frametime);

		flightSurface->flap_DEG = actuator.m_current;
		flightSurface->flap_Right_PCT = actuator.m_current / 20.0;
		flightSurface->flap_Left_PCT = actuator.m_current / 20.0;
	}
};

#endif // ifndef _F16FCSTRAILINGFLAPCONTROLLER_H_
