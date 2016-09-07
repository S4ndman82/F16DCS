#ifndef _F16FCSTRAILINGFLAPCONTROLLER_H_
#define _F16FCSTRAILINGFLAPCONTROLLER_H_

#include <cmath>

#include "../UtilityFunctions.h"

#include "F16FcsCommon.h"

class F16FcsTrailingFlapController
{
protected:
	F16BodyState *bodyState;
	F16FlightSurface *flightSurface;

	// is alternate flaps mode (extend regardless of landing gear lever)
	bool isAltFlaps;

	// Passive flap schedule for the F-16...nominal for now from flight manual comments
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
		double tef_DEG = fcs_flap_controller(gearLevelUp, airspeed_KTS);

		flightSurface->flap_DEG = tef_DEG;
		flightSurface->flap_PCT = tef_DEG / 20.0;
	}
};

#endif // ifndef _F16FCSTRAILINGFLAPCONTROLLER_H_
