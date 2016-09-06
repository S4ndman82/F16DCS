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

public:
	F16FcsTrailingFlapController(F16BodyState *bs, F16FlightSurface *fs) :
		bodyState(bs),
		flightSurface(fs)
	{}
	~F16FcsTrailingFlapController() {}

	bool initialize(double dt)
	{
		return true;
	}
	void reset(double dt)
	{
	}

	// Passive flap schedule for the F-16...nominal for now from flight manual comments
	double fcs_flap_controller(bool gearLevelUp, double airspeed_KTS)
	{
		double trailing_edge_flap_deflection = 0.0;

		/*
		// TODO: if gear lever is down -> max flaps?
		if (gearLevelUp == false)
		{
			return 20.0;
		}
		*/

		if (airspeed_KTS < 240.0)
		{
			trailing_edge_flap_deflection = 20.0;
		}
		else if ((airspeed_KTS >= 240.0) && (airspeed_KTS <= 370.0))
		{
			trailing_edge_flap_deflection = (1.0 - ((airspeed_KTS - 240.0) / (370.0 - 240.0))) * 20.0;
		}
		else
		{
			//trailing_edge_flap_deflection = (1.0 - ((airspeed_KTS - 240.0) / (370.0 - 240.0))) * 20.0;
			trailing_edge_flap_deflection = 0.0;
		}

		trailing_edge_flap_deflection = limit(trailing_edge_flap_deflection, 0.0, 20.0);
		return trailing_edge_flap_deflection;
	}

	void updateFrame(double frametime) {}
};

#endif // ifndef _F16FCSTRAILINGFLAPCONTROLLER_H_
