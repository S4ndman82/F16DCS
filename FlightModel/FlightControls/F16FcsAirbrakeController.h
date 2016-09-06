#ifndef _F16FCSAIRBRAKECONTROLLER_H_
#define _F16FCSAIRBRAKECONTROLLER_H_

#include <cmath>

#include "../UtilityFunctions.h"

#include "F16FcsCommon.h"

class F16FcsAirbrakeController
{
protected:
	F16BodyState *bodyState;
	F16FlightSurface *flightSurface;

public:
	F16FcsAirbrakeController(F16BodyState *bs, F16FlightSurface *fs) :
		bodyState(bs),
		flightSurface(fs)
	{}
	~F16FcsAirbrakeController() {}

	bool initialize(double dt)
	{
		return true;
	}
	void reset(double dt)
	{
	}

	void updateFrame(double frametime) {}
};

#endif // ifndef _F16FCSAIRBRAKECONTROLLER_H_
