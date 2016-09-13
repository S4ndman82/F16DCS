#ifndef _F16FCSLEADINGEDGECONTROLLER_H_
#define _F16FCSLEADINGEDGECONTROLLER_H_

#include <cmath>

#include "../UtilityFunctions.h"

#include "F16FcsCommon.h"
#include "F16Actuator.h"

class F16FcsLeadingEdgeController
{
protected:
	F16BodyState *bodyState;
	F16FlightSurface *flightSurface;

	double		leading_edge_flap_integral;
	double		leading_edge_flap_integrated;
	double		leading_edge_flap_rate;
	double		leading_edge_flap_final;

	Limiter<double>		lefLimiter;

	// 25(deg)/sec
	F16Actuator			lefActuator; // symmetric

	// is in automatic operation or locked in position
	bool isAuto;

public:
	F16FcsLeadingEdgeController(F16BodyState *bs, F16FlightSurface *fs) :
		bodyState(bs),
		flightSurface(fs),
		leading_edge_flap_integral(0),
		leading_edge_flap_integrated(0),
		leading_edge_flap_rate(0),
		leading_edge_flap_final(0),
		lefLimiter(-2, 25),
		lefActuator(7.25, -2, 25),
		isAuto(true)
	{}
	~F16FcsLeadingEdgeController() {}

	bool initialize(double dt)
	{
		return true;
	}
	void reset(double dt)
	{
	}
	void setAutoLocked(bool onoff)
	{
		isAuto = onoff;
	}

	// Controller for the leading edge flaps
	// symmetrical, as function of alpha and mach number
	void fcsCommand(const double qbarOverPs, const bool isWoW, const double frameTime)
	{
		/*
		// TODO: fix rest of handling for this too
		// actuator movement needs support too
		if (isWoW == true)
		{
			// weight on wheels -> lock to -2 degrees (up)
			return -2.0;
		}
		*/
		// also add handling in transonic speeds..

		leading_edge_flap_rate = (bodyState->alpha_DEG - leading_edge_flap_integrated) * 7.25;
		leading_edge_flap_integral += (leading_edge_flap_rate * frameTime);

		leading_edge_flap_integrated = leading_edge_flap_integral + bodyState->alpha_DEG * 2.0;

		double lef_gained = leading_edge_flap_integrated * 1.38;
		double press = (9.05 * qbarOverPs) + 1.45;
		leading_edge_flap_final = lef_gained - press;

		flightSurface->leadingEdgeFlap_Command = lefLimiter.limit(leading_edge_flap_final);
	}

	void updateFrame(double frameTime)
	{
		// actuator movement here..
		//lefActuator.commandMove(flightSurface->leadingEdgeFlap_DEG);
		//lefActuator.updateFrame(frameTime);
		flightSurface->leadingEdgeFlap_DEG = flightSurface->leadingEdgeFlap_Command;

		// this is bugged when there's weight on wheels (-2 up)
		double lef_PCT = limit(flightSurface->leadingEdgeFlap_DEG / 25.0, 0.0, 1.0);
		flightSurface->leadingEdgeFlap_Right_PCT = lef_PCT;
		flightSurface->leadingEdgeFlap_Left_PCT = lef_PCT;
	}
};

#endif // ifndef _F16FCSLEADINGEDGECONTROLLER_H_
