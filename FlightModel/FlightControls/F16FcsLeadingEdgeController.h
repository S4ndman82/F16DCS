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
	double		leading_edge_flap_integrated_gained;
	double		leading_edge_flap_integrated_gained_biased;

	/*
	F16Actuator		leadingedgeActuatorLeft;
	F16Actuator		leadingedgeActuatorRight;
	*/

	// is in automatic operation or locked in position
	bool isAuto;

public:
	F16FcsLeadingEdgeController(F16BodyState *bs, F16FlightSurface *fs) :
		bodyState(bs),
		flightSurface(fs),
		leading_edge_flap_integral(0),
		leading_edge_flap_integrated(0),
		leading_edge_flap_rate(0),
		leading_edge_flap_integrated_gained(0),
		leading_edge_flap_integrated_gained_biased(0),
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

	// Controller for the leading edge flaps
	double leading_edge_flap_controller(bool simInitialized, double qbarOverPs, double frameTime)
	{
		if (!(simInitialized))
		{
			leading_edge_flap_integral = -bodyState->alpha_DEG;
			leading_edge_flap_integrated = leading_edge_flap_integral + 2 * bodyState->alpha_DEG;
			return leading_edge_flap_integral;
		}

		leading_edge_flap_rate = (bodyState->alpha_DEG - leading_edge_flap_integrated) * 7.25;
		leading_edge_flap_integral += (leading_edge_flap_rate * frameTime);

		leading_edge_flap_integrated = leading_edge_flap_integral + bodyState->alpha_DEG * 2.0;
		leading_edge_flap_integrated_gained = leading_edge_flap_integrated * 1.38;
		leading_edge_flap_integrated_gained_biased = leading_edge_flap_integrated_gained + 1.45 - (9.05 * qbarOverPs);

		flightSurface->leadingEdgeFlap_DEG = leading_edge_flap_integrated_gained_biased;
		flightSurface->leadingEdgeFlap_PCT = limit(leading_edge_flap_integrated_gained_biased / 25.0, 0.0, 1.0);

		return leading_edge_flap_integrated_gained_biased;
	}

	void updateFrame(double frametime) {}
};

#endif // ifndef _F16FCSLEADINGEDGECONTROLLER_H_
