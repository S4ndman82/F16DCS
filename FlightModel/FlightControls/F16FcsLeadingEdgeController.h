#ifndef _F16FCSLEADINGEDGECONTROLLER_H_
#define _F16FCSLEADINGEDGECONTROLLER_H_

#include <cmath>

#include "../UtilityFunctions.h"

#include "F16FcsCommon.h"

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

public:
	F16FcsLeadingEdgeController(F16BodyState *bs, F16FlightSurface *fs) :
		bodyState(bs),
		flightSurface(fs),
		leading_edge_flap_integral(0),
		leading_edge_flap_integrated(0),
		leading_edge_flap_rate(0),
		leading_edge_flap_integrated_gained(0),
		leading_edge_flap_integrated_gained_biased(0)
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
	double leading_edge_flap_controller(bool simInitialized, double dynamicPressure_FTLB, double staticPressure_FTLB, double frameTime)
	{
		double qbarOverPs = dynamicPressure_FTLB / staticPressure_FTLB;

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

		return leading_edge_flap_integrated_gained_biased;
	}

	void updateFrame(double frametime) {}
};

#endif // ifndef _F16FCSLEADINGEDGECONTROLLER_H_