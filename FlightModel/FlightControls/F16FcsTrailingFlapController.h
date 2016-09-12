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

	// lowers flaps a few degrees when enabled?
	// check this
	bool isAirRefuelMode;

	// Passive flap schedule for the F-16...nominal for now from flight manual comments
	// below specific dynamic pressure (q) -> function as flaps,
	// otherwise only as ailerons.
	// Normally flaps are down when gear lever is down.
	// With alternate flaps switch, flaps are extended regardless of gear lever.
	double fcs_flap_controller(bool isGearUp, bool isAltFlaps, double airspeed_KTS)
	{
		const double tef_min = 0.0;
		const double tef_max = 20.0;

		// TODO: electrical bias on adjustment?

		// also bit of flaps in transonic speeds?
		// 

		/*
		// lower tef by some degrees?
		// triggered by refueling trap door?
		if (isAirRefuelMode == true)
		{
			return 5.0; // <- just some value for placeholder, figure out right one
		}
		*/

		// no "alt flaps" and lg is up -> no flap deflection
		if (isAltFlaps == false && isGearUp == true)
		{
			return tef_min;
		}

		// also: hydraulic pressure limit on startup/shutdown?

		// else if gear lever is down -> max flaps
		// else if alt flap switch -> max flaps

		// speed high enough -> no flap deflection
		if (airspeed_KTS > 370.0) // ~190m/s
		{
			// no deflection
			return tef_min;
		}

		// low speed -> full deflection
		if (airspeed_KTS < 240.0) // ~123m/s
		{
			// max deflection
			return tef_max;
		}

		// otherwise deflection is some value in between..
		//if ((airspeed_KTS >= 240.0) && (airspeed_KTS <= 370.0))
		double trailing_edge_flap_deflection = (1.0 - ((airspeed_KTS - 240.0) / (370.0 - 240.0))) * 20.0;
		return limit(trailing_edge_flap_deflection, tef_min, tef_max);
	}

public:
	F16FcsTrailingFlapController(F16BodyState *bs, F16FlightSurface *fs) :
		bodyState(bs),
		flightSurface(fs),
		actuator(10.0, 0, 20.0), // <- check adjustment rate
		isAirRefuelMode(false)
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
	void updateFrame(bool isGearUp, bool isAltFlaps, double airspeed_KTS, double qbarOverPs, double frametime)
	{
		// flaps in transonic speeds? 
		// -2 deg when qbarOverPs >= 1.008
		// 0..-2 deg when qbarOverPs >= 0.787 && qbarOverPs <= 1.008

		flightSurface->flap_Command = fcs_flap_controller(isGearUp, isAltFlaps, airspeed_KTS);
		actuator.commandMove(flightSurface->flap_Command);
		actuator.updateFrame(frametime);

		flightSurface->flap_DEG = actuator.m_current;
		flightSurface->flap_Right_PCT = actuator.m_current / 20.0;
		flightSurface->flap_Left_PCT = actuator.m_current / 20.0;
	}
};

#endif // ifndef _F16FCSTRAILINGFLAPCONTROLLER_H_
