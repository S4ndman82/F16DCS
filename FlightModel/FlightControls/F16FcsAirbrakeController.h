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
	double airbrakeDrag;

public:
	F16FcsAirbrakeController(F16BodyState *bs, F16FlightSurface *fs) :
		bodyState(bs),
		flightSurface(fs),
		airbrakeSwitch(false),
		airbrakeActuator(1.0, 0, 1.0), //
		airbrakeDrag(0)
	{}
	~F16FcsAirbrakeController() {}

	void initAirBrakeOff()
	{
		airbrakeDrag = 0;
		airbrakeSwitch = false;
		airbrakeActuator.m_current = 0;
		airbrakeActuator.m_commanded = 0;
	}
	void setAirbrake(bool status)
	{
		airbrakeSwitch = status;
		if (airbrakeSwitch == true)
		{
			airbrakeActuator.m_commanded = 1;
		}
		else
		{
			airbrakeActuator.m_commanded = 0;
		}
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

	void updateAirBrake(bool isGearDown, const double dynamicPressure_NM2, const double frameTime)
	{
		// TODO: change values to degrees here

		// for now, just use frametime for rate of movement
		// (multiplier 1)

		// note: airbrake limit 60 degrees normally, 
		// 43 deg when landing gear down (prevent strike to runway)
		double maxAnglePCT = 1.0; // 60 deg
		if (isGearDown == true)
		{
			maxAnglePCT = 0.71; // ~43 deg
		}

		// TODO: if weight on wheel -> max opening
		// if gear down but no weight on wheel -> restricted
		// controlled by additional switch in cockpit?

		if (airbrakeSwitch == true)
		{
			// open to max allowed by limit
			airbrakeActuator.m_commanded = maxAnglePCT;
		}
		else
		{
			// close it
			airbrakeActuator.m_commanded = 0;
		}
		airbrakeActuator.updateFrame(frameTime);


		// after actuator move, calculate new drag at new position

		// TODO: switch to actual angles instead of percentages
		//double angle = cos(airbrakeActuator.m_current);

		// TEST!
		// just use full now for testing
		//double force = dynamicPressure_LBFT2 * 16.0 * cos(60) * 0.7;

		/* source: http://www.f-16.net/forum/viewtopic.php?t=11398
		Because landing is such a low speed, I did not bother to calculate those forces.
		But to estimate the force on the speedbrake at landing, you can use the dynamic pressure at landing speed x speedbrake area x cos 60 deg x Cd

		dynamic pressure q ~ 125 lb/sq ft (from q/M^2 = 1480 lb/sq ft)
		area ~ 4 sq ft x 4
		cos 60 deg = .866
		Cd ~ .7

		total drag force ~ 1212 lb

		That is the total force (parallel to the fuselage centerline) on all four panels at landing speed. It is much less than 3 tons.
		*/
		// ~1.48645m^2 area

		//double force = dynamicPressure_LBFT2 * 16.0 * cos(60) * 0.7;

		if (airbrakeActuator.m_current > 0)
		{
			double CDAirbrake = airbrakeActuator.m_current * 0.7;
			airbrakeDrag = -(CDAirbrake * cos(F16::degtorad));

			//double pressureAreaFT2 = airbrakeArea_FT2 * dynamicPressure_LBFT2;
			//double airbrake_DEG = (airbrakeActuator.m_current * 60); // <- PCT to DEG
			//airbrakeDrag = -(0.7 * cos(airbrake_DEG));
		}
		else
		{
			airbrakeDrag = 0;
		}

	}

	void updateFrame(double frametime) {}
};

#endif // ifndef _F16FCSAIRBRAKECONTROLLER_H_
