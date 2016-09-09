#ifndef _F16FCSYAWCONTROLLER_H_
#define _F16FCSYAWCONTROLLER_H_

#include <cmath>

#include "../UtilityFunctions.h"

//#include "../include/general_filter.h"
#include "DummyFilter.h"

#include "F16FcsCommon.h"


class F16FcsYawController
{
protected:
	F16BodyState *bodyState;
	F16FlightSurface *flightSurface;

	Limiter<double>		rudderLimiter;

	DummyFilter	rudderCommandFilter;
	DummyFilter	yawRateWashout;
	DummyFilter	yawRateFilter;
	DummyFilter	yawServoFilter;

public:
	double getRudderCommand(const double pedInput) const
	{
		double rudderForceCommand = pedInput * 450.0;

		double rudderCommand = 0.0;
		if (abs(rudderForceCommand) < 44.0)
		{
			rudderCommand = 0.0;
		}
		else if (rudderForceCommand >= 44.0)
		{
			rudderCommand = -0.0739 * rudderForceCommand + 3.2512;
		}
		else if (rudderForceCommand <= -44.0)
		{
			rudderCommand = -0.0739 * rudderForceCommand - 3.2512;
		}

		return limit(rudderCommand, -30.0, 30.0);
	}

	// Controller for yaw
	double fcs_yaw_controller(double pedInput, double trimYaw, double alphaFiltered, double aileron_commanded, double dt)
	{
		const double roll_rate = bodyState->getRollRateDegs();
		const double yaw_rate = bodyState->getYawRateDegs();
		double ay = bodyState->getAccYPerG();

		double rudderCommand = getRudderCommand(pedInput);
		double rudderCommandFiltered = rudderCommandFilter.Filter(dt, rudderCommand);
		double rudderCommandFilteredWTrim = trimYaw - rudderCommandFiltered;

		double alphaGained = alphaFiltered * (1.0 / 57.3);
		double rollRateWithAlpha = roll_rate * alphaGained;
		double yawRateWithRoll = yaw_rate - rollRateWithAlpha;

		double yawRateWithRollWashedOut = yawRateWashout.Filter(dt, yawRateWithRoll);
		double yawRateWithRollFiltered = yawRateFilter.Filter(dt, yawRateWithRollWashedOut);

		double yawRateFilteredWithSideAccel = yawRateWithRollFiltered;// + (ay * 19.3);

		double aileronGained = limit(0.05 * alphaFiltered, 0.0, 1.5) * aileron_commanded;

		double finalRudderCommand = aileronGained + yawRateFilteredWithSideAccel + rudderCommandFilteredWTrim;

		flightSurface->rudder_DEG = limit(finalRudderCommand, -30.0, 30.0);
		flightSurface->rudder_PCT = flightSurface->rudder_DEG / 30.0;

		return finalRudderCommand;

		//TODO: Figure out why there is a ton of flutter at high speed due to these servo dynamics
		//double yawServoCommand = yawServoFilter.Filter(!(simInitialized),dt,finalRudderCommand);
		//return yawServoCommand;
	}

public:
	F16FcsYawController(F16BodyState *bs, F16FlightSurface *fs) :
		bodyState(bs),
		flightSurface(fs),
		rudderLimiter(-30, 30), // deflection limit
		rudderCommandFilter(),
		yawRateWashout(),
		yawRateFilter(),
		yawServoFilter()
	{
		// just do this once when constructing
		initialize(0);
		reset(0);
	}
	~F16FcsYawController() {}

	bool initialize(double dt)
	{
		double numerators[2] = { 0.0, 4.0 };
		double denominators[2] = { 1.0, 4.0 };
		rudderCommandFilter.InitFilter(numerators, denominators, 1);

		double numerators1[2] = { 1.0, 0.0 };
		double denominators1[2] = { 1.0, 1.0 };
		yawRateWashout.InitFilter(numerators1, denominators1, 1);

		double numerators2[2] = { 3.0, 15.0 };
		double denominators2[2] = { 1.0, 15.0 };
		yawRateFilter.InitFilter(numerators2, denominators2, 1);

		double numerators3[3] = { 0.0, 0.0, pow(52.0, 2.0) };
		double denomiantors3[3] = { 1.0, 2.0*0.7*52.0, pow(52.0, 2.0) };
		yawServoFilter.InitFilter(numerators3, denomiantors3, 2);
		return true;
	}
	void reset(double dt)
	{
		rudderCommandFilter.ResetFilter(dt);
		yawRateWashout.ResetFilter(dt);
		yawRateFilter.ResetFilter(dt);
		yawServoFilter.ResetFilter(dt);
	}

	void updateFrame(double frametime) {}
};

#endif // ifndef _F16FCSYAWCONTROLLER_H_
