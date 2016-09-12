#ifndef _F16FCSYAWCONTROLLER_H_
#define _F16FCSYAWCONTROLLER_H_

#include <cmath>

#include "../UtilityFunctions.h"

#include "F16FcsCommon.h"
#include "F16Actuator.h"


class F16FcsYawController
{
protected:
	F16BodyState *bodyState;
	F16FlightSurface *flightSurface;
	F16TrimState *trimState;

	F16Actuator		rudderActuator;
	Limiter<double>		rudderLimiter;

protected:
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

		return rudderLimiter.limit(rudderCommand);
	}

public:
	F16FcsYawController(F16BodyState *bs, F16FlightSurface *fs, F16TrimState *ts) :
		bodyState(bs),
		flightSurface(fs),
		trimState(ts),
		rudderActuator(60.0, -30.0, 30.0), // <- check rate
		rudderLimiter(-30, 30) // deflection limit
	{
	}
	~F16FcsYawController() {}

	bool initialize(double dt)
	{
		return true;
	}
	void reset(double dt)
	{
	}

	// Controller for yaw
	void fcs_yaw_controller(double pedInput, double alphaFiltered)
	{
		const double roll_rate = bodyState->getRollRateDegs();
		const double yaw_rate = bodyState->getYawRateDegs();

		double rudderCommand = getRudderCommand(pedInput);
		double rudderCommandFilteredWTrim = trimState->trimYaw - rudderCommand;

		double alphaGained = alphaFiltered * (1.0 / 57.3);
		double rollRateWithAlpha = roll_rate * alphaGained;
		double yawRateWithRoll = yaw_rate - rollRateWithAlpha;

		// TODO: use flightSurface->roll_Command instead?
		double aileronGained = limit(0.05 * alphaFiltered, 0.0, 1.5) * flightSurface->aileron_DEG;

		//double ay = bodyState->getAccYPerG();
		// TODO: side acceleration (+ (ay * 19.3)) ?
		flightSurface->yaw_Command = aileronGained + yawRateWithRoll + rudderCommandFilteredWTrim;

		// without blending, rudder command == yaw command
		// -> change in mixer (after this call) if/when necessary
		flightSurface->rudder_Command = flightSurface->yaw_Command;
	}

	void updateFrame(double frametime) 
	{
		rudderActuator.commandMove(flightSurface->rudder_Command);
		rudderActuator.updateFrame(frametime);

		flightSurface->rudder_DEG = rudderActuator.m_current;
		flightSurface->rudder_PCT = flightSurface->rudder_DEG / 30.0;
	}
};

#endif // ifndef _F16FCSYAWCONTROLLER_H_
