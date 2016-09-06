#ifndef _F16FCSYAWCONTROLLER_H_
#define _F16FCSYAWCONTROLLER_H_

#include <cmath>

#include "../UtilityFunctions.h"


class F16FcsYawController
{
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

public:
	F16FcsYawController() {}
	~F16FcsYawController() {}

	void updateFrame(double frametime) {}
};

#endif // ifndef _F16FCSYAWCONTROLLER_H_
