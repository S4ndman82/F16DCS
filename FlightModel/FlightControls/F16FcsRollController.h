#ifndef _F16FCSROLLCONTROLLER_H_
#define _F16FCSROLLCONTROLLER_H_

#include <cmath>

#include "../UtilityFunctions.h"


class F16FcsRollController
{
public:
	double getRollFeelGain(const double longStickForce) const
	{
		double longStickForceGained = longStickForce * 0.0667;
		double rollFeelGain = 0.0;
		if (abs(longStickForce) > 25.0)
		{
			rollFeelGain = 0.7;
		}
		else if (longStickForce >= 0.0)
		{
			rollFeelGain = -0.012 * longStickForceGained + 1.0;
		}
		else if (longStickForce < 0.0)
		{
			rollFeelGain = 0.012 * longStickForceGained + 1.0;
		}
		return rollFeelGain;
	}

	double getRollRateCommand(const double latStickForceFinal) const
	{
		double rollRateCommand = 0.0;
		if (abs(latStickForceFinal) < 3.0)
		{
			rollRateCommand = 0.0;
		}
		else if ((latStickForceFinal >= 3.0) && (latStickForceFinal <= 25.0))
		{
			rollRateCommand = 0.9091 * latStickForceFinal - 2.7273;
		}
		else if ((latStickForceFinal > 25.0) && (latStickForceFinal <= 46.0))
		{
			rollRateCommand = 2.8571 * latStickForceFinal - 51.429;
		}
		else if ((latStickForceFinal > 46.0))
		{
			rollRateCommand = 7.5862 * latStickForceFinal - 268.97;
		}
		else if ((latStickForceFinal <= -3.0) && (latStickForceFinal >= -25.0))
		{
			rollRateCommand = 0.9091 * latStickForceFinal + 2.7273;
		}
		else if ((latStickForceFinal < -25.0) && (latStickForceFinal >= -46.0))
		{
			rollRateCommand = 2.8571 * latStickForceFinal + 51.429;
		}
		else if ((latStickForceFinal < -46.0))
		{
			rollRateCommand = 7.5862 * latStickForceFinal + 268.97;
		}
		return rollRateCommand;
	}

public:
	F16FcsRollController() {}
	~F16FcsRollController() {}

	void updateFrame(double frametime) {}
};

#endif // ifndef _F16FCSROLLCONTROLLER_H_
