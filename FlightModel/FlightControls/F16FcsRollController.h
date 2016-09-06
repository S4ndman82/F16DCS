#ifndef _F16FCSROLLCONTROLLER_H_
#define _F16FCSROLLCONTROLLER_H_

#include <cmath>

#include "../UtilityFunctions.h"

//#include "../include/general_filter.h"
#include "DummyFilter.h"

#include "F16FcsCommon.h"


class F16FcsRollController
{
protected:
	F16BodyState *bodyState;
	F16FlightSurface *flightSurface;

	DummyFilter	latStickForceFilter;
	DummyFilter	rollCommandFilter;
	DummyFilter	rollActuatorDynamicsFilter;
	DummyFilter	rollRateFilter1;
	DummyFilter	rollRateFilter2;

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

	double getPressureGain(double dynamicPressure_NM2) const
	{
		double pressureGain = 0.0;
		if (dynamicPressure_NM2 < 19153.0)
		{
			pressureGain = 0.2;
		}
		else if ((dynamicPressure_NM2 >= 19153.0) && (dynamicPressure_NM2 <= 23941.0))
		{
			pressureGain = -0.00002089 * dynamicPressure_NM2 + 0.6;
		}
		else
		{
			pressureGain = 0.1;
		}
		return pressureGain;
	}

	// Controller for roll
	double fcs_roll_controller(double latStickInput, double longStickForce, double trimRoll, double dynPressure_LBFT2, double dt)
	{
		const double roll_rate = bodyState->getRollRateDegs();
		double ay = bodyState->getAccYPerG();


		double latStickForceCmd = latStickInput * 75.0;
		double latStickForce = latStickForceFilter.Filter(dt, latStickForceCmd);

		double latStickForceBiased = latStickForce - (ay * 8.9);  // CJS: remove side acceleration bias?

		double rollFeelGain = getRollFeelGain(longStickForce); // <- bug? (should be lateral?)
		double rollRateCommand = getRollRateCommand(latStickForceBiased * rollFeelGain);

		double rollRateCommandFilterd = rollCommandFilter.Filter(dt, rollRateCommand);

		double rollRateFiltered1 = rollRateFilter1.Filter(dt, roll_rate);
		double rollRateFiltered2 = (rollRateFilter2.Filter(dt, rollRateFiltered1));

		double rollRateCommandCombined = rollRateFiltered2 - rollRateCommandFilterd - trimRoll;

		double dynamicPressure_NM2 = dynPressure_LBFT2 * 47.880258889;

		double pressureGain = getPressureGain(dynamicPressure_NM2);

		double rollCommandGained = limit(rollRateCommandCombined * pressureGain, -21.5, 21.5);

		// Mechanical servo dynamics
		double rollActuatorCommand = rollActuatorDynamicsFilter.Filter(dt, rollCommandGained);
		return rollActuatorCommand;
	}

public:
	F16FcsRollController(F16BodyState *bs, F16FlightSurface *fs) :
		bodyState(bs),
		flightSurface(fs),
		latStickForceFilter(),
		rollCommandFilter(),
		rollActuatorDynamicsFilter(),
		rollRateFilter1(),
		rollRateFilter2()
	{}
	~F16FcsRollController() {}

	bool initialize(double dt)
	{
		double numerators[2] = { 0.0, 60.0 };
		double denominators[2] = { 1.0, 60.0 };
		latStickForceFilter.InitFilter(numerators, denominators, 1);

		double numerators1[2] = { 0.0, 10.0 };
		double denominators1[2] = { 1.0, 10.0 };
		rollCommandFilter.InitFilter(numerators1, denominators1, 1);

		double numerators2[3] = { 0.0, 0.0, pow(52.0, 2.0) };
		double denomiantors2[3] = { 1.0, 2.0*0.7*52.0, pow(52.0, 2.0) };
		rollActuatorDynamicsFilter.InitFilter(numerators2, denomiantors2, 2);

		double numerators3[2] = { 0.0, 50.0 };
		double denominators3[2] = { 1.0, 50.0 };
		rollRateFilter1.InitFilter(numerators3, denominators3, 1);

		double numerators4[3] = { 4.0, 64.0, 6400.0 };
		double denomiantors4[3] = { 1.0, 80.0, 6400.0 };
		rollRateFilter2.InitFilter(numerators4, denomiantors4, 2);
		return true;
	}
	void reset(double dt)
	{
		latStickForceFilter.ResetFilter(dt);
		rollCommandFilter.ResetFilter(dt);
		rollActuatorDynamicsFilter.ResetFilter(dt);
		rollRateFilter1.ResetFilter(dt);
		rollRateFilter2.ResetFilter(dt);
	}

	void updateFrame(double frametime) {}
};

#endif // ifndef _F16FCSROLLCONTROLLER_H_
