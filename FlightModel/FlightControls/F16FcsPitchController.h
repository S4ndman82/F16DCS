#ifndef _F16FCSPITCHCONTROLLER_H_
#define _F16FCSPITCHCONTROLLER_H_

#include <cmath>

#include "../UtilityFunctions.h"

//#include "../include/general_filter.h"
#include "DummyFilter.h"

#include "F16FcsCommon.h"
#include "F16Actuator.h"

class F16FcsPitchController
{
protected:
	double		m_stickCommandPosFiltered;
	double		m_alphaFiltered;
	double		m_longStickForce;
	double		m_latStickForce;

	F16BodyState *bodyState;
	F16FlightSurface *flightSurface;

	Limiter<double>		htailLimiter;

	DummyFilter	pitchRateWashout;
	DummyFilter	pitchIntegrator;
	DummyFilter	pitchPreActuatorFilter;
	DummyFilter	pitchActuatorDynamicsFilter;
	DummyFilter	accelFilter;

	/*
	F16Actuator		elevatorActuatorLeft;
	F16Actuator		elevatorActuatorRight;
	*/

public:
	// Schedule gain component due to dynamic pressure
	double dynamic_pressure_schedule(const double dynamicPressure_kNM2) const
	{
		double scheduleOutput = 0.0;
		if (dynamicPressure_kNM2 < 9.576)
		{
			scheduleOutput = 1.0;
		}
		else if ((dynamicPressure_kNM2 >= 9.576) && (dynamicPressure_kNM2 <= 43.0))
		{
			scheduleOutput = (-0.018 * dynamicPressure_kNM2) + 1.1719;
			//scheduleOutput =  (-0.0239 * dynamicPressure_kNM2) + 1.2292;
		}
		else if (dynamicPressure_kNM2 > 43.0)
		{
			scheduleOutput = -0.003 * dynamicPressure_kNM2 + 0.5277;
			//scheduleOutput = -0.001 * dynamicPressure_kNM2 + 0.2422;
		}
		return limit(scheduleOutput, 0.05, 1.0);
	}


	double getPitchRateCommand(const double longStickInputForce) const
	{
		double longStickCommand_G = 0.0;
		if (abs(longStickInputForce) <= 8.0)
		{
			longStickCommand_G = 0.0;
		}
		else if ((longStickInputForce < -8) && (longStickInputForce > -33.0))
		{
			longStickCommand_G = (0.016 * longStickInputForce) + 0.128;
		}
		else if (longStickInputForce <= -33.0)
		{
			longStickCommand_G = (0.067 * longStickInputForce) + 1.8112;
		}
		else if ((longStickInputForce > 8.0) && (longStickInputForce < 33.0))
		{
			longStickCommand_G = (0.032 * longStickInputForce) - 0.256;
		}
		else if (longStickInputForce >= 33.0)
		{
			longStickCommand_G = 0.0681*longStickInputForce - 1.4468;
		}
		return longStickCommand_G;
	}

	// Stick force schedule for pitch control
	double fcs_pitch_controller_force_command(double longStickInput, double trimPitch, double frameTime)
	{
		double longStickInputForce = 0.0;
		if (longStickInput > 0.0)
		{
			longStickInputForce = longStickInput * 80.0;
		}
		else
		{
			longStickInputForce = longStickInput * 180.0;
		}
		longStickInputForce = limit(longStickInputForce, -180.0, 80.0);
		m_longStickForce = longStickInputForce;

		// TODO: if pitch override command is in effect -> override G-limit
		//if (manualPitchOverride == true)

		double longStickCommand_G = getPitchRateCommand(longStickInputForce);

		double longStickCommandWithTrim_G = trimPitch - longStickCommand_G;

		double longStickCommandWithTrimLimited_G = limit(longStickCommandWithTrim_G, -4.0, 8.0);

		double longStickCommandWithTrimLimited_G_Rate = 4.0 * (longStickCommandWithTrimLimited_G - m_stickCommandPosFiltered);
		m_stickCommandPosFiltered += (longStickCommandWithTrimLimited_G_Rate * frameTime);

		return m_stickCommandPosFiltered;
	}

	// Angle of attack limiter logic
	double angle_of_attack_limiter(const double alphaFiltered, const double pitchRateCommand) const
	{
		// TODO: 
		//if (manualPitchOverride == true)

		double topLimit = limit((alphaFiltered - 22.5) * 0.69, 0.0, 99999.0);
		double bottomLimit = limit((alphaFiltered - 15.0 + pitchRateCommand) * 0.322, 0.0, 99999.0);

		return (topLimit + bottomLimit);
	}


	// Controller for pitch
	// (differentialCommand is hard-coded to 0 in caller)
	double fcs_pitch_controller(double longStickInput, double trimPitch, double differentialCommand, double dynamicPressure_kNM2, double dt)
	{
		const double pitch_rate = bodyState->getPitchRateDegs();
		const double az = bodyState->getAccZPerG();

		double stickCommandPos = fcs_pitch_controller_force_command(longStickInput, trimPitch, dt);
		double dynamicPressureScheduled = dynamic_pressure_schedule(dynamicPressure_kNM2);

		double azFiltered = accelFilter.Filter(dt, az - 1.0);

		double alphaLimited = limit(bodyState->alpha_DEG, -5.0, 30.0);
		double alphaLimitedRate = 10.0 * (alphaLimited - m_alphaFiltered);
		m_alphaFiltered += (alphaLimitedRate * dt);

		double pitchRateWashedOut = pitchRateWashout.Filter(dt, pitch_rate);
		double pitchRateCommand = pitchRateWashedOut * 0.7 * dynamicPressureScheduled;

		double limiterCommand = angle_of_attack_limiter(-m_alphaFiltered, pitchRateCommand);

		double gLimiterCommand = -(azFiltered + (pitchRateWashedOut * 0.2));

		double finalCombinedCommand = dynamicPressureScheduled * (2.5 * (stickCommandPos + limiterCommand + gLimiterCommand));

		double finalCombinedCommandFilteredLimited = limit(pitchIntegrator.Filter(dt, finalCombinedCommand), -25.0, 25.0);
		finalCombinedCommandFilteredLimited = finalCombinedCommandFilteredLimited + finalCombinedCommand;

		double finalPitchCommandTotal = pitchPreActuatorFilter.Filter(dt, finalCombinedCommandFilteredLimited);
		finalPitchCommandTotal += (0.5 * m_alphaFiltered);

		// TODO: separate movements on opposing side (with roll authority)
		flightSurface->elevator_DEG = limit(-finalPitchCommandTotal, -25.0, 25.0);
		flightSurface->elevator_Right_PCT = flightSurface->elevator_DEG / 25.0;
		flightSurface->elevator_Left_PCT = flightSurface->elevator_DEG / 25.0;

		return finalPitchCommandTotal;

		// TODO: There are problems with flutter with the servo dynamics...needs to be nailed down!
		//double actuatorDynamicsResult = pitchActuatorDynamicsFilter.Filter(!(simInitialized),dt,finalPitchCommandTotal);
		//return actuatorDynamicsResult;	
	}

public:
	F16FcsPitchController(F16BodyState *bs, F16FlightSurface *fs) :
		m_stickCommandPosFiltered(0),
		m_alphaFiltered(0),
		m_longStickForce(0),
		m_latStickForce(0),
		bodyState(bs),
		flightSurface(fs),
		htailLimiter(-25, 25), // stab. deflection limits
		pitchRateWashout(),
		pitchIntegrator(),
		pitchPreActuatorFilter(),
		pitchActuatorDynamicsFilter(),
		accelFilter()
	{
		// just do this once when constructing
		initialize(0);
		reset(0);
	}
	~F16FcsPitchController() {}

	bool initialize(double dt)
	{
		double numerators[2] = { 1.0, 0.0 };
		double denominators[2] = { 1.0, 1.0 };
		pitchRateWashout.InitFilter(numerators, denominators, 1);

		numerators[0] = 0.0; numerators[1] = 2.5;
		denominators[0] = 1.0; denominators[1] = 0.0;
		pitchIntegrator.InitFilter(numerators, denominators, 1);

		numerators[0] = 3.0; numerators[1] = 15;
		denominators[0] = 1.0; denominators[1] = 15.0;
		pitchPreActuatorFilter.InitFilter(numerators, denominators, 1);

		double numerators2[3] = { 0.0, 0.0, pow(52.0, 2.0) };
		double denomiantors2[3] = { 1.0, 2.0*0.7*52.0, pow(52.0, 2.0) };
		pitchActuatorDynamicsFilter.InitFilter(numerators2, denomiantors2, 2);

		numerators[0] = 0.0; numerators[1] = 15.0;
		denominators[0] = 1.0; denominators[1] = 15.0;
		accelFilter.InitFilter(numerators, denominators, 1);
		return true;
	}
	void reset(double dt)
	{
		pitchRateWashout.ResetFilter(dt);
		pitchIntegrator.ResetFilter(dt);
		pitchPreActuatorFilter.ResetFilter(dt);
		pitchActuatorDynamicsFilter.ResetFilter(dt);
		accelFilter.ResetFilter(dt);
	}

	double getAlphaFiltered() const { return m_alphaFiltered; }
	double getLongStickForce() const { return m_longStickForce; }
	double getLatStickForce() const { return m_latStickForce; }

	void updateFrame(double frametime) {}
};

#endif // ifndef _F16FCSPITCHCONTROLLER_H_
