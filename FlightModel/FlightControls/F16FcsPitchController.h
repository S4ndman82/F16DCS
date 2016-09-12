#ifndef _F16FCSPITCHCONTROLLER_H_
#define _F16FCSPITCHCONTROLLER_H_

#include <cmath>

#include "../UtilityFunctions.h"

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
	F16TrimState *trimState;

	/*
	F16Actuator		elevatorActuatorLeft;
	F16Actuator		elevatorActuatorRight;
	*/

protected:
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
		double topLimit = limit((alphaFiltered - 22.5) * 0.69, 0.0, 99999.0);
		double bottomLimit = limit((alphaFiltered - 15.0 + pitchRateCommand) * 0.322, 0.0, 99999.0);

		return (topLimit + bottomLimit);
	}

public:
	F16FcsPitchController(F16BodyState *bs, F16FlightSurface *fs, F16TrimState *ts) :
		m_stickCommandPosFiltered(0),
		m_alphaFiltered(0),
		m_longStickForce(0),
		m_latStickForce(0),
		bodyState(bs),
		flightSurface(fs),
		trimState(ts)
	{
	}
	~F16FcsPitchController() {}

	bool initialize(double dt)
	{
		return true;
	}
	void reset(double dt)
	{
	}

	double getAlphaFiltered() const { return m_alphaFiltered; }
	double getLongStickForce() const { return m_longStickForce; }
	double getLatStickForce() const { return m_latStickForce; }

	// Controller for pitch
	// TODO: implement differential actuator handling to mixer and actuator stages
	void fcs_pitch_controller(double longStickInput, double dynamicPressure_kNM2, bool manualPitchOverride, double dt)
	{
		double stickCommandPos = fcs_pitch_controller_force_command(longStickInput, trimState->trimPitch, dt);
		double dynamicPressureScheduled = dynamic_pressure_schedule(dynamicPressure_kNM2);


		double alphaLimited = limit(bodyState->alpha_DEG, -5.0, 30.0);
		double alphaLimitedRate = 10.0 * (alphaLimited - m_alphaFiltered);
		m_alphaFiltered += (alphaLimitedRate * dt);

		double pitchRateWashedOut = bodyState->getPitchRateDegs();
		double pitchRateCommand = pitchRateWashedOut * 0.7 * dynamicPressureScheduled;

		// TODO: 
		if (manualPitchOverride == true)
		{
		}

		double azFiltered = bodyState->getAccZPerG() - 1.0;
		double limiterCommand = angle_of_attack_limiter(-m_alphaFiltered, pitchRateCommand);
		double gLimiterCommand = -(azFiltered + (pitchRateWashedOut * 0.2));
		double finalCombinedCommand = dynamicPressureScheduled * (2.5 * (stickCommandPos + limiterCommand + gLimiterCommand));

		double finalCombinedCommandFilteredLimited = limit(finalCombinedCommand, -25.0, 25.0);
		finalCombinedCommandFilteredLimited = finalCombinedCommandFilteredLimited + finalCombinedCommand;
		double finalPitchCommandTotal = finalCombinedCommandFilteredLimited;
		finalPitchCommandTotal += (0.5 * m_alphaFiltered);

		flightSurface->pitch_Command = finalPitchCommandTotal;
	}

	void updateFrame(double frametime) 
	{
		// TODO: elevator result after mixing with roll controller

		// TODO: separate movements on opposing side (with roll authority)
		flightSurface->elevator_DEG = limit(-flightSurface->pitch_Command, -25.0, 25.0);
		flightSurface->elevator_Right_PCT = flightSurface->elevator_DEG / 25.0;
		flightSurface->elevator_Left_PCT = flightSurface->elevator_DEG / 25.0;
	}
};

#endif // ifndef _F16FCSPITCHCONTROLLER_H_
