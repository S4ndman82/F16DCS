#ifndef _F16FCSPITCHCONTROLLER_H_
#define _F16FCSPITCHCONTROLLER_H_

#include <cmath>

#include "../UtilityFunctions.h"


class F16FcsPitchController
{
protected:
	double		m_stickCommandPosFiltered;
	double		m_alphaFiltered;
	double		m_longStickForce;
	double		m_latStickForce;

public:
	// Schedule gain component due to dynamic pressure
	double dynamic_pressure_schedule(const double dynPressure_LBFT2) const
	{
		double dynamicPressure_kNM2 = dynPressure_LBFT2 * 1.4881639 / 1000.0; //for kN/m^2
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

public:
	F16FcsPitchController() :
		m_stickCommandPosFiltered(0),
		m_alphaFiltered(0),
		m_longStickForce(0),
		m_latStickForce(0)
	{}
	~F16FcsPitchController() {}

	double getAlphaFiltered() const { return m_alphaFiltered; }
	double getLongStickForce() const { return m_longStickForce; }
	double getLatStickForce() const { return m_latStickForce; }

	void updateFrame(double frametime) {}
};

#endif // ifndef _F16FCSPITCHCONTROLLER_H_
