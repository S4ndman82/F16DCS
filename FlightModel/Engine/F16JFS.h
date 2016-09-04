#ifndef _F16JFS_H_
#define _F16JFS_H_

#include "include/ED_FM_Utility.h"		// Provided utility functions that were in the initial EFM example
#include "include/F16Constants.h"		// Common constants used throughout this DLL


/*
sources:
- https://www.coursehero.com/flashcards/669965/F-16-PW-220-Engine/

JFS = Jet Fuel Starter,
gas turbine -> auxiliary drive gearbox -> clutch -> engine starter and maintains rpm
*/

// two batteries "accumulators",
// charged by hydraulic system
//class F16JFSAccu

// torque to main engine via ADG+clutch,
// if engine "seized" clutch slippage
// ..
class F16JFS
{
protected:

	// keeps running until switched off,
	// may provide additional power
	bool m_switchOn;

	// see conditions for light illumination
	bool m_runLight;

	// while spooldown in progress, cannot restart JFS
	bool m_spoolDown;
	bool m_spoolUp;

	double m_currentRpm;
	double m_currentTemperature;

public:
	F16JFS() 
		: m_switchOn(false)
		, m_runLight(false)
		, m_spoolDown(false)
		, m_spoolUp(false)
		, m_currentRpm(0)
	{}
	~F16JFS() {}

	double getRpm() const
	{
		return 0;
	}
	double getRelatedRpm() const
	{
		return 0;
	}

	double getTemperature() const
	{
		return 0;
	}
	double getOilPressure() const
	{
		return 0;
	}
	double getFuelFlow() const
	{
		return 0;
	}

	void start()
	{
		m_switchOn = true;
		if (m_spoolDown == false)
		{
			// -> starting
			m_spoolUp = true;
		}

	}
	void stop()
	{
		// start spooldown
		if (m_switchOn == true)
		{
			m_spoolDown = true;
		}
		m_switchOn = false;
	}

	void updateFrame(const double frameTime)
	{
		// when ground start
		// if (m_currentRpm >= 12%) -> charge accumulators
		// if (m_currentRpm >= 50%) -> light goes out (shutdown?)

		// when airstart
		// if (m_currentRpm >= 70%) -> charge accumulators
		// 3-4 sec -> run light on

		if (m_spoolDown == true && m_currentRpm == 0)
		{
			// just check if we have reached 0 rpm
			// -> can start again
			m_spoolDown = false;
		}
	}

};

#endif // ifndef _F16JFS_H_

