#ifndef _F16OXYGENSYSTEM_H_
#define _F16OXYGENSYSTEM_H_

#include "ED_FM_Utility.h"		// Provided utility functions that were in the initial EFM example
#include "F16Constants.h"		// Common constants used throughout this DLL

// amount of bleed air from engine (oxygen generator)
// oxygen tanks
// valve open/not
// pressurization

// 5-liter liquid oxygen -> diluter (0..100 percent O2)

class F16OxygenSystem
{
protected:
	const double tank_volume; // whole volume of tank
	double tank_usage; // amount used
	double diluter_setting; // normal/100% ?

	double pressureGen; // pressure provided (pascals), used over ambient pressure

	// sensors? (for cockpit?)
	// valves?

public:
	F16OxygenSystem() 
		: tank_volume(5.0)
		, tank_usage(5.0)
		, diluter_setting(0) // <- off
		, pressureGen(0)
	{}
	~F16OxygenSystem() {}

	void setDiluterNormal()
	{
		diluter_setting = 100;
	}

	double getPressure() const
	{
		return pressureGen;
	}

	// very rough value for cockpit pressure over ambient pressure
	void updateFrame(const double ambientPressure, const double altitude, const double frameTime)
	{
		// very rough usage
		double use = frameTime * diluter_setting;
		if (tank_usage > use)
		{
			tank_usage -= use;
		}
		//tanks += o2_gen;

		// pressure provided depends on diluter setting
		// and ambient pressure ?
		pressureGen = 101325; // standard sea-level atmosphere
		pressureGen -= ambientPressure; // reduce ambient pressure (result pressure over ambient)
	}
};

#endif // ifndef _F16OXYGENSYSTEM_H_
