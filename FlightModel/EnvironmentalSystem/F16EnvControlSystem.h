#ifndef _F16ENVCONTROLSYSTEM_H_
#define _F16ENVCONTROLSYSTEM_H_

#include "F16Constants.h"

#include "Atmosphere/F16Atmosphere.h"			//Atmosphere model functions

#include "EnvironmentalSystem/F16AirConditioning.h"
#include "EnvironmentalSystem/F16OxygenSystem.h"


// ENVIRONMENTAL CONTROL SYSTEM (ECS)
// cockpit pressure, temperature, sealing, defogging, G-suit pressure, fuel tank pressure, equipment cooling


// use high/low bleed air pressure from engine

// TODO: G-suit pressure..

class F16EnvControlSystem
{
protected:
	double lowpressure;
	double highpressure;

	// TODO: cockpit pressure in pascals over external (get update from oxygen system also)
	double cockpitPressure;

public:
	F16AirConditioning AirCond;
	F16OxygenSystem Oxy;

	F16Atmosphere *pAtmos;

public:
	F16EnvControlSystem(F16Atmosphere *atmos)
		: lowpressure(0)
		, highpressure(0)
		, cockpitPressure(0)
		, AirCond()
		, Oxy()
		, pAtmos(atmos)
	{}
	~F16EnvControlSystem() {}

	void setOxygenSystem(float value)
	{
		// value currently ignored, just use as event trigger
		Oxy.setDiluterNormal();
	}

	// TODO:
	// get cockpit pressure in pascals over external (get update from oxygen system also)
	// -> set to ambient pressure when canopy gone or failure in ECS
	double getCockpitPressure() const
	{
		return cockpitPressure;
	}

	void updateFrame(const double frameTime)
	{
		// logic of using high/low pressure of bleed air?
			
		Oxy.updateFrame(pAtmos->ambientPressure, pAtmos->altitude, frameTime);

		// just use oxygen system pressure directly?
		cockpitPressure = Oxy.getPressure();

	}

	// there's "high" and "low" pressure from the engine
	void setLowPressureBleedAir(const double value)
	{
		lowpressure = value;
	}
	void setHighPressureBleedAir(const double value)
	{
		highpressure = value;
	}
};

#endif // ifndef 
