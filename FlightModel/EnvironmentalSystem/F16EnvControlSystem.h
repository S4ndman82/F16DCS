#ifndef _F16ENVCONTROLSYSTEM_H_
#define _F16ENVCONTROLSYSTEM_H_

#include "../stdafx.h"

#include "F16AirConditioning.h"
#include "F16OxygenSystem.h"

// ENVIRONMENTAL CONTROL SYSTEM (ECS)
// cockpit pressure, temperature, sealing, defogging, G-suit pressure, fuel tank pressure, equipment cooling


// use high/low bleed air pressure from engine

namespace F16
{

	class F16EnvControlSystem
	{
	protected:
		double lowpressure;
		double highpressure;

	public:
		F16AirConditioning AirCond;
		F16OxygenSystem Oxy;

	public:
		F16EnvControlSystem()
			: lowpressure(0)
			, highpressure(0)
			, AirCond()
			, Oxy()
		{}
		~F16EnvControlSystem() {}

		void updateFrame(const double frameTime)
		{
			// logic of using high/low pressure of bleed air?

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
}

#endif // ifndef 
