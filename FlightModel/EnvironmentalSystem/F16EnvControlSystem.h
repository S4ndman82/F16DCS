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
	public:
		F16AirConditioning AirCond;
		F16OxygenSystem Oxy;

	public:
		F16EnvControlSystem()
			: AirCond()
			, Oxy()
		{}
		~F16EnvControlSystem() {}

		void updateFrame(const double frameTime)
		{
		}
	};
}

#endif // ifndef 
