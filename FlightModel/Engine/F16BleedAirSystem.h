#ifndef _F16BLEEDAIRSYSTEM_H_
#define _F16BLEEDAIRSYSTEM_H_

#include "../stdafx.h"

/*
 bleed air from jet engine can be used for various thing (anti-ice, oxygen generator..),
 handle that system here

 -> air conditioning (cabin pressure, cockpit temperature)

 F100-PW-200
 -> to AB fuel pump (driven by this)
 (low) -> fan duct (avoid compressor stall)

 (high) -> EPU
 (high/low) -> ECS

 F100-PW-220
 -> to AB fuel pump (driven by this)
 (low) -> fan duct (avoid compressor stall)
 -> anti-icing

 -> CENC

 (high) -> EPU
 (high/low) -> ECS

*/
namespace F16
{
	class F16BleedAirSystem
	{
	protected:
		// 
		//double currentEngineRpm;

		double lowpressure;
		double highpressure;

		// TODO:
		//F16FuelPump *pABFuelPump;
		//F16EnvControlSystem *pEnvSystem;
		//F16EPU *pEpu;

	public:
		F16BleedAirSystem() 
			: lowpressure(0)
			, highpressure(0)
		{}
		~F16BleedAirSystem() {}

		// update with engine/APU rpm/torque
		// and power consumption
		void updateFrame(const double frameTime)
		{
		}

		/*
		// we might need to calculate air pressure directly in engine code
		// due to differences in engines?
		void setEngineRpm(const double value)
		{
		}
		*/

		// actually, there's "high" and "low" pressure from the engine
		// -> set them from engine
		void setLowPressureBleedAir(const double value)
		{
			// just a function of engine RPM?
			lowpressure = value;
		}
		void setHighPressureBleedAir(const double value)
		{
			// just a function of engine RPM?
			highpressure = value;
		}

	};
}

#endif // ifndef _F16BLEEDAIRSYSTEM_H_

