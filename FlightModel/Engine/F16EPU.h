#ifndef _F16EPU_H_
#define _F16EPU_H_

#include "../stdafx.h"

/*
EPU = emergency power unit

-> get bleed air from engine and/or use hydrazine
-> emergency hydraulic pressure to system A
-> emergency electrical power
*/

namespace F16
{
	class F16EPU
	{
	protected:
		// if EPU/GEN test switch is used
		bool testSwitch;

		// high-pressure bleed air from engine
		double highBleedAir;

		// hydraulic pressure to/from ?
		double hydrPress;

		// AC/DC buses?

		// warning light status
		bool hydrazineLight; 

		//F16ElectricSystem *pElecSys;
		//F16HydraulicSystem *pHydSys;

	public:
		F16EPU() 
			: testSwitch(false)
			, highBleedAir(0)
			, hydrPress(0)
			, hydrazineLight(false)
		{}
		~F16EPU() {}

		void updateFrame(const double frameTime)
		{
			//if (testSwitch)

			//if (highBleedAir < limit)
			//-> augment with hydrazine
		}

		void setHighPressureBleedAir(const double value)
		{
			highBleedAir = value;
		}
	};
}

#endif // ifndef _F16EPU_H_

