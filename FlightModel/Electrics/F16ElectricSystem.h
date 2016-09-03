#ifndef _F16ELECTRICSYSTEM_H_
#define _F16ELECTRICSYSTEM_H_

#include "Electrics/F16AcDcConverter.h"
#include "Electrics/F16Battery.h"
#include "Electrics/F16ElectricBus.h"
#include "Electrics/F16MainGenerator.h"

/*
sources:
- NASA TP 2857

*/

namespace F16
{
	class F16ElectricSystem
	{
	protected:
		F16Battery battery;

		// bit too generic: FC3 style use only
		bool electricsOnOff;

		// type: AC no 1, AC no 2, DC "battery" bus
		F16ElectricBus AcNo1;
		F16ElectricBus AcNo2;
		F16ElectricBus DcBat;

	public:
		F16ElectricSystem() 
			: battery()
			, electricsOnOff(false)
		{}
		~F16ElectricSystem() {}

		// FC3 style electric power on/off
		void toggleElectrics()
		{
			electricsOnOff = !electricsOnOff;
		}
		void setElectricsOn()
		{
			electricsOnOff = true;
		}
		void setElectricsOff()
		{
			electricsOnOff = false;
		}
		
		// update with engine/APU rpm/torque
		// and power consumption
		void updateFrame(const double frameTime)
		{
		}

	};
}

#endif // ifndef _F16ELECTRICSYSTEM_H_

