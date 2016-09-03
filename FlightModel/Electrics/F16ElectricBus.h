#ifndef _F16ELECTRICBUS_H_
#define _F16ELECTRICBUS_H_

#include "include/ED_FM_Utility.h"		// Provided utility functions that were in the initial EFM example
#include "include/F16Constants.h"		// Common constants used throughout this DLL

//#include "Electrics/AbstractElectricDevice.h"

/*

sources:
- NASA TP 2857

*/

namespace F16
{
	class F16ElectricBus
	{
	protected:
		// type: AC no 1, AC no 2, DC "battery" bus
		// voltage: 28V DC?
		// on/off
		// battery/generator status?

		//AbstractElectricDevice devices[];

	public:
		F16ElectricBus() {}
		~F16ElectricBus() {}

	
		void updateFrame(const double frameTime)
		{
		}

	};
}

#endif // ifndef _F16ELECTRICBUS_H_

