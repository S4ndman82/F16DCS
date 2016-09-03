#ifndef _F16ELECTRICBUS_H_
#define _F16ELECTRICBUS_H_


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

