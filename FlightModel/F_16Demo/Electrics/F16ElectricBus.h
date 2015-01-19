#ifndef _F16ELECTRICBUS_H_
#define _F16ELECTRICBUS_H_

#include "../stdafx.h"

namespace F16
{
	class F16ElectricBus
	{
	protected:
		// type: AC, DC bus
		// voltage
		// on/off
		// battery/generator status?

	public:
		F16ElectricBus() {}
		~F16ElectricBus() {}
		
		void updateFrame(const double frameTime)
		{
		}

	};
}

#endif // ifndef _F16ELECTRICBUS_H_

