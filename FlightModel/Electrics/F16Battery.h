#ifndef _F16BATTERY_H_
#define _F16BATTERY_H_

#include "../stdafx.h"

#include "Electrics/AbstractElectricDevice.h"

/*

sources:
- NASA TP 2857

*/

namespace F16
{
	class F16Battery //: public AbstractElectricDevice
	{
	public:
		// 28V DC?

		// battery "storage" values
		// discharge rate
		// charging rate

		// if battery power turned on
		bool m_isOn;

	public:
		F16Battery() 
			: m_isOn(false)
		{}
		~F16Battery() {}

		void updateFrame(const double frameTime)
		{
		}
	};
}

#endif // ifndef _F16BATTERY_H_

