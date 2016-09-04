#ifndef _F16BATTERY_H_
#define _F16BATTERY_H_

#include "include/ED_FM_Utility.h"		// Provided utility functions that were in the initial EFM example
#include "include/F16Constants.h"		// Common constants used throughout this DLL

#include "Electrics/AbstractElectricDevice.h"

/*

sources:
- NASA TP 2857

*/

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

#endif // ifndef _F16BATTERY_H_

