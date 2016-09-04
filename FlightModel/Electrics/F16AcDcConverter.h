#ifndef _F16ACDCCONVERTER_H_
#define _F16ACDCCONVERTER_H_

#include "ED_FM_Utility.h"		// Provided utility functions that were in the initial EFM example
#include "F16Constants.h"		// Common constants used throughout this DLL

#include "Electrics/AbstractElectricDevice.h"


/*
// also inverter/regulator?
// also transformer rectifier (TR) ? (500VA)

sources:
- NASA TP 2857

*/

class F16AcDcConverter : public AbstractElectricDevice
{
public:
	F16AcDcConverter(void *_parentSystem) 
		: AbstractElectricDevice(_parentSystem)
	{}
	~F16AcDcConverter() {}

	void updateFrame(const double frameTime)
	{
	}

};

#endif // ifndef _F16ACDCCONVERTER_H_

