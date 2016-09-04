#ifndef _ABSTRACTELECTRICDEVICE_H_
#define _ABSTRACTELECTRICDEVICE_H_

#include "ED_FM_Utility.h"		// Provided utility functions that were in the initial EFM example
#include "F16Constants.h"		// Common constants used throughout this DLL

// base class for electric device:
// each device (battery, generator, sensor etc.) inherits from this 
// to provide unified interface for the system
//
class AbstractElectricDevice
{
public:
	// TODO: replace with correct type
	void *parentSystem;

	AbstractElectricDevice(void *_parentSystem) 
		: parentSystem(_parentSystem)
	{}
	~AbstractElectricDevice() {}

	void updateFrame(const double frameTime)
	{
	}
};

#endif // ifndef _ABSTRACTELECTRICDEVICE_H_

