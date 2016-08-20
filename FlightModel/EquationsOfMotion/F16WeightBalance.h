/*
	weight balance calculations, determine center of gravity
*/

#ifndef _F16WEIGHTBALANCE_H_
#define _F16WEIGHTBALANCE_H_

#include "../stdafx.h"
#include <math.h>

#include "include/ED_FM_Utility.h"		// Provided utility functions that were in the initial EFM example
#include "include/F16Constants.h"		// Common constants used throughout this DLL

namespace F16
{
	// mostly amount of fuel at different stations matter here
	// (unless we start adding payload support into flight model)
	class F16WeightBalance
	{
	public:
		F16WeightBalance() {}
		~F16WeightBalance() {}

	};
}

#endif // ifndef _F16WEIGHTBALANCE_H_
