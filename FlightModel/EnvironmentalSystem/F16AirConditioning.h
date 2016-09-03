#ifndef _F16AIRCONDITIONING_H_
#define _F16AIRCONDITIONING_H_

#include "include/ED_FM_Utility.h"		// Provided utility functions that were in the initial EFM example
#include "include/F16Constants.h"		// Common constants used throughout this DLL

namespace F16
{
	// heating/cooling temperature control
	// electric operated

	class F16AirConditioning
	{
	protected:
		enum eAirSourceKnob // air source knob
		{
			EAS_OFF,
			EAS_NORM,
			EAS_DUMP,
			EAS_RAM
		};
		enum eTempKnob
		{
			ETMP_AUTO,
			ETMP_MAN,
			ETMP_OFF
		};
		enum eDefogKnob
		{
			EDFG_MIN,
			EDFG_MAX
		};
		eAirSourceKnob airsourceKnob = EAS_OFF;
		eTempKnob tempKnob = ETMP_AUTO;
		eDefogKnob defogKnob = EDFG_MIN;

	public:
		F16AirConditioning() {}
		~F16AirConditioning() {}

		void updateFrame(const double frameTime)
		{
			// heat transfer
		}
	};
}

#endif // ifndef _F16AIRCONDITIONING_H_
