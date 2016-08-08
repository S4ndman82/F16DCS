#ifndef _F16AIRCONDITIONING_H_
#define _F16AIRCONDITIONING_H_

#include "../stdafx.h"

// ENVIRONMENTAL CONTROL SYSTEM (ECS)
// cockpit pressure, temperature, sealing, defogging, G-suit pressure, fuel tank pressure, equipment cooling

namespace F16
{
	// heating/cooling temperature control
	// electric operated

	// bleed air pressure from engine?

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
