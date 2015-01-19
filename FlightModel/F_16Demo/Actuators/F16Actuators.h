#ifndef _F16ACTUATORS_H_
#define _F16ACTUATORS_H_

#include "../stdafx.h"
#include <memory.h>
#include "../UtilityFunctions.h"

namespace F16
{
	// TODO: speed brake handling..

	class F16Actuators
	{
	public:
		bool	simInitialized;

		double	elevatorPosition_DEG;
		double	elevatorRate_DEGPERSEC;
		double	aileronPosition_DEG;
		double	aileronRate_DEGPERSEC;
		double	rudderPosition_DEG;
		double	rudderRate_DEGPERSEC;

		F16Actuators() 
			: simInitialized(false)
			, elevatorPosition_DEG(0)
			, elevatorRate_DEGPERSEC(0)
			, aileronPosition_DEG(0)
			, aileronRate_DEGPERSEC(0)
			, rudderPosition_DEG(0)
			, rudderRate_DEGPERSEC(0)
		{}
		~F16Actuators() {}

		double  elevator_actuator(double elevatorCommanded_DEG, double frameTime)
		{
			if(!simInitialized)
			{
				elevatorPosition_DEG = elevatorCommanded_DEG;
				return elevatorPosition_DEG;		
			}

			elevatorRate_DEGPERSEC = 20.2 * (elevatorCommanded_DEG - elevatorPosition_DEG);

			elevatorRate_DEGPERSEC = limit(elevatorRate_DEGPERSEC,-60.0,60.0);

			elevatorPosition_DEG += (elevatorRate_DEGPERSEC * frameTime);

			elevatorPosition_DEG = limit(elevatorPosition_DEG, -25.0, 25.0);

			return elevatorPosition_DEG;
		}

		double  aileron_actuator(double aileronCommanded_DEG, double frameTime)
		{
			if(!simInitialized)
			{
				aileronPosition_DEG	 = aileronCommanded_DEG;
				return aileronPosition_DEG	;
			}

			aileronRate_DEGPERSEC = 20.2 * (aileronCommanded_DEG - aileronPosition_DEG	);

			aileronRate_DEGPERSEC = limit(aileronRate_DEGPERSEC,-56.0,56.0);

			aileronPosition_DEG	 += (aileronRate_DEGPERSEC * frameTime);

			aileronPosition_DEG = limit(aileronPosition_DEG, -21.5, 21.5);

			return aileronPosition_DEG;
		}

		double  rudder_actuator(double rudderCommanded_DEG, double frameTime)
		{
			if(!simInitialized)
			{
				rudderPosition_DEG	 = rudderCommanded_DEG;
				return rudderPosition_DEG	;
			}

			rudderRate_DEGPERSEC = 20.2 * (rudderCommanded_DEG - rudderPosition_DEG	);

			rudderRate_DEGPERSEC = limit(rudderRate_DEGPERSEC,-120.0,120.0);

			rudderPosition_DEG	 += (rudderRate_DEGPERSEC * frameTime);

			rudderPosition_DEG = limit(rudderPosition_DEG,-30.0,30.0);

			return rudderPosition_DEG;
		}
	};
}

#endif // ifndef _F16ACTUATORS_H_
