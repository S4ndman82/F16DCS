#ifndef _F16GEARBOX_H_
#define _F16GEARBOX_H_

#include "../stdafx.h"

/*
main engine is connected to gearbox which powers 
hydraulic pumps, fuel pumps and has some torque resistance on engine.

also JFS is connected to gearbox for rotating engine, especially during startup.
*/

namespace F16
{
	class F16Gearbox
	{
	protected:
		double engineTorque;
		double engineRpm;

		//F16FuelPump *pFuelPump;
		//F16MainGenerator *pGenerator;
		//F16HydraulicSystem *pHydrPump;

	public:
		F16Gearbox() {}
		~F16Gearbox() {}

		// update with engine/APU rpm/torque
		// and power consumption
		void updateFrame(const double engineRPM, const double frameTime)
		{
			// APU (JFS) torque -> to engine for starting
			// without APU, engine -> auxiliary systems
			// airstart when engine rotating? (disconnect aux systems?)

			/*
			if (engineTorque > threshold)
			{
				pAux->update(engineTorque, engineRpm)
			}
			**/
		
		}


	};
}

#endif // ifndef _F16GEARBOX_H_

