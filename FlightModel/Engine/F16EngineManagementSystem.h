#ifndef _F16ENGINEMANAGEMENTSYSTEM_H_
#define _F16ENGINEMANAGEMENTSYSTEM_H_

#include "../stdafx.h"

#include "include/ED_FM_Utility.h"		// Provided utility functions that were in the initial EFM example
#include "include/F16Constants.h"		// Common constants used throughout this DLL

#include "Atmosphere/F16Atmosphere.h"

#include "Engine/F16Engine.h"					//Engine model functions
#include "Engine/F16FuelSystem.h"				//Fuel usage and tank usage functions

#include "Engine/F16JFS.h"						//APU
#include "Engine/F16EPU.h"						//Emergency Power Unit (electric&hydraulic power)
#include "Engine/F16BleedAirSystem.h"
#include "Engine/F16Gearbox.h"


/*
 aka Engine Monitoring System (EMS)
*/
namespace F16
{
	//class F16FuelOilCooler

	//class F16OilSystem

	//class F16FuelControl

	/*
	class F16Alternator
	{};
	*/

	// PRI / SEC
	enum eABResetSwitch
	{
		AB_RESET,
		AB_NORM,
		ENGDATA
	};


	class F16EngineManagementSystem
	{
	public:
		F16EPU Epu;
		F16JFS JFS;

		F16Atmosphere *pAtmos;
		F16FuelSystem *pFuel;

		// main engine: turbofan with AB
		F16Engine Engine;

		// logic for EPU/JFS/main engine torque
		F16Gearbox Gearbox;

		// logic for bleed air to env system, anti-ice, back to engine etc.
		F16BleedAirSystem BleedAir;
		//BleedAir.pEpu = &Epu;

	public:
		F16EngineManagementSystem(F16Atmosphere *atmos, F16FuelSystem *fuels) 
			: pAtmos(atmos)
			, pFuel(fuels)
			, Engine(ET_F100PW200, atmos)
			, Gearbox()
			, BleedAir()
		{}
		~F16EngineManagementSystem() {}

		void initEngineOff()
		{
			Engine.throttleInput = 0;
			Engine.m_fuelPerFrame = 0;

			Engine.starting = false;
			Engine.stopping = false;

			// temporary for testing
			Engine.isIgnited = false;
		}
		void initEngineIdle()
		{
			Engine.throttleInput = 1;
			Engine.m_fuelPerFrame = 1;

			Engine.starting = false;
			Engine.stopping = false;

			// temporary for testing
			Engine.isIgnited = true;
		}
		void initEngineCruise()
		{
			Engine.throttleInput = 50;
			Engine.m_fuelPerFrame = 10;

			Engine.starting = false;
			Engine.stopping = false;

			// temporary for testing
			Engine.isIgnited = true;
		}

		void JfsStart()
		{
			// just direct start for now..
			startEngine();
		}
		void JfsStop()
		{
		}

		void startEngine()
		{
			Engine.starting = true;
			Engine.stopping = false;

			// temporary for testing
			Engine.isIgnited = true;
		}
		void stopEngine()
		{
			Engine.stopping = true;
			Engine.starting = false;

			// temporary for testing
			Engine.isIgnited = false;
			Engine.m_fuelPerFrame = 0;
		}

		// MaksRUD	=	0.85, -- Military power state of the throttle
		// ForsRUD	=	0.91, -- Afterburner state of the throttle
		void setThrottleInput(double value)
		{
			// old code, see if we need changes..
			double limited = limit(((-value + 1.0) / 2.0) * 100.0, 0.0, 100.0);

			// --------------------------- OLD STUFF
			// Coded from the simulator study document
			Engine.throttleInput = limited;
			if (Engine.throttleInput < 78.0)
			{
				Engine.m_percentPower = Engine.throttleInput * 0.6923;
			}
			else
			{
				Engine.m_percentPower = Engine.throttleInput *4.5455 - 354.55;
			}
			Engine.m_percentPower = limit(Engine.m_percentPower, 0.0, 100.0);

			// TODO: calculate fuel mixture needed for given throttle setting and flight conditions
		}

		double getEngineRpm() const
		{
			if (Engine.isIgnited == false) // ignore throttle setting if engine is not running
			{
				return 0;
			}

			// ED_FM_ENGINE_1_RPM:
			return (Engine.throttleInput / 100.0) * 3000;
		}
		double getEngineRelatedRpm() const
		{
			if (Engine.isIgnited == false)
			{
				return 0;
			}

			// ED_FM_ENGINE_1_RELATED_RPM:
			return (Engine.throttleInput / 100.0);
		}
		double getEngineThrust() const
		{
			if (Engine.isIgnited == false)
			{
				return 0;
			}

			// ED_FM_ENGINE_1_THRUST:
			return (Engine.throttleInput / 100.0) * 5000 * 9.81;
		}
		double getEngineRelatedThrust() const
		{
			if (Engine.isIgnited == false)
			{
				return 0;
			}

			// ED_FM_ENGINE_1_RELATED_THRUST:
			return (Engine.throttleInput / 100.0);
		}

		/*
		double getThrottleInput() const
		{
			return Engine.throttleInput;
		}
		*/

		double getFuelPerFrame() const
		{
			return Engine.getFuelPerFrame();
		}

		void updateFrame(const double frameTime)
		{
			// 
			//Engine CIVV control

			//pFuel->updateFrame(frameTime);
			Engine.updateFrame(frameTime);

			/*
			if (getEngineRpm() < minRpm)
			{
				Epu.start();
			}
			*/

			Epu.updateFrame(frameTime);
			JFS.updateFrame(frameTime);
			Gearbox.updateFrame(frameTime);
			BleedAir.updateFrame(frameTime);
		}


		float getAfterburnerDraw() const
		{
			return (float)Engine.afterburnerDraw;
		}

		// TODO: we'll need to control nozzle position:
		// for example, landing gear out -> open nozzle more 
		// to reduce possibility of power loss
		float getNozzlePos() const
		{
			return (float)Engine.afterburnerDraw;
		}

		// return gyroscopic effect of engine for motions,
		// use angular momentum directly now
		double getGyroscopicEffect() const
		{
			return Engine.engineParams.angular_momentum;
		}

	};
}

#endif // ifndef _F16ENGINEMANAGEMENTSYSTEM_H_

