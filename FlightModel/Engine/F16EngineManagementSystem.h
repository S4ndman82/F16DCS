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

namespace F16
{
	//class F16FuelOilCooler

	//class F16OilSystem

	//class F16FuelControl

	//class F16EEC

	/*
	class F16Alternator
	{};
	*/

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

		//F16BleedAirSystem BleedAir;
		//BleedAir.pEpu = &Epu;

	public:
		F16EngineManagementSystem(F16Atmosphere *atmos, F16FuelSystem *fuels) 
			: pAtmos(atmos)
			, pFuel(fuels)
			, Engine(ET_F100PW200, atmos)
			, Gearbox()
		{}
		~F16EngineManagementSystem() {}

		void initEngineOff()
		{
			Engine.throttleInput = 0;
			Engine.fuelPerFrame = 0;

			Engine.starting = false;
			Engine.stopping = false;

			// temporary for testing
			Engine.isIgnited = false;
		}
		void initEngineIdle()
		{
			Engine.throttleInput = 1;
			Engine.fuelPerFrame = 1;

			Engine.starting = false;
			Engine.stopping = false;

			// temporary for testing
			Engine.isIgnited = true;
		}
		void initEngineCruise()
		{
			Engine.throttleInput = 50;
			Engine.fuelPerFrame = 10;

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
			Engine.fuelPerFrame = 0;
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
				Engine.percentPower = Engine.throttleInput * 0.6923;
			}
			else
			{
				Engine.percentPower = Engine.throttleInput *4.5455 - 354.55;
			}
			Engine.percentPower = limit(Engine.percentPower, 0.0, 100.0);
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
			Epu.updateFrame(frameTime);
			JFS.updateFrame(frameTime);
			Gearbox.updateFrame(frameTime);

			//pFuel->updateFrame(frameTime);
			Engine.updateFrame(frameTime);
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

	};
}

#endif // ifndef _F16ENGINEMANAGEMENTSYSTEM_H_

