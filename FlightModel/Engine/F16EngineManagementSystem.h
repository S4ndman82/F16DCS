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
		F16Engine *pEngine;

		//F16BleedAirSystem BleedAir;
		//BleedAir.pEpu = &Epu;

	public:
		F16EngineManagementSystem(F16Atmosphere *atmos, F16FuelSystem *fuels, F16Engine *engine) 
			: pAtmos(atmos)
			, pFuel(fuels)
			, pEngine(engine)
		{}
		~F16EngineManagementSystem() {}

		void JfsStart()
		{
			// just direct start for now..
			//pEngine->startEngine();
		}
		void JfsStop()
		{
		}

		void updateFrame(const double frameTime)
		{
			Epu.updateFrame(frameTime);
			JFS.updateFrame(frameTime);

			//pFuel->updateFrame(frameTime);
			//pEngine->updateFrame(frameTime);
		}

	};
}

#endif // ifndef _F16ENGINEMANAGEMENTSYSTEM_H_

