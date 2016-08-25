/*
	weight balance calculations, determine center of gravity

sources:
- 68548, Model of F-16 Fighter Aircraft - Equation of Motions

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
	protected:

		// calculate new center of gravity
		// for using in motion calculations
		Vec3 balanced_center_of_gravity;
		//double total_mass_kg; // new "wet" mass with balance

		// reference location (0.35)
		Vec3 original_cog;

		// dry mass used with original cog
		double dry_mass_kg;

	public:
		F16WeightBalance() 
			: balanced_center_of_gravity()
			, original_cog()
			, dry_mass_kg(0)
		{}
		~F16WeightBalance() {}

		// used to initialize
		void setMassState(double mass_kg, Vec3 &center_of_gravity)
		{
			dry_mass_kg = mass_kg;
			original_cog = center_of_gravity;
			balanced_center_of_gravity = center_of_gravity;
		}

		void balance(double mass, Vec3 &position, Vec3 &size)
		{
			// update cog with given mass, position and bounding size
			// 
		}

	};
}

#endif // ifndef _F16WEIGHTBALANCE_H_
