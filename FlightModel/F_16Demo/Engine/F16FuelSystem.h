#ifndef _F16FUELSYSTEM_H_
#define _F16FUELSYSTEM_H_

#include "../stdafx.h"

namespace F16
{
	// we need fuel usage curve for the used engine here

	// also calculation for the mass of the fuel and position of it

	// F100-PW-229
	//Specific fuel consumption: Military thrust: 77.5 kg/(kN·h) (0.76 lb/(lbf·h)) Full afterburner: 197.8 kg/(kN·h) (1.94 lb/(lbf·h))

	// three externals: wings and centerline station
	// five internals: left and right wing tanks, two forward (F-1, F-2) and one aft (A-1) fuselage tank.
	// fwd & aft reservoir

	// total internal: [A] 6950 +- 300 (JP-4), 7290 +- 300 (JP-5/8) [B] 5650 +- 300, 5930 +- 300
	// [A]: fwd fuselage: 3100 +- 100 / 3250 +- 100, 
	// [B]: fwd fuselage: 1800 +- 100, 1890 +- 100

	class F16FuelSystem
	{
	public:
		// add:
		// - currently used (selected) station
		// - support for internal tanks and external (pylons)
		/*
		struct Fuel_Tank
		{
			int station;
			double volume; // capacity of tank
			double fuel; // amount of fuel in tank

			double x;
			double y;
			double z;

			Fuel_Tank()
				: station(0)
				, volume(0)
				, fuel(0)
				, x(0)
				, y(0)
				, z(0)
			{}
		};
		//Fuel_Tank fuel_tank[3]; // three at most? do we include externals in this?
		Fuel_Tank fuselage;
		*/

		// Internal fuel as calculated by the EFM fuel system modeling
		// total amount of fuel
		double  internal_fuel;

		// total weight of current fuel
		double fuel_weight; 

		F16FuelSystem() 
			: internal_fuel(0)
			, fuel_weight(0)
			//: fuselage()
		{
			/*
			// or zero-based?
			fuel_tank[0].station = 1;
			fuel_tank[1].station = 2;
			fuel_tank[2].station = 3;
			*/
		}
		~F16FuelSystem() {}

		double getFuelMass()
		{
			return fuel_weight;
		}

		void setExternalFuel(int station, double fuel, double x, double y, double z)
		{
		}
		double getExternalFuel()
		{
			return 0;
		}

		void refuel(const double frameTime)
		{
			// aerial refuel support?
		}

		void dumpFuel(const double frameTime)
		{
			// amount of fuel dumped from tanks within frame,
			// decrement from tanks
		}

		// we need current thrust-setting (and altitude?) to calculate fuel usage for current frame
		//
		//void updateFrame(double thrust, double frameTime);
		void updateFrame(const double throttle, const double frameTime);
	};

	//void F16FuelSystem::updateFrame(double thrust, double frameTime)
	void F16FuelSystem::updateFrame(const double throttle, const double frameTime)
	{
		double fuelPerFrame =  10 * throttle * frameTime; //10 kg persecond
		if (fuelPerFrame > internal_fuel)
		{
			// set to what is remaining
			fuelPerFrame = internal_fuel;
		}

		internal_fuel -= fuelPerFrame; // remove

		//fuel_weight = internal_fuel * weightconstant;
	/*
		fuel_consumption_since_last_time =  10 * throttle * dt; //10 kg persecond
		if (fuel_consumption_since_last_time > Fuel.internal_fuel)
			fuel_consumption_since_last_time = internal_fuel;
		internal_fuel -= fuel_consumption_since_last_time;
	 */
	}
}

#endif // ifndef _F16FUELSYSTEM_H_
