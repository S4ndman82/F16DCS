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

	// add:
	// - currently used (selected) station
	// - support for internal tanks and external (pylons)
	class F16Fuel_Tank
	{
	public:
		int station; // do we need this here or not?

		double volume; // capacity of tank
		double fuel; // amount of fuel in tank

		double x;
		double y;
		double z;

		F16Fuel_Tank(double _volume = 0, double _fuel = 0)
			: station(0)
			, volume(_volume)
			, fuel(_fuel)
			, x(0)
			, y(0)
			, z(0)
		{}
		~F16Fuel_Tank() {}

		// add what is possible, return remaining if full
		double addFuel(const double addition)
		{
			double space = volume - fuel;
			if (space < addition)
			{
				fuel = volume; // set to max
				return (addition - space); // overflow
			}
			fuel += addition;
			return 0;
		}

		double decFuel(const double decrement)
		{
			if (fuel < decrement)
			{
				double tmp = decrement - fuel;
				fuel = 0; // set to min
				return tmp; // remaining
			}
			fuel -= decrement;
			return 0;
		}

		double getSpace() const
		{
			return (volume - fuel);
		}
	};

	class F16FuelSystem
	{
	protected:

		bool is_unlimited_fuel;
		double previous_usage;

		// Internal fuel as calculated by the EFM fuel system modeling
		F16Fuel_Tank FwdFus1; // foward fuselage 1
		F16Fuel_Tank FwdFus2; // forward fuselage 2
		F16Fuel_Tank AftFus1; // aft fuselage
		F16Fuel_Tank LeftWing; // wing internal
		F16Fuel_Tank RightWing; // wing internal

		// externals: one per wing plus centerline (if equipped)
		F16Fuel_Tank ext_Center;
		F16Fuel_Tank ext_LWing;
		F16Fuel_Tank ext_RWing;

	public:
		// TODO: check units
		F16FuelSystem() 
			: is_unlimited_fuel(false)
			, previous_usage(0)
			, FwdFus1(3100)
			, FwdFus2(3100)
			, AftFus1(2800)
			, LeftWing(525)
			, RightWing(525)
			, ext_Center(1800)
			, ext_LWing(2300)
			, ext_RWing(2300)
		{}
		~F16FuelSystem() {}

		/* 
		// for weight-balance calculation,
		// we need amount of fuel in each tank and position
		double getFuelMass()
		{
			//return getInternalFuel() * weightconstant;
			return 0;
		}
		*/

		// is low fuel indication
		bool isLowFuel() const
		{
			// check remining fuel
			if (getInternalFuel() <= 500)
			{
				return true;
			}
			return false;
		}

		// TODO: check units
		// called on initialization and on refueling
		void setInternalFuel(const double fuel)
		{
			// distribute fuel to each tank for weight balance
			double addition = fuel;

			addition = AftFus1.addFuel(addition);
			addition = FwdFus2.addFuel(addition);
			addition = FwdFus1.addFuel(addition);

			addition = LeftWing.addFuel(addition);
			addition = RightWing.addFuel(addition);
		}
		double getInternalFuel() const
		{
			return (FwdFus1.fuel + FwdFus2.fuel + AftFus1.fuel + LeftWing.fuel + RightWing.fuel);
		}

		// not ready yet
		void setExternalFuel(int station, double fuel, double x, double y, double z)
		{
		}
		// not ready yet
		double getExternalFuel() const
		{
			return 0;
		}

		// TODO: check units
		void refuelAdd(const double fuel)
		{
			// aerial refuel support: add given amount of fuel
			setInternalFuel(fuel);
		}

		void dumpFuel(const double frameTime)
		{
			// amount of fuel dumped from tanks within frame,
			// decrement from tanks
		}

		void setUnlimitedFuel(bool status)
		{
			is_unlimited_fuel = status;
		}

		double getUsageSinceLastFrame() const
		{
			return previous_usage;
		}
		void clearUsageSinceLastFrame()
		{
			previous_usage = 0;
		}

		// TODO: check units
		void updateFrame(const double fuelPerFrame, const double frameTime)
		{
			// TODO: decrement from tanks by order of usage
			// TODO: usage by throttle setting / altitude?

			if (is_unlimited_fuel == true)
			{
				return;
			}
			if (getInternalFuel() == 0 && is_unlimited_fuel == false)
			{
				return;
			}

			previous_usage += fuelPerFrame; // add to usage since last time updated

			double fueltmp = fuelPerFrame;

			fueltmp = RightWing.decFuel(fueltmp);
			fueltmp = LeftWing.decFuel(fueltmp);

			fueltmp = FwdFus1.decFuel(fueltmp);
			fueltmp = FwdFus2.decFuel(fueltmp);
			fueltmp = AftFus1.decFuel(fueltmp);
		}
	};
}

#endif // ifndef _F16FUELSYSTEM_H_
