#ifndef _F16ATMOSPHERE_H_
#define _F16ATMOSPHERE_H_

#include "../stdafx.h"
#include <math.h>

#include "include/ED_FM_Utility.h"		// Provided utility functions that were in the initial EFM example
#include "include/F16Constants.h"		// Common constants used throughout this DLL

namespace F16
{
	// Simple atmospheric calculations
	class F16Atmosphere
	{
	protected:
		// internally used temporary values
		double rho;

		//-----------------------------------------------------------------
		// The local winds acting on the air vehicle as calculated by the
		// DCS Simulation
		//
		// Units: Meters/(Second^2)
		//-----------------------------------------------------------------
		Vec3	wind;
		//-----------------------------------------------------------------
		// Absolute velocities of the air vehicle as calculated by DCS World
		//
		// Units: Meters/(Second^2)
		//-----------------------------------------------------------------
		Vec3	velocity_world_cs;
		//-----------------------------------------------------------------
		// Get the total absolute velocity acting on the aircraft with wind included
		// using english units so airspeed is in feet/second here
		Vec3	m_airspeed;

	public:
		double		ambientTemperature_DegK;	// Ambient temperature (kelvon)
		double		ambientDensity_KgPerM3;		// Ambient density (kg/m^3)
		double		dynamicPressure_LBFT2;		// Dynamic pressure (lb/ft^2)
		double		speed_of_sound;				// (meters/sec)
		double		mach; // Well..Mach, yeah

		double		altitude_FT;		// Absolute altitude MSL (ft)
		double		ps_LBFT2;			// Ambient calculated pressure (lb/ft^2)
		double		totalVelocity_FPS;	// Total velocity (always positive) (ft/s)

		F16Atmosphere() 
			: rho(0)
			, wind()
			, velocity_world_cs()
			, m_airspeed()
			, ambientTemperature_DegK(0)
			, ambientDensity_KgPerM3(0)
			, dynamicPressure_LBFT2(0)
			, speed_of_sound(0)
			, mach(0)
			, altitude_FT(0)
			, ps_LBFT2(0)
			, totalVelocity_FPS(0)
		{}
		~F16Atmosphere() {}

		void setAtmosphere(const double temperature, const double density, const double soundspeed, const double altitude, const double pressure)
		{
			ambientTemperature_DegK = temperature;
			ambientDensity_KgPerM3 = density; 
			altitude_FT = altitude * F16::meterToFoot; // meters to feet
			ps_LBFT2 = pressure * 0.020885434273; // (N/m^2) to (lb/ft^2)
			speed_of_sound = soundspeed;

			// calculate some helpers already
			rho = ambientDensity_KgPerM3 * 0.00194032033;
		}

		void setAirspeed(const double vx, const double vy, const double vz, const double wind_vx, const double wind_vy, const double wind_vz)
		{
			velocity_world_cs.x = vx;
			velocity_world_cs.y = vy;
			velocity_world_cs.z = vz;

			wind.x = wind_vx;
			wind.y = wind_vy;
			wind.z = wind_vz;

			// Get the total absolute velocity acting on the aircraft with wind included
			// using english units so airspeed is in feet/second here
			m_airspeed.x = velocity_world_cs.x - wind.x;
			m_airspeed.y = velocity_world_cs.y - wind.y;
			m_airspeed.z = velocity_world_cs.z - wind.z;
		}

		void updateFrame(double frameTime)
		{
			totalVelocity_FPS = sqrt(m_airspeed.x * m_airspeed.x + m_airspeed.y * m_airspeed.y + m_airspeed.z * m_airspeed.z) * F16::meterToFoot;
			if (totalVelocity_FPS < 0.01)
			{
				totalVelocity_FPS = 0.01;
			}

			double temp = ambientTemperature_DegK * 1.8; // In Deg Rankine

			// Call the atmosphere model to get mach and dynamic pressure
			// I'm used to english units so I am using LB/FT^2 for the pressures
			mach = (totalVelocity_FPS) / sqrt(1.4 * 1716.3 * temp);
			dynamicPressure_LBFT2 = .5 * rho * pow(totalVelocity_FPS, 2);
		}

		void getAirspeed(Vec3 &airSpeed) const
		{
			airSpeed = m_airspeed;
		}
	};
}

#endif // ifndef _F16ATMOSPHERE_H_
