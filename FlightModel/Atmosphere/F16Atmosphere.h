#ifndef _F16ATMOSPHERE_H_
#define _F16ATMOSPHERE_H_

#include <cmath>

#include "ED_FM_Utility.h"		// Provided utility functions that were in the initial EFM example
#include "F16Constants.h"		// Common constants used throughout this DLL

// Simple atmospheric calculations
class F16Atmosphere
{
protected:
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
	double		ambientTemperature_DegK;	// Ambient temperature (kelvin)
	double		ambientDensity;		// Ambient density (kg/m^3)
	double		dynamicPressure;	// Dynamic pressure (Pa)
	double		speed_of_sound;		// (meters/sec)
	double		ambientPressure;	// atmosphere pressure (N/m^2)
	double		altitude;			// Absolute altitude MSL (meters)
	double		totalVelocity;		// velocity in m/s
	double		machNumber;			// M, gas compressibility, velocity per speed of sound

	F16Atmosphere() 
		: wind()
		, velocity_world_cs()
		, m_airspeed()
		, ambientTemperature_DegK(0)
		, ambientDensity(0)
		, dynamicPressure(0)
		, speed_of_sound(0)
		, ambientPressure(0)
		, altitude(0)
		, totalVelocity(0)
		, machNumber(0)
	{}
	~F16Atmosphere() {}

	void setAtmosphere(const double temperature, const double density, const double soundspeed, const double alt, const double pressure)
	{
		ambientTemperature_DegK = temperature;
		ambientDensity = density; 
		altitude = alt;
		ambientPressure = pressure;
		speed_of_sound = soundspeed;
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

	void updateFrame(const double frameTime)
	{
		totalVelocity = sqrt(m_airspeed.x * m_airspeed.x + m_airspeed.y * m_airspeed.y + m_airspeed.z * m_airspeed.z);
		dynamicPressure = .5 * ambientDensity * pow(totalVelocity, 2);

		if (speed_of_sound > 0) // avoid crash in case we don't have this yet..
		{
			machNumber = totalVelocity / speed_of_sound;
		}
	}

	double getAltitudeFeet() const
	{
		return altitude * F16::meterToFoot; // meters to feet
	}
	double getTotalVelocityKTS() const
	{
		return totalVelocity * F16::metersToKnots;
	}
	double getTotalVelocityFPS() const
	{
		// to feets per second
		double totalVelocity_FPS = totalVelocity * F16::meterToFoot;
		if (totalVelocity_FPS < 0.01)
		{
			totalVelocity_FPS = 0.01;
		}
		return totalVelocity_FPS;
	}
	double getAmbientPressureLBFTSQ() const
	{
		return ambientPressure * F16::Nm_sq_to_lbft_sq; // (N/m^2) to (lb/ft^2)
	}
	double getDynamicPressureLBFTSQ() const
	{
		// Call the atmosphere model to get mach and dynamic pressure
		// I'm used to english units so I am using LB/FT^2 for the pressures
		double totalVelocity_FPS = getTotalVelocityFPS();
		double rho = ambientDensity * 0.00194032033;
		double dynamicPressure_LBFT2 = .5 * rho * pow(totalVelocity_FPS, 2);
		return dynamicPressure_LBFT2;
	}

	/*
	// this is old stuff, pointlessly complicated since we get speed of sound already
	double getMachSpeed() const
	{
		double tempR = ambientTemperature_DegK * F16::kelvin_to_rankine; // In Deg Rankine
		double soundspeed = sqrt(1.4 * 1716.3 * tempR);
		double totalVelocity_FPS = getTotalVelocityFPS();
		double mach = totalVelocity_FPS / soundspeed;
		return mach;
	}
	*/

	void getAirspeed(Vec3 &airSpeed) const
	{
		airSpeed = m_airspeed;
	}
};

#endif // ifndef _F16ATMOSPHERE_H_
