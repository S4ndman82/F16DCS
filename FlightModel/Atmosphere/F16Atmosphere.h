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
	double		ambientTemperature;	// Ambient temperature (kelvin)
	double		ambientDensity;		// Ambient density (kg/m^3) (rho in some equations)
	double		dynamicPressure;	// Dynamic pressure (Pa == N/m^2) (velocity pressure)
	double		speed_of_sound;		// (meters/sec)
	double		ambientPressure;	// atmosphere pressure (Pa == N/m^2)
	double		altitude;			// Absolute altitude MSL (meters)
	double		totalVelocity;		// velocity in m/s
	double		machNumber;			// M, gas compressibility, velocity per speed of sound
	double		QcOverPs;			// qbar/ps in some spec

	F16Atmosphere() 
		: wind()
		, velocity_world_cs()
		, m_airspeed()
		, ambientTemperature(0)
		, ambientDensity(0)
		, dynamicPressure(0)
		, speed_of_sound(0)
		, ambientPressure(0)
		, altitude(0)
		, totalVelocity(0)
		, machNumber(0)
		, QcOverPs(0)
	{}
	~F16Atmosphere() {}

	void setAtmosphere(const double temperature, const double density, const double soundspeed, const double alt, const double pressure)
	{
		ambientTemperature = temperature;
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
		double speedsqr = m_airspeed.x * m_airspeed.x + m_airspeed.y * m_airspeed.y + m_airspeed.z * m_airspeed.z;
		totalVelocity = sqrt(speedsqr);
		//dynamicPressure = .5 * ambientDensity * pow(totalVelocity, 2);
		dynamicPressure = .5 * ambientDensity * speedsqr;
		QcOverPs = dynamicPressure / ambientPressure;
		if (speed_of_sound > 0) // avoid crash in case we don't have this yet..
		{
			// "flow speed" over speed of sound
			machNumber = totalVelocity / speed_of_sound;
		}
	}

	/*
	// note: for testing, not checked
	double getStaticAirTemperature() const
	{
		double correctionFactor = 1/(1+r0.2M^2)

		return ambientTemperature * correctionFactor;
	}
	*/
	// note: for testing, not checked
	double getTrueAirspeed() const
	{
		// should use computed static air temperature in this case?
		double temp = sqrt(ambientTemperature);
		temp *= machNumber;
		temp *= 20.0468;
		return temp;
	}

	// q = impact pressure aka. stagnation pressure aka. pitot pressure: 
	// calibrated airspeed
	//
	// stagnation pressure? (used to detect transonic speeds?)
	// dynamic pressure to static pressure ratio
	//
	// get Qc/Ps ratio
	// 
	double getQcOverPs() const
	{
		return QcOverPs;
	}

	double getAltitudeFeet() const
	{
		return altitude * F16::meterToFoot; // meters to feet
	}
	double getTotalVelocityKTS() const
	{
		return totalVelocity * F16::metersToKnots;
	}
	double getAeroTotalVelocityFPS() const
	{
		// to feets per second
		double totalVelocity_FPS = totalVelocity * F16::meterToFoot;

		// in original nlplant there is 2*vt, but is that because lift and drag 
		// are not calculated for both sides separately? (no support for differential deflections)
		return 2 * totalVelocity_FPS; // <- is this a bug?
	}

	void getAirspeed(Vec3 &airSpeed) const
	{
		airSpeed = m_airspeed;
	}
};

#endif // ifndef _F16ATMOSPHERE_H_
