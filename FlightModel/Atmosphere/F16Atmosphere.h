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
	double		QcOverPs;			// qbar/ps in some spec, impact pressure over static pressure

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
		m_airspeed.x = velocity_world_cs.x - wind.x;
		m_airspeed.y = velocity_world_cs.y - wind.y;
		m_airspeed.z = velocity_world_cs.z - wind.z;
	}

	void updateFrame(const double frameTime)
	{
		// reuse V^2, calculate total velocity and dynamic pressure (1/2 * p * V^2)
		double speedsqr = m_airspeed.x * m_airspeed.x + m_airspeed.y * m_airspeed.y + m_airspeed.z * m_airspeed.z;
		totalVelocity = sqrt(speedsqr);
		dynamicPressure = .5 * ambientDensity * speedsqr; // pow(totalVelocity, 2)
		QcOverPs = dynamicPressure / ambientPressure;
		if (speed_of_sound > 0) // avoid crash in case we don't have this yet..
		{
			// "flow speed" over speed of sound
			machNumber = totalVelocity / speed_of_sound;
		}
	}

	// calculate dynamic viscosity coefficient
	// according to Sutherland's formula
	double getDynamicViscosity() const
	{
		const double scAir = 120; // sutherland's constant C in Kelvins

		/*
		double mu0 = 18.27; // uPa*s
		double T0 = 291.15; // K
		double temp = pow((ambientTemperature / T0), 1.5); // T/T0 ^3/2
		double temp2 = (T0 + scAir) / (ambientTemperature + scAir);
		return mu * temp * temp2;
		*/

		// shorter with constant lambda: 
		const double lambdaAir = 1.512041288;
		double temp = pow(ambientTemperature, 1.5) / (ambientTemperature + scAir);
		return lambdaAir * temp;

		// abosolute (dynamic) viscosity of air: 1.983*10^-5
		//return 1.983e-5;
	}

	// kinematic viscosity of air
	// (SI has Stokes: 1 St == 10^-4 m^2/s == 1 cm^2/s)
	// (poise P == 0.1 Pa*s (Pascal*seconds)
	double getKinematicViscosity() const
	{
		const double airAbsV = getDynamicViscosity();

		// v = u / p
		// ..where:
		// v is kinematic viscosity (m^2/s)
		// u is absolute/dynamic viscosity (N s/m^2)
		// p is density (kg/m^3)

		if (ambientDensity != 0)
		{
			return airAbsV / ambientDensity;
		}
		return 0;
	}

	/*
	// pressure ratio Pt/Ps
	double getPressureRatio(const double machNumber) const
	{
	}
	*/

	// note: for testing, not checked
	double getStaticAirTemperature(const double measTemp) const
	{
		//double Tm = measTemp; // measured (indicated) temperature

		double r = 0.0; // <- value from somewhere..
		double M = 0.2 * pow(machNumber, 2);
		double correctionFactor = 1 / (1 + r * M);

		return measTemp * correctionFactor;
	}
	// note: for testing, not checked
	double getTrueAirspeed() const
	{
		// should use computed static air temperature in this case?
		double Ts = getStaticAirTemperature(ambientTemperature);

		double Vt = sqrt(Ts);
		Vt *= machNumber;
		Vt *= 20.0468;
		return Vt;
	}

	// get impact pressure Qc
	// calculations from Introduction to Avionics Systems by R.P.G. Collinson
	// (mistakes are mine)
	double getImpactPressure(const double Vc) const
	{
		double Qc = 0.0;

		// used in multiple cases
		double V = (Vc / 340.294);

		if (Vc <= 340.3) // 340.3m/s ~661.5kts
		{
			Qc = 1 + 0.2 * pow(V, 3.5) -1;
		}
		else
		{
			double upper = 166.92 * pow(V, 7);
			double denom = 7 * pow(V, 2) - 1;
			denom = pow(denom, 2.5);

			if (denom != 0)
			{
				Qc = upper / denom - 1;
			}
		}

		// final step in both cases
		Qc *= 101.325;

		return Qc;
	}

	// q = impact pressure aka. stagnation pressure aka. pitot pressure: 
	// calibrated airspeed
	//
	// stagnation pressure? (used to detect transonic speeds?)
	// dynamic pressure to static pressure ratio
	//
	// dynamic pressure (due to compressibility?) higher at higher speed
	// -> ratio over 1
	//
	// get Qc/Ps ratio
	// 
	double getQcOverPs() const
	{
		return QcOverPs;
	}

	// deep-copy of struct -> should not matter too much
	const Vec3& getAirspeed() const
	{
		return m_airspeed;
	}
};

#endif // ifndef _F16ATMOSPHERE_H_
