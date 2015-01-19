#include "../stdafx.h"
#include <math.h>

namespace F16
{
	// Simple atmospheric calculations
	class F16Atmosphere
	{
	protected:
		// internally used temporary values
		double temp;
		double rho;

	public:
		double		ambientTemperature_DegK;	// Ambient temperature (kelvon)
		double		ambientDensity_KgPerM3;		// Ambient density (kg/m^3)
		double		dynamicPressure_LBFT2;		// Dynamic pressure (lb/ft^2)
		double		mach; // Well..Mach, yeah

		double		altitude_FT;			// Absolute altitude MSL (ft)
		double		ps_LBFT2;			// Ambient calculated pressure (lb/ft^2)

		F16Atmosphere() 
			: temp(0)
			, rho(0)
			, ambientTemperature_DegK(0)
			, ambientDensity_KgPerM3(0)
			, dynamicPressure_LBFT2(0)
			, mach(0)
			, altitude_FT(0)
			, ps_LBFT2(0)
		{}
		~F16Atmosphere() {}

		void setAtmosphere(const double temperature, const double density, const double altitude, const double pressure)
		{
			ambientTemperature_DegK = temperature;
			ambientDensity_KgPerM3 = density; 
			altitude_FT = altitude;
			ps_LBFT2 = pressure;

			// calculate some helpers already
			temp = ambientTemperature_DegK * 1.8; // In Deg Rankine
			rho = ambientDensity_KgPerM3 * 0.00194032033;
		}

		void updateFrame(const double vt)
		{
			mach = (vt) / sqrt(1.4*1716.3*temp);
			dynamicPressure_LBFT2 = .5*rho*pow(vt, 2);
		}
	};
}
