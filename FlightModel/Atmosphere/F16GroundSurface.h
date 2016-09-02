#ifndef _F16GROUNDSURFACE_H_
#define _F16GROUNDSURFACE_H_

#include "../stdafx.h"
#include <math.h>

#include "include/ED_FM_Utility.h"		// Provided utility functions that were in the initial EFM example
#include "include/F16Constants.h"		// Common constants used throughout this DLL

#include "Atmosphere/F16Atmosphere.h"			//Atmosphere model functions

namespace F16
{
	// This will be used in two ways:
	// - to determine altitude and ground effect lift from surface below the aircraft
	// - calculate orientation relative to surface and weight on each wheel
	//
	// The second part is already given by DCS so that might not be needed.
	// Just the plan though if needed.
	//
	class F16GroundSurface
	{
	protected:
		F16Atmosphere *pAtmos;

		double surfaceHeight;
		double surfaceHeightWithObj;
		unsigned surfaceType;

		Vec3 m_surfaceNormal;

	public:
		F16GroundSurface(F16Atmosphere *atmos)
			: pAtmos(atmos)
			, surfaceHeight(0)
			, surfaceHeightWithObj(0)
			, surfaceType(0)
			, m_surfaceNormal()
		{}
		~F16GroundSurface() {}

		void setSurface(double h, double h_obj, unsigned surface_type, Vec3 &sfcNormal)
		{
			surfaceHeight = h; 
			surfaceHeightWithObj = h_obj;
			surfaceType = surface_type;
			m_surfaceNormal = sfcNormal;
		}

		void groundEffect(double frameTime)
		{
			// downwash at entire trailing edge of wing,
			// also wingtip vortices
			// -> lift increase
			// -> reduced drag
			// wingspan, altitude, angle of attack


			if (surfaceHeightWithObj > surfaceHeight)
			{
				// ground effect disrupted by object (or uneven surface?)
				// TODO: more detailed version
				return;
			}

			//calculation of ground surface with aircraft normal
			// -> max effect when aligned, reduced as banking

			// trailing edge downwash effect (are flaps down?)
			// wingtip vortices (sidewinders?)

			// we might need airspeed and air pressure to determine magnitude of lift effect
			Vec3 airSpeed;
			pAtmos->getAirspeed(airSpeed);

			// also reduction of induced drag when in ground effect -> less thrust needed

			// TODO: check height, set for ground effect simulation?
			// also if weight on wheels?
			if (F16::wingSpan_m >= surfaceHeight /*&& F16::LandingGear.isWoW() == false*/)
			{
				// in ground effect with the surface?
				// flying above ground, no weight on wheels?

				//double diff = F16::wingSpan_FT - (F16::meterToFoot*surfaceHeight);

			}

		}

		void updateFrame(double frameTime)
		{
			groundEffect(frameTime);
		}
	};
}

#endif // ifndef _F16GROUNDSURFACE_H_
