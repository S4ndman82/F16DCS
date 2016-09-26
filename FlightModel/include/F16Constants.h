#ifndef _F16CONSTANTS_H_
#define _F16CONSTANTS_H_

#pragma once

#include <cmath>

//-------------------------------------------------------
// Start of F-16 Simulation Variables
// Probably doesn't need it's own namespace or anything
// I just quickly did this to organize my F-16 specific
// variables, needs to be done better eventually
//-------------------------------------------------------
namespace F16
{
	const double		pi					= M_PI;			// Pi (3.14159....) - use value from math.h
	const double		degtorad			= M_PI / 180.0;	// 
	const double		radiansToDegrees	= 180.0 / M_PI;	// Conversion factor from radians to degrees - use value from math.h

	const double		wingSpan_FT				= 32.667;		// F-16 wing-span (ft)
	const double		wingSpan_m				= 10.00;		// F-16 wing-span (m)
	const double		wingArea_FT2			= 300.0;		// F-16 wing area (ft^2)
	const double		wingArea_m2				= 27.87;		// F-16 wing area (m^2)

	const double		wing_wetted_area_FT2				= 341.1; // from JSBSIM readme
	const double		ventral_fin_each_wetted_area_FT2	= 15.3; // from JSBSIM readme
	const double		horiz_tail_wetted_area_FT2			= 127.8; // from JSBSIM readme
	const double		vertical_tail_wetted_area_FT2		= 128.7; // from JSBSIM readme
	const double		fuselage_wetted_area_FT2			= 775.8; // from JSBSIM readme

	const double		length_m				= 15.03;
	const double		height_m				= 5.09;

	const double		meanChord_FT			= 11.32;		// F-16 mean aerodynamic chord (ft) 
	const double		meanChord_m				= 3.450336;		// mean aerodynamic chord (m) 

	const double		airbrakeArea_FT2		= 14.26;		// 
	const double		airbrakeArea_m2			= 1.48645;		// based on ~16ft^2 -> m^2

	//note: need dynamic CG to calculations: lift position as function of speed, mass balance by fuel/payload mass
	const double		referenceCG_PCT			= 0.35;			// Reference CG as a % of wing chord
	//const double		actualCG_PCT			= 0.30;			// Actual CG as a % of wing chord

	const double		inertia_Ix_KGM2			= 12874.0;		// Reference moment of inertia (kg/m^2) // 12662 in one source? (scale model?)
	const double		inertia_Iy_KGM2			= 75673.6;		// Reference moment of inertia (kg/m^2) // 53147 in one source? (scale model?)
	const double		inertia_Iz_KGM2			= 85552.1;		// Reference moment of inertia (kg/m^2) // 63035 in one source? (scale model?)
	const double		inertia_Ixz_KGM2		= 179.0;		// Reference moment of inertia (kg/m^2) 

	const double		metersToKnots			= 1.943844;		// m/s to knots
	const double		knotsToMeters			= 0.514444;		// knots to m/s

	const double		meterToFoot				= 3.28084;		// Meter to foot conversion factor
	const double		feetToMeter				= 0.3048;		// multiplier, ft to m
	const double		inchesToCentim			= 2.54;			// multiplier, in to cm

	//const double		kg_to_newtons			= 9.80665002864;
	//const double		Nm_sq_to_lbft_sq		= 0.020885434273; // (N/m^2) to (lb/ft^2);

	const double		lb_to_kg				= 0.45359237; // multiplier, lb to kg
	const double		lbf_to_N				= 4.44822162825; // multiplier, pound force to Newtons
	const double		lbf_to_Nm				= 1.35581795; // multiplier, "pound-foot" to Newtonmeters

	const double		weight_empty_kg			= 9207.9;
	const double		internal_fuel_kg		= 2685.2;

	const double		gallon_to_litre			= 3.7854118;

	const double		standard_gravity		= 9.80665; // "average", m/s^2

	//const double		kelvin_to_rankine		= 1.8; // 

	/*
	////////////////////
	// some additional stuff that should be used as functions perhaps (see if we need them)
	// VS2013 does not yet support constexpr though, need upgrade to 2015.. :(
	//
	//constexpr 
	double rankineToKelvin(double rankine) { return rankine / 1.8; }
	//constexpr 
	double kelvinToRankine(double kelvin) { return kelvin * 1.8; }

	// alternative, by division
	double lbtokg(double lb) { return lb / 2.2046; }
	double fttom(double ft) { return ft / 3.2808; }
	double intocm(double in) { return in / 0.39370; }
	*/
}

#endif // ifndef _F16CONSTANTS_H_

