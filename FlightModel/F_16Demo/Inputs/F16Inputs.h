/*
	Keep input enum in separate file: this can get potentially long list
	so don't define it in .cpp source directly.
*/

#ifndef __F16INPUTS__
#define __F16INPUTS__

// These are taken from Export.lua
//
// Used in ed_fm_set_command() and ed_fm_get_param()
//
enum F16InputCommands
{
	Flaps				= 72, // Flaps (toggle)		(doesn't work)
	WheelBrakesOn		= 74, // Wheel brakes on	(doesn't work)
	WheelBrakesOff		= 75, // Wheel brakes off	(doesn't work)
	FlapsOn				= 145, // Flaps on			(doesn't work)
	FlapsOff			= 146, // Flaps off			(doesn't work)
	AirBrakeOn			= 147, // Air brake on		(doesn't work)
	AirBrakeOff			= 148, // Air brake off		(doesn't work)
	LandingGearUp		= 430, // Gear up			(doesn't work)	
	LandingGearDown		= 431, // Gear down			(doesn't work)

	JoystickPitch		= 2001,	
	JoystickRoll		= 2002,
	JoystickYaw			= 2003,
	JoystickThrottle	= 2004,
	JoystickLeftEngineThrottle = 2005,
	JoystickRightEngineThrottle = 2006
};

// TODO: trivial container for values or hard-coded for efficiency?
//std::map<F16InputCommands,double>

#endif
