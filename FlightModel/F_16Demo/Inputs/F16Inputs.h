/*
	Keep input enum in separate file: this can get potentially long list
	so don't define it in .cpp source directly.
*/

#ifndef _F16INPUTS_H_
#define _F16INPUTS_H_

// These are taken from Export.lua
//
// Used in ed_fm_set_command() and ed_fm_get_param()
//
enum F16InputCommands
{
	Gear				= 68, // Gear (toggle)		(doesn't work)
	Canopy				= 71, // Canopy				(doesn't work)

	Flaps				= 72, // Flaps (toggle)		(doesn't work)
	AirBrake			= 73, // Air brake (toggle)	(doesn't work)

	WheelBrakesOn		= 74, // Wheel brakes on	(doesn't work)
	WheelBrakesOff		= 75, // Wheel brakes off	(doesn't work)
	FlapsOn				= 145, // Flaps on			(doesn't work)
	FlapsOff			= 146, // Flaps off			(doesn't work)
	AirBrakeOn			= 147, // Air brake on		(doesn't work)
	AirBrakeOff			= 148, // Air brake off		(doesn't work)
	LandingGearUp		= 430, // Gear up			(doesn't work)	
	LandingGearDown		= 431, // Gear down			(doesn't work)

	EnginesStart		= 309, // Engines start 				
	EnginesStop			= 310, // Engines stop 			
	LeftEngineStart		= 311, // Left engine start 			
	RightEngineStart	= 312, // Right engine start 			
	LeftEngineStop		= 313, // Left engine stop 				
	RightEngineStop		= 314, // Right engine stop 			
	PowerOnOff			= 315, // Power on/off 					

	//WheelBrakeLeft = ;
	//WheelBrakeRight = ;

	//NoseWheelSteering

	JoystickPitch		= 2001,	
	JoystickRoll		= 2002,
	JoystickYaw			= 2003,
	JoystickThrottle	= 2004,
	JoystickLeftEngineThrottle = 2005,
	JoystickRightEngineThrottle = 2006,

	MouseCameraRotateLeftRight = 2007, // mouse camera rotate left/right  
	MouseCameraRotateUpDown = 2008, // mouse camera rotate up/down
	MouseCameraZoom = 2009 // mouse camera zoom 
};

// TODO: trivial container for values or hard-coded for efficiency?
//std::map<F16InputCommands,double>

#endif // ifndef _F16INPUTS_H_
