#ifndef _F16FCSCOMMON_H_
#define _F16FCSCOMMON_H_

#include <cmath>

#include "UtilityFunctions.h"

// if/when trimming support is needed,
// keep settings in one place.
class F16TrimState
{
public:
	// increments per "notch" for each axis
	const double pitchIncrement;
	const double rollIncrement;
	const double yawIncrement;

	// TODO: limits of trimmer for each axis
	//const double pitchLimit; // +/- 0.90 g/sec?
	//const double rollLimit; // +/- 7.5 deg/sec?
	//const double yawLimit;

	double trimPitch;
	double trimRoll;
	double trimYaw;

public:
	F16TrimState(const double pitch, const double roll, const double yaw)
		//: pitchLimit(5.0), rollLimit(5.0), yawLimit(5.0)
		: pitchIncrement(0.1), rollIncrement(0.1), yawIncrement(0.1)
		, trimPitch(pitch), trimRoll(roll), trimYaw(yaw)
	{}
	~F16TrimState() {}

	void pitchUp() 		{ trimPitch += pitchIncrement; }
	void pitchDown()	{ trimPitch -= pitchIncrement; }

	void rollCCW()		{ trimRoll -= rollIncrement; }
	void rollCW()		{ trimRoll += rollIncrement; }

	void yawLeft()		{ trimYaw += yawIncrement; }
	void yawRight()		{ trimYaw -= yawIncrement; }
};

// just keep some things together in easily accessible way
class F16BodyState
{
public:
	double	alpha_DEG;			// Angle of attack (deg)
	double	beta_DEG;			// Slideslip angle (deg)

	double	rollRate_RPS;		// Body roll rate (rad/sec)
	double	pitchRate_RPS;		// Body pitch rate (rad/sec)
	double	yawRate_RPS;		// Body yaw rate (rad/sec)

	double	ay_world;		// World referenced up/down acceleration (m/s^2)
	double	accz;			// Az (per normal direction convention) out the bottom of the a/c (m/s^2)
	double	accy;			// Ay (per normal direction convention) out the right wing (m/s^2)

public:
	F16BodyState() 
		: alpha_DEG(0), beta_DEG(0)
		, rollRate_RPS(0), pitchRate_RPS(0), yawRate_RPS(0)
		, ay_world(0), accz(0), accy(0)
	{}
	~F16BodyState() {}

	double getRollRateDegs() const
	{
		return rollRate_RPS * F16::radiansToDegrees;
	}
	double getPitchRateDegs() const
	{
		return pitchRate_RPS * F16::radiansToDegrees;
	}
	double getYawRateDegs() const
	{
		return yawRate_RPS * F16::radiansToDegrees;
	}

	double getAccZPerG() const
	{
		return accz / F16::standard_gravity;
	}
	double getAccYPerG() const
	{
		return accy / F16::standard_gravity;
	}
};

// keep flight surface positions in one place
class F16FlightSurface
{
public:
	double		leadingEdgeFlap_Command;		// command before actuator movement
	double		leadingEdgeFlap_DEG;			// Leading edge flap deflection (deg)
	double		leadingEdgeFlap_Right_PCT;			// Leading edge flap as a percent of maximum (0 to 1)
	double		leadingEdgeFlap_Left_PCT;			// Leading edge flap as a percent of maximum (0 to 1)

	double		flap_Command;		// command from tef controller (position), can be adjusted by roll controller?

	// result of command mixer (flaps and roll commands)
	double		flaperon_Right_Command;
	double		flaperon_Left_Command;
	double		flaperon_Right_DEG;			// flaperon deflection (deg)
	double		flaperon_Left_DEG;			// flaperon deflection (deg)
	double		flaperon_Right_PCT;			// flaperon deflection as a percent of maximum (-1 to 1)
	double		flaperon_Left_PCT;			// flaperon deflection as a percent of maximum (-1 to 1)

	double flap_Left_DEG;
	double flap_Right_DEG;
	double flap_Left_PCT;
	double flap_Right_PCT;

	double		elevon_Left_Command;
	double		elevon_Right_Command;
	//double		elevon_Right_DEG;			// elevon deflection (deg): elevator assist to aileron
	//double		elevon_Left_DEG;			// elevon deflection (deg): elevator assist to aileron

	double		elevator_Right_DEG;			// Elevator deflection (deg) (pitch)
	double		elevator_Left_DEG;			// Elevator deflection (deg) (pitch)
	double		elevator_Right_PCT;			// Elevator deflection as a percent of maximum (-1 to 1)
	double		elevator_Left_PCT;			// Elevator deflection as a percent of maximum (-1 to 1)

	double		rudder_Command;
	double		rudder_DEG;			// Rudder  deflection (deg)
	double		rudder_PCT;			// Rudder deflection as a percent of maximum (-1 to 1)

	double		airbrake_Command;
	double		airbrake_Right_DEG;
	double		airbrake_Left_DEG;
	double		airbrake_Right_PCT;
	double		airbrake_Left_PCT;

	// new set of commands: split from old code,
	// then code in mixer to determine combinations of surfaces.
	// These are what each controller wants, actual deflection (according to actuator) comes after.
	double		pitch_Command;
	double		roll_Command;
	double		yaw_Command;

public:
	F16FlightSurface() :
		leadingEdgeFlap_Command(0),
		leadingEdgeFlap_DEG(0),
		leadingEdgeFlap_Right_PCT(0),
		leadingEdgeFlap_Left_PCT(0),
		flap_Command(0),
		flaperon_Right_Command(0),
		flaperon_Left_Command(0),
		flaperon_Right_DEG(0),
		flaperon_Left_DEG(0),
		flaperon_Right_PCT(0),
		flaperon_Left_PCT(0),
		flap_Left_DEG(0),
		flap_Right_DEG(0),
		flap_Left_PCT(0),
		flap_Right_PCT(0),
		elevon_Left_Command(0),
		elevon_Right_Command(0),
		//elevon_Right_DEG(0),
		//elevon_Left_DEG(0),
		elevator_Right_DEG(0),
		elevator_Left_DEG(0),
		elevator_Right_PCT(0),
		elevator_Left_PCT(0),
		rudder_Command(0),
		rudder_DEG(0),
		rudder_PCT(0),
		airbrake_Command(0),
		airbrake_Right_DEG(0),
		airbrake_Left_DEG(0),
		airbrake_Right_PCT(0),
		airbrake_Left_PCT(0),
		pitch_Command(0),
		roll_Command(0),
		yaw_Command(0)
	{}
	~F16FlightSurface() {}
};


#endif // ifndef _F16FCSCOMMON_H_
