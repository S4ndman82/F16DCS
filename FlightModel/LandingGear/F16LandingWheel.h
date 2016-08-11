#ifndef _F16LANDINGWHEEL_H_
#define _F16LANDINGWHEEL_H_

#include "../stdafx.h"

namespace F16
{
	// nosewheel steering (NWS) limited to 32 degrees in each direction

	// gears up/down status
	// weight-on-wheels
	// actuator movement
	// aerodynamic drag
	// wheelbrake deceleration
	// parking brake

	// TODO: different friction coefficient on each surface?
	// (tarmac, concrete, grass, mud...)


	class F16LandingWheel
	{
	protected:
		//const double rolling_friction;		// Rolling friction amount (constant now)
		//double slip_friction;		// TODO: Slip friction amount? (depends on surface as well..)
		//double forcefriction // <- direction and combined forces to calculate if maximum friction is exceeded?

		const double wheel_radius; //					  = 0.479,
		const double wheel_static_friction_factor; //  = 0.65 , --Static friction when wheel is not moving (fully braked)
		const double wheel_side_friction_factor; //    = 0.65 ,
		const double wheel_roll_friction_factor; //    = 0.025, --Rolling friction factor when wheel moving
		const double wheel_glide_friction_factor; //   = 0.28 , --Sliding aircraft
		const double wheel_damage_force_factor; //     = 250.0, -- Tire is explosing due to hard landing
		const double wheel_damage_speed; //			   = 150.0, -- Tire burst due to excessive speed
		const double wheel_moment_of_inertia; //		= 3.6, --wheel moi as rotation body
		const double wheel_brake_moment_max; //		= 15000.0, -- maximum value of braking moment  , N*m 

	public:
		double strutCompression;
		double wheelStrutDownAngle; // angle of strut (suspension, up/down)

		// current frictions applied on wheel
		double CxWheelFriction;
		double CzWheelFriction; // side-ways friction (should be Zbody axis)

		double brakeInput; // braking command/input from user
		double brakeForce; // result force

		// from DCS, see ed_fm_suspension_feedback()
		Vec3 actingForce;
		Vec3 actingForcePoint;
		double integrityFactor;

		F16LandingWheel(const double wheelRadius, const double wheelInertia)
			: wheel_radius(wheelRadius) // all other same on each wheel? (check)
			, wheel_static_friction_factor(0.65)
			, wheel_side_friction_factor(0.65)
			, wheel_roll_friction_factor(0.025)
			, wheel_glide_friction_factor(0.28)
			, wheel_damage_force_factor(250.0)
			, wheel_damage_speed(150.0)
			, wheel_moment_of_inertia(wheelInertia) // <- should be different for nose wheel? (smaller wheel)
			, wheel_brake_moment_max(15000.0)
			, strutCompression(0)
			, wheelStrutDownAngle(0)
			, CxWheelFriction(0)
			, CzWheelFriction(0)
			, brakeInput(0)
			, brakeForce(0)
			, actingForce()
			, actingForcePoint()
			, integrityFactor(0)
		{}
		~F16LandingWheel() {}

		bool isWoW() const
		{
			if (strutCompression > 0)
			{
				return true;
			}
			return false;
		}

		void setActingForce(double x, double y, double z)
		{
			actingForce.x = x;
			actingForce.y = y;
			actingForce.z = z;
		}
		void setActingForcePoint(double x, double y, double z)
		{
			actingForcePoint.x = x;
			actingForcePoint.y = y;
			actingForcePoint.z = z;
		}
		void setIntegrityFactor(double d)
		{
			integrityFactor = d;
		}

		double getStrutCompression() const
		{
			return strutCompression;
		}

		// we might get this directly at initialization so set here
		void setStrutCompression(const double compression)
		{
			strutCompression = compression;
		}

		double getStrutAngle() const
		{
			return wheelStrutDownAngle;
		}

		// expecting to be within limits,
		// change if something else is needed
		void setStrutAngle(const double angle)
		{
			wheelStrutDownAngle = angle;
		}

		/*
		double getUpLock() const
		{}
		double getDownLock() const
		{}
		*/

		// calculate new direction of force and if it exceeds friction (begins sliding)
		// TODO: need ground speed here for rolling/static friction
		// also, depending on how many wheels the weight is distributed on
		void updateForceFriction(const double groundSpeed, const double weightN)
		{
			if (isWoW() == false) // strut is not compressed -> no weight on wheel
			{
				CxWheelFriction = 0;
				CzWheelFriction = 0;
				brakeForce = 0;
				return;
			}

			// TODO: also if wheel rotation is slower than speed relative to ground
			// -> apply sliding friction factor
			//if (braking && wheel locked (anti-skid==false) -> glide-factor?)

			if (groundSpeed > 0 && brakeInput > 0)
			{
				// just percentage of max according to input 0..1, right?
				brakeForce = abs(brakeInput) * wheel_brake_moment_max;

				// if anti-skid is enabled -> check for locking
				// rotation speed of the wheel related to linear ground speed
				//weightN * wheel_glide_friction_factor if locking?
			}

			// note: DCS has "left-hand notation" so side-slip is Z-axis?
			CzWheelFriction = wheel_side_friction_factor * weightN; // <- side-slip factor?

			if (groundSpeed > 0)
			{
				// just rolling friction?
				CxWheelFriction = (-wheel_roll_friction_factor * weightN);
			}
			else
			{
				// "static" friction needed to overcome to start moving?
				CxWheelFriction = (-wheel_static_friction_factor * weightN);
			}

		}
	};

}

#endif // ifndef _F16LANDINGWHEEL_H_
