#ifndef _F16LANDINGGEAR_H_
#define _F16LANDINGGEAR_H_

#include "../stdafx.h"

#include "F16LandingWheel.h"

namespace F16
{
	// nosewheel steering (NWS) limited to 32 degrees in each direction

	// gears up/down status
	// weight-on-wheels
	// actuator movement
	// aerodynamic drag
	// wheelbrake deceleration
	// parking brake

	// anti-skid system

	// TODO: different friction coefficient on each surface?
	// (tarmac, concrete, grass, mud...)

	// amount of braking fluid (liquid) in system? (limited amount available)

	class F16LandingGear //: public AbstractHydraulicDevice
	{
	protected:
		// precalculate some things
		const double gearZsin = sin(F16::degtorad);
		const double gearYcos = cos(F16::degtorad);

		bool gearLevelUp; // gear lever up/down (note runway/air start)
		double gearDownAngle;	// Is the gear currently down? (If not, what angle is it?)
		//bool gearDownLocked; // gear down and locked?

	public:

		bool nosewheelSteering; // is active/not
		double noseGearTurnAngle; // steering angle {-1=CW max;1=CCW max}

		// aerodynamic drag when gears are not fully up
		double CDGearAero;
		double CzGearAero;
		double CxGearAero;

		F16LandingWheel wheelNose;
		F16LandingWheel wheelLeft;
		F16LandingWheel wheelRight;

		bool parkingBreakOn;
		bool antiSkidOn;

		//F16HydraulicSystem *pHydSys; // parent system providing force

		F16LandingGear() 
			: gearLevelUp(false)
			, gearDownAngle(0)
			, nosewheelSteering(true) // <- enable by default until button mapping works
			, noseGearTurnAngle(0)
			, CDGearAero(0)
			, CzGearAero(0)
			, CxGearAero(0)
			, wheelNose(0.479, 3.6) // <- inertia should be different for nose wheel? (smaller wheel)
			, wheelLeft(0.68, 3.6)
			, wheelRight(0.68, 3.6)
			, parkingBreakOn(false)
			, antiSkidOn(false)
		{
		}
		~F16LandingGear() {}

		bool isWoW()
		{
			if (wheelNose.isWoW()
				|| wheelLeft.isWoW()
				|| wheelRight.isWoW())
			{
				return true;
			}
			return false;
		}
		int countWoW()
		{
			int n = 0;
			if (wheelNose.isWoW())
				n++;
			if (wheelLeft.isWoW())
				n++;
			if (wheelRight.isWoW())
				n++;
			return n;
		}

		void setParkingBreak(bool OnOff)
		{
			parkingBreakOn = OnOff;
		}

		// joystick axis
		void setWheelBrakeLeft(const double value)
		{
			// 0..1 from input
			wheelLeft.brakeInput = abs(value);
		}
		// joystick axis
		void setWheelBrakeRight(const double value)
		{
			// 0..1 from input
			wheelRight.brakeInput = abs(value);
		}

		// key press DOWN
		void setWheelBrakesON()
		{
			wheelLeft.brakeInput = 1;
			wheelRight.brakeInput = 1;
		}
		// key press UP
		void setWheelBrakesOFF()
		{
			wheelLeft.brakeInput = 0;
			wheelRight.brakeInput = 0;
		}

		void toggleNosewheelSteering()
		{
			nosewheelSteering = !nosewheelSteering;
		}

		// TODO: joystick input is usually -100..100
		// and we have limit of 32 degrees each way
		// -> calculate correct nosewheel angle by input amount
		// or just "cut" out excess input?
		void nosewheelTurn(const double value)
		{
			if (isWoW() == false)
			{
				return;
			}
			if (nosewheelSteering == false)
			{
				return;
			}

			// TODO: check value
			//noseGearTurnAngle = value;

			// for now, just cut input to allowed range in degrees
			noseGearTurnAngle = limit(value, -32, 32);
		}

		// in case of nose wheel: 
		// calculate direction vector based on the turn angle
		// (polar to cartesian coordinates)
		Vec3 getNoseTurnDirection()
		{
			// in DCS body coordinates, (x,z) plane vector instead of (x,y) ?
			double x = cos(noseGearTurnAngle);
			double z = cos(noseGearTurnAngle);

			return Vec3(x, 0, z); // test!

			//Vec3 direction(1, 0, 0); // <- start with aligned to body direction
			// ..vector multiply by radians?
			// translation matrix?
			//return direction;
		}

		// nosegear turn -1..1 for drawargs
		float getNosegearTurn()
		{
			/*
			if (isWoW() == false)
			{
				return 0;
			}
			*/
			return (float)limit(noseGearTurnAngle, -1, 1);
		}

		// actual nosegear turn angle
		double getNosegearAngle()
		{
			/*
			if (isWoW() == false)
			{
				return 0;
			}
			*/
			return noseGearTurnAngle;
		}

		void initGearsDown()
		{
			gearLevelUp = false;
			gearDownAngle = 1.0;
			wheelNose.setStrutAngle(gearDownAngle);
			wheelLeft.setStrutAngle(gearDownAngle);
			wheelRight.setStrutAngle(gearDownAngle);
		}
		void initGearsUp()
		{
			gearLevelUp = true;
			gearDownAngle = 0;
			wheelNose.setStrutAngle(gearDownAngle);
			wheelLeft.setStrutAngle(gearDownAngle);
			wheelRight.setStrutAngle(gearDownAngle);
		}

		// user "lever" action
		void switchGearUpDown()
		{
			gearLevelUp = !gearLevelUp;
		}
		void setGearDown()
		{
			gearLevelUp = false;
		}
		void setGearUp()
		{
			gearLevelUp = true;
		}

		// is gear "down and locked" -> affects airbrake logic
		bool isGearDownLocked() const
		{
			if (gearDownAngle < 1.0)
			{
				// not completely down
				return false;
			}
			// at max -> down & locked
			return true;
		}

		// need current weight of the whole aircraft
		// and speed relative to ground (static, sliding or rolling friction of each wheel)
		void updateFrame(const double groundSpeed, const double weightN, double frameTime)
		{
			wheelNose.clearForceTotal();
			wheelLeft.clearForceTotal();
			wheelRight.clearForceTotal();

			// TODO: angle for each wheel individually

			// if there is weight on wheels -> do nothing
			if (isWoW() == false)
			{
				//if status != gearLevelUp
				// -> actuator movement up/down by frame step
				if (gearLevelUp == true && gearDownAngle > 0)
				{
					// reduce angle by actuator movement
					// -> simple hack for now
					gearDownAngle -= (frameTime / 10);
				}
				else if (gearLevelUp == false && gearDownAngle < 1.0)
				{
					// increase angle by actuator movement
					// -> simple hack for now
					gearDownAngle += (frameTime / 10);
				}

				// check we don't go over limit
				gearDownAngle = limit(gearDownAngle, 0, 1.0);

				wheelNose.setStrutAngle(gearDownAngle);
				wheelLeft.setStrutAngle(gearDownAngle);
				wheelRight.setStrutAngle(gearDownAngle);
			}

			gearAeroDrag();

			// let's see.. 
			// total static friction should be amount of friction required (by engine thrust)
			// to be exceeded before we start rolling
			if (isWoW() == true) // <- at least one wheel has weight on it
			{
				double weightperwheel = weightN / countWoW();

				wheelNose.updateForceFriction(groundSpeed, weightperwheel);
				wheelLeft.updateForceFriction(groundSpeed, weightperwheel);
				wheelRight.updateForceFriction(groundSpeed, weightperwheel);
			}
		}


	protected:

		void gearAeroDrag(/*double airspeed?*/)
		{
			// TODO Gear aero (from JBSim F16.xml config)
			CDGearAero = 0.0270 * gearDownAngle; 
			CzGearAero = - (CDGearAero * gearZsin);
			CxGearAero = - (CDGearAero * gearYcos);
		}

		void brakeFluidUsage(double frameTime)
		{
			// TODO: there's fluid for approx. 75s (toe brakes)
			// parking brake usage does not deplete fluid
		}
	};
}

#endif // ifndef _F16LANDINGGEAR_H_
