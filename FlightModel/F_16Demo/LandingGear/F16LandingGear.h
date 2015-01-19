#ifndef _F16LANDINGGEAR_H_
#define _F16LANDINGGEAR_H_

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

	class F16WheelFriction
	{
	public:
		const double rolling_friction;			// Rolling friction amount (constant now)

		double strutCompression;

		double CxWheelFriction;
		double CyWheelFriction;

		double brakeForce;

		F16WheelFriction() 
			: rolling_friction(0.03)
			, strutCompression(0)
			, CxWheelFriction(0)
			, CyWheelFriction(0)
			, brakeForce(0)
		{}
		~F16WheelFriction() {}

		// TODO: amount of weight per wheel instead?
		void wheelFriction(const double weightN, bool weight_on_wheel)
		{
			// TODO: if wheel brakes ON -> braking power multiplier

			if (weight_on_wheel)
			{
				//CxWheelFriction = -rolling_friction * weightN;
				CxWheelFriction = rolling_friction * weightN;
				CyWheelFriction = 0.18 * weightN;
			}
			else
			{
				CxWheelFriction = 0.0;
				CyWheelFriction = 0.0;
			}
		}

		void wheelBrake(const double weightN, bool weight_on_wheel)
		{
			if (weight_on_wheel == false)
			{
				return;
			}
			if (brakeForce <= 0)
			{
				return;
			}

			// TODO: find out some reasonable values,
			// do we need to have brake fading support as well?
			// TODO: also switch calculation to reduction in kinectic energy in motion handling

			double brakeFriction = brakeForce * 10; // guess

			// just add it to rolling friction
			CxWheelFriction += (rolling_friction * brakeFriction * weightN);
			//CyWheelFriction = 0.18 * weightN;

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

		void setStrutRetracted()
		{
			strutCompression = 1;
		}
		void setStrutParking()
		{
			strutCompression = 0.5;
		}
		void setStrutExtended()
		{
			strutCompression = 0;
		}
	};

	class F16LandingGear
	{
	protected:
		// precalculate some things
		const double gearZsin = sin(F16::degtorad);
		const double gearYcos = cos(F16::degtorad);

	public:
		// 
		double	weight_on_wheels;			// Weight on wheels flag (not used right now)

		double	gearDownAngle;	// Is the gear currently down? (If not, what angle is it?)

		bool nosewheelSteering; // is active/not
		double noseGearTurnAngle; // steering angle {-1=CW max;1=CCW max}

		double CDGearAero;
		double CzGearAero;
		double CxGearAero;

		F16WheelFriction wheelNose;
		F16WheelFriction wheelLeft;
		F16WheelFriction wheelRight;

		bool parkingBreakOn;

		F16LandingGear() 
			: weight_on_wheels(0)
			, gearDownAngle(0)
			, nosewheelSteering(false)
			, noseGearTurnAngle(0)
			, CDGearAero(0)
			, CzGearAero(0)
			, CxGearAero(0)
			, wheelNose()
			, wheelLeft()
			, wheelRight()
			, parkingBreakOn(false)
		{
		}
		~F16LandingGear() {}

		bool isWoW()
		{
			return (weight_on_wheels > 0);
		}

		void setParkingBreak(bool OnOff)
		{
			parkingBreakOn = OnOff;
		}

		// joystick axis
		void setWheelBrakeLeft(double value)
		{
			// 0..1 from input
			wheelLeft.brakeForce = value;
		}
		// joystick axis
		void setWheelBrakeRight(double value)
		{
			// 0..1 from input
			wheelRight.brakeForce = value;
		}

		// key press DOWN
		void setWheelBrakesON()
		{
			wheelLeft.brakeForce = 1;
			wheelRight.brakeForce = 1;
		}
		// key press UP
		void setWheelBrakesOFF()
		{
			wheelLeft.brakeForce = 0;
			wheelRight.brakeForce = 0;
		}

		void setNosewheelSteeringON()
		{
			nosewheelSteering = true;
		}
		void setNosewheelSteeringOFF()
		{
			nosewheelSteering = false;
		}

		void nosewheelTurn(double value)
		{
			if (nosewheelSteering == false)
			{
				return;
			}
			if (isWoW() == false)
			{
				return;
			}
			// TODO: check value
			noseGearTurnAngle = value;
		}
		double getNosegearTurn()
		{
			/*
			if (!weight_on_wheels)
			{
				return 0;
			}
			*/
			return noseGearTurnAngle;
		}

		double getNoseGearDown()
		{
			return limit(gearDownAngle, 0, 1);
		}
		double getLeftGearDown()
		{
			return limit(gearDownAngle, 0, 1);
		}
		double getRightGearDown()
		{
			return limit(gearDownAngle, 0, 1);
		}

		void switchGearUpDown()
		{
			if (gearDownAngle > 0)
			{
				// down -> up
				setGearUp();
			}
			else
			{
				// up -> down
				setGearDown();
			}
		}
		void setGearDown()
		{
			gearDownAngle = 1.0;

			// !! disabled for now
			//weight_on_wheels = 1;
		}
		void setGearUp()
		{
			gearDownAngle = 0;

			// !! disabled for now
			//weight_on_wheels = 0;
		}

		void updateFrame(const double weightN, double frameTime);

	protected:

		void gearAeroDrag(/*double airspeed?*/)
		{
			// TODO Gear aero (from JBSim F16.xml config)
			CDGearAero = 0.0270 * gearDownAngle; 
			CzGearAero = - (CDGearAero * gearZsin);
			CxGearAero = - (CDGearAero * gearYcos);
		}

		void wheelFriction(const double weightN)
		{
			// TODO: if wheel brakes ON -> braking power multiplier

			bool WoW = isWoW();

			wheelNose.wheelFriction(weightN, WoW);
			wheelLeft.wheelFriction(weightN, WoW);
			wheelRight.wheelFriction(weightN, WoW);
		}

		void wheelBrakeEffect(const double weightN)
		{
			// TODO: also anti-skid system, wheel locking without it etc.

			bool WoW = isWoW();

			wheelNose.wheelBrake(weightN, WoW);
			wheelLeft.wheelBrake(weightN, WoW);
			wheelRight.wheelBrake(weightN, WoW);
		}

		void brakeFluidUsage(double frameTime)
		{
			// TODO: there's fluid for approx. 75s (toe brakes)
			// parking brake usage does not deplete fluid
		}

		void updateStrutCompression()
		{
			// TODO: actual calculations of weight on each wheel,
			// especially force when landing unevenly etc.

			if (gearDownAngle == 0)
			{
				// retracted
				wheelNose.setStrutRetracted();
				wheelLeft.setStrutRetracted();
				wheelRight.setStrutRetracted();
			}
			else if (weight_on_wheels)
			{
				// parking
				wheelNose.setStrutParking();
				wheelLeft.setStrutParking();
				wheelRight.setStrutParking();
			}
			else
			{
				// extended
				wheelNose.setStrutExtended();
				wheelLeft.setStrutExtended();
				wheelRight.setStrutExtended();
			}
		}
	};

	void F16LandingGear::updateFrame(const double weightN, double frameTime)
	{
		gearAeroDrag();
		wheelFriction(weightN);
		wheelBrakeEffect(weightN);
		updateStrutCompression();
	}
}

#endif // ifndef _F16LANDINGGEAR_H_
