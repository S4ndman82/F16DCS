#ifndef _F16LANDINGGEAR_H_
#define _F16LANDINGGEAR_H_

#include "../stdafx.h"

namespace F16
{
	// gears up/down status
	// weight-on-wheels
	// actuator movement
	// aerodynamic drag
	// wheelbrake deceleration

	class F16LandingGear
	{
	protected:
		// precalculate some things
		const double gearZsin = sin(M_PI/180.0);
		const double gearYcos = cos(M_PI/180.0);

	public:
		// 
		double		weight_on_wheels;			// Weight on wheels flag (not used right now)
		double		rolling_friction;			// Rolling friction amount (not use right now)

		double		gearDownAngle;	// Is the gear currently down? (If not, what angle is it?)

		double noseGearAngle; // steering angle {-1=CW max;1=CCW max}

		double strutCompressionNose;
		double strutCompressionLeft;
		double strutCompressionRight;

		double CDGearAero;
		double CzGearAero;
		double CxGearAero;

		double CxWheelFriction;
		double CyWheelFriction;

		// differential braking support?
		// analog and button/switch support?
		double brakeForceLeft;
		double brakeForceRight;

		F16LandingGear() 
			: weight_on_wheels(0)
			, rolling_friction(0.03)
			, gearDownAngle(0)
			, noseGearAngle(0)
			, strutCompressionNose(0)
			, strutCompressionLeft(0)
			, strutCompressionRight(0)
			, CDGearAero(0)
			, CzGearAero(0)
			, CxGearAero(0)
			, CxWheelFriction(0)
			, CyWheelFriction(0)
			, brakeForceLeft(0)
			, brakeForceRight(0)
		{
		}
		~F16LandingGear() {}

		void updateFrame(const double weightN, const double rudderPCT, double frameTime);

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

			if (weight_on_wheels)
			{
				CxWheelFriction = -rolling_friction * weightN;
				CyWheelFriction = 0.18 * weightN;
			}
			else
			{
				CxWheelFriction = 0.0;
				CyWheelFriction = 0.0;
			}
		}

		void updateStrutCompression()
		{
			if (gearDownAngle == 0)
			{
				// retracted
				strutCompressionNose = 1;
				strutCompressionLeft = 1;
				strutCompressionRight = 1;
			}
			else if (weight_on_wheels)
			{
				// parking
				strutCompressionNose = 0.5;
				strutCompressionLeft = 0.5;
				strutCompressionRight = 0.5;
			}
			else
			{
				// extended
				strutCompressionNose = 0;
				strutCompressionLeft = 0;
				strutCompressionRight = 0;
			}
		}
	};

	void F16LandingGear::updateFrame(const double weightN, const double rudderPCT, double frameTime)
	{
		noseGearAngle = rudderPCT; // nose gear angle {-1=CW max;1=CCW max}

		gearAeroDrag();
		wheelFriction(weightN);
		updateStrutCompression();
	}
}

#endif // ifndef _F16LANDINGGEAR_H_
