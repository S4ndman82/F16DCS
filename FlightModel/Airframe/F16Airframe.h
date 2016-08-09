#ifndef _F16AIRFRAME_H_
#define _F16AIRFRAME_H_

#include "../stdafx.h"

namespace F16
{
	// - canopy status
	// - refueling slot status
	// - dragging chute status
	// - tail hook support?
	// - nav lights
	// - formation lights
	// - landing lights?
	// - damage in sections? 
	// - accumulated over-g stress?

	class F16Airframe
	{
	protected:
		double canopyAngle;		// Canopy status/angle {0=closed;0.9=elevated;1=no draw}
		bool canopySwitchDown; // up/down

		// TODO: cockpit pressure in pascals over external (get update from oxygen system also)
		double cockpitPressure;

		// TODO: support for each lamp in lights?
		//bool navigationLight[10];
		//bool formationLight[8];
		//bool landingLamp[5];

		// current 3D model has three lights:
		// left, right and back (tail)
		// 0=off, 1=on
		float leftWingLamp;
		float rightWingLamp;
		float backTailLamp;

		bool navigationLights;
		bool formationLights;
		bool landingLights;
		bool strobeLights;

		// TODO:
		// damage status of section in 0.01 increments
		double elementIntegrity[336]; // TODO: check what size we would need here
		bool isImmortal; // <- ignore damage

	public:
		F16Airframe()
			: canopyAngle(0)
			, canopySwitchDown(false)
			, cockpitPressure(0)
			, leftWingLamp(0)
			, rightWingLamp(0)
			, backTailLamp(0)
			, navigationLights(false)
			, formationLights(false)
			, landingLights(false)
			, strobeLights(false)
			, isImmortal(false)
		{
			// TODO: check values, size (how many we need)
			// is zero "no fault" or "fully broken"? 
			::memset(elementIntegrity, 0, 336 * sizeof(double));
		}
		~F16Airframe() {}

		void initCanopyOpen()
		{
			canopySwitchDown = false;
			canopyAngle = 0.9; // up
		}
		void initCanopyClosed()
		{
			canopySwitchDown = true;
			canopyAngle = 0; // down
		}

		// canopy open/close toggle
		void canopyToggle()
		{
			canopySwitchDown = !canopySwitchDown;
		}
		void setCanopySwitchUp()
		{
			canopySwitchDown = false;
		}
		void setCanopySwitchDown()
		{
			canopySwitchDown = true;
		}

		void canopyJettison() // <- no binding yet
		{
			canopyAngle = 1.0; // gone
		}

		// draw angle of canopy {0=closed;0.9=elevated;1=no draw}
		float getCanopyAngle() const
		{
			return (float)canopyAngle;
		}

		float getRefuelingDoorAngle() const
		{
			// not yet implemented
			return 0;
		}

		// TODO:
		// get cockpit pressure in pascals over external (get update from oxygen system also)
		double getCockpitPressure() const
		{
			return cockpitPressure;
		}

		float isNavigationLight() const
		{
			return (navigationLights == true) ? 1.0f : 0.0f;
		}
		float isFormationLight() const
		{
			return (formationLights == true) ? 1.0f : 0.0f;
		}
		float isLandingLight() const
		{
			return (landingLights == true) ? 1.0f : 0.0f;
		}
		float isStrobeLight() const
		{
			return (strobeLights == true) ? 1.0f : 0.0f;
		}

		// current 3D model has three lights:
		// left, right and back (tail)
		float getLeftLight() const
		{
			return leftWingLamp;
		}
		float getRightLight() const
		{
			return rightWingLamp;
		}
		float getBackLight() const
		{
			return backTailLamp;
		}

		/*
		bool draggingChuteCap() const
		{}
		*/

		void onAirframeDamage(int Element, double element_integrity_factor)
		{
			// TODO: check what kind of amount of elements we would need here
			if (Element >= 0 && Element < 336)
			{
				elementIntegrity[Element] = element_integrity_factor;
			}
		}

		void onRepair()
		{
			// TODO: check values, size (how many we need)
			// is zero "no fault" or "fully broken"? 
			::memset(elementIntegrity, 0, 336*sizeof(double));
		}

		bool isRepairNeeded() const
		{
			// TODO: check values, size (how many we need)
			// is zero "no fault" or "fully broken"? 
			for (int i = 0; i < 336; i++)
			{
				if (elementIntegrity[i] > 0)
				{
					return true;
				}
			}
			return false;
		}

		void setImmortal(bool value)
		{
			isImmortal = value;
		}

		// accumulate stress?
		// update pressure?
		void updateFrame(const double frameTime)
		{
			if (canopySwitchDown == false && canopyAngle < 0.9)
			{
				// move up -> increase angle
				canopyAngle += (frameTime / 10);
				if (canopyAngle > 0.9)
				{
					canopyAngle = 0.9; // check we don't exceed limit
				}
			}
			else if (canopySwitchDown == true && canopyAngle > 0)
			{
				// move down -> decrease angle
				canopyAngle -= (frameTime / 10);
				if (canopyAngle < 0)
				{
					canopyAngle = 0; // check we don't exceed limit
				}
			}
			// TODO: if in flight and canopy open -> canopy gone
			// also, if sealing is not working and internal pressure exceeds external -> canopy gone

			// TODO: some light switching logic on/off?

			// TODO: update cockpit pressure from oxygen system?

		}
	};
}

#endif // ifndef _F16AIRFRAME_H_

