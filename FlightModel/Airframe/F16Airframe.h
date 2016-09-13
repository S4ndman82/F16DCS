#ifndef _F16AIRFRAME_H_
#define _F16AIRFRAME_H_

#include "ED_FM_Utility.h"		// Provided utility functions that were in the initial EFM example
#include "F16Constants.h"		// Common constants used throughout this DLL

// for actuator code
#include "FlightControls/F16Actuator.h"

#include "UtilityFunctions.h"

// - canopy status
// - refueling slot status
// - dragging chute status
// - tail hook support?
// - nav lights
// - formation lights
// - landing lights?
// - damage in sections? 
// - accumulated over-g stress?


// simple helper for different lights and sequences of blinking:
// set length of sequence (amount of states), time for each state (in seconds)
// and set sequence that is desired, for example: 0, 1, 0, 0, 1, 1 (start to end)
// (0=off, 1=on)
class F16LightBlinker
{
private:
	// disallow direct copying
	F16LightBlinker(const F16LightBlinker &other);
	F16LightBlinker& operator=(const F16LightBlinker &other);

protected:
	int index = 0;
	double elapsed = 0;

public:
	int sequenceCount = 0;
	float *pSequence; // ex. { 0.0f, 1.0f, 0.0f, 0.0f, 1.0f };
	double blinkRate = 1.0f; // in seconds
	bool isEnabled = false; // if blinker is active

	F16LightBlinker(const int seqCount, const double blinker) 
		: sequenceCount(seqCount)
		, pSequence(nullptr)
		, blinkRate(blinker)
	{
		pSequence = new float[seqCount];
		::memset(pSequence, 0, sizeof(float)*seqCount);
	}
	~F16LightBlinker() 
	{
		if (pSequence != nullptr)
		{
			delete pSequence;
			pSequence = nullptr;
		}
	}

	float getCurrent() const
	{
		return pSequence[index];
	}

	void setPattern(const int newCount, const float *newSequence)
	{
		if (newCount != sequenceCount && pSequence != nullptr)
		{
			delete pSequence;
			pSequence = new float[newCount];
			sequenceCount = newCount;
		}
		::memcpy(pSequence, newSequence, sizeof(float) * newCount);
	}

	void updateFrame(const double frameTime)
	{
		if (isEnabled == false)
		{
			return;
		}

		elapsed += frameTime;
		if (elapsed < blinkRate)
		{
			return;
		}
		index++;
		if (index >= sequenceCount)
		{
			// back to start
			index = 0;
		}
		// until next step
		elapsed = 0;
	}
};

class F16Airframe
{
protected:
	//Canopy status/angle {0=closed;0.9=elevated;1=no draw}
	F16Actuator actCanopy;
	F16Actuator actRefuelingDoor; // trapdoor in the back

	bool canopySwitchDown; // up/down
	bool canopyGone; // simplify some code

	bool refuelingDoorOpen; // open/close switch

	bool ejectingSeat; // ejecting seat in place/gone
	int ejectCount; // times pressed eject

	// TODO: support for each lamp in lights?
	//bool navigationLight[10];
	//bool formationLight[8];
	//bool landingLamp[5];

	// current 3D model has three lights:
	// left, right and back (tail)
	// 0=off, 1=on
	// likely would need more of these but 3D model has some things linked for now
	F16LightBlinker leftBlinker;
	F16LightBlinker rightBlinker;
	F16LightBlinker backBlinker;

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
		: actCanopy(0.5, 0, 0.9)
		, actRefuelingDoor(0.5, 0, 1)
		, canopySwitchDown(false)
		, canopyGone(false)
		, refuelingDoorOpen(false)
		, ejectingSeat(true)
		, ejectCount(0)
		, leftBlinker(5, 1.5)
		, rightBlinker(5, 1.5)
		, backBlinker(5, 1.5)
		, navigationLights(false)
		, formationLights(false)
		, landingLights(false)
		, strobeLights(false)
		, isImmortal(false)
	{
		// TODO: check values, size (how many we need)
		// is zero "no fault" or "fully broken"? 
		::memset(elementIntegrity, 0, 336*sizeof(double));
	}
	~F16Airframe() {}

	void initCanopyOpen()
	{
		canopySwitchDown = false;
		//canopyAngle = 0.9; // up
		actCanopy.m_commanded = actCanopy.m_limiter.upper_limit;
		actCanopy.m_current = actCanopy.m_commanded;
		canopyGone = false;
		ejectCount = 0;
	}
	void initCanopyClosed()
	{
		canopySwitchDown = true;
		//canopyAngle = 0; // down
		actCanopy.m_commanded = actCanopy.m_limiter.lower_limit;
		actCanopy.m_current = actCanopy.m_commanded;
		canopyGone = false;
		ejectCount = 0;
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
		canopyGone = true;
	}

	// damage to canopy (in case open in flight, failure on sealing..)
	void canopyDamage(int integrityfactor)
	{
		// just set it gone for now
		// -> pressure to ambient pressure
		canopyGone = true;
	}

	// draw angle of canopy {0=closed;0.9=elevated;1=no draw}
	float getCanopyAngle() const
	{
		if (canopyGone == true)
		{
			return 1.0; // gone
		}
		return (float)actCanopy.m_current;
	}

	// update aero drag from canopy when gone
	// (on taxiing does not matter if halfway?)
	void updateCanopyDrag(const double frameTime)
	{
	}

	void toggleRefuelingDoor()
	{
		refuelingDoorOpen = !refuelingDoorOpen;
	}

	// animation support for refueling door at the back
	float getRefuelingDoorAngle() const
	{
		// not yet implemented in 3D mesh
		return (float)actRefuelingDoor.m_current;
	}

	float getEjectingSeatDraw() const
	{
		if (ejectingSeat == true)
		{
			// pilot and seat in plane
			return 0;
		}
		// could have 0.5 == pilot gone, seat in place?
		// 1 == seat gone
		return 1;
	}

	void setNavigationLights(bool onoff)
	{
		if (onoff == true && navigationLights == false)
		{
			// just some example pattern for testing
			float blinkerSequence[5] = { 0.0f, 1.0f, 0.0f, 0.0f, 1.0f };

			leftBlinker.setPattern(5, blinkerSequence);
			rightBlinker.setPattern(5, blinkerSequence);
			backBlinker.setPattern(5, blinkerSequence);
		}
		navigationLights = onoff;
		leftBlinker.isEnabled = onoff;
		rightBlinker.isEnabled = onoff;
		backBlinker.isEnabled = onoff;
	}
	void setFormationLights(bool onoff)
	{
		if (onoff == true && formationLights == false)
		{
			// just some example pattern for testing
			float blinkerSequence[5] = { 0.0f, 1.0f, 0.0f, 1.0f, 1.0f };

			leftBlinker.setPattern(5, blinkerSequence);
			rightBlinker.setPattern(5, blinkerSequence);
			backBlinker.setPattern(5, blinkerSequence);
		}
		formationLights = onoff;
		leftBlinker.isEnabled = onoff;
		rightBlinker.isEnabled = onoff;
		backBlinker.isEnabled = onoff;
	}
	void setLandingLights(bool onoff)
	{
		if (onoff == true && landingLights == false)
		{
			// use shorter simple pattern for testing in this case
			float blinkerSequence[4] = { 0.0f, 1.0f, 0.0f, 1.0f };

			leftBlinker.setPattern(4, blinkerSequence);
			rightBlinker.setPattern(4, blinkerSequence);
			backBlinker.setPattern(4, blinkerSequence);
		}
		landingLights = onoff;
		leftBlinker.isEnabled = onoff;
		rightBlinker.isEnabled = onoff;
		backBlinker.isEnabled = onoff;
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
		return leftBlinker.getCurrent();
	}
	float getRightLight() const
	{
		return rightBlinker.getCurrent();
	}
	float getBackLight() const
	{
		return backBlinker.getCurrent();
	}

	/*
	bool draggingChuteCap() const
	{}
	*/

	void toggleNavigationLights()
	{
		setNavigationLights(!navigationLights);
	}
	void toggleFormationLights()
	{
		setFormationLights(!formationLights);
	}
	void toggleLandingLights()
	{
		setLandingLights(!landingLights);
	}

	void onEject()
	{
		/*
		// TODO: check for three times in quick succession before doing more here
		if (ejectCount < 3)
		{
			ejectCount++;
			return;
		}
		*/
		// TODO: check time since first press (must be rapid)

		// pilot and seat gone
		ejectingSeat = false;
		canopyGone = true;
		actCanopy.m_current = 1.0;
		actCanopy.m_isWorking = false;
	}

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
		canopyGone = false;
		actCanopy.m_commanded = actCanopy.m_limiter.lower_limit;
		actCanopy.m_isWorking = true;
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
		if (canopyGone == false)
		{
			if (canopySwitchDown == false)
			{
				actCanopy.m_commanded = actCanopy.m_limiter.upper_limit;
			}
			else
			{
				actCanopy.m_commanded = actCanopy.m_limiter.lower_limit;
			}
			actCanopy.updateFrame(frameTime);
		}

		if (refuelingDoorOpen == true)
		{
			actRefuelingDoor.m_commanded = 1;
		}
		else
		{
			actRefuelingDoor.m_commanded = 0;
		}

		// TODO: refueling door support
		actRefuelingDoor.updateFrame(frameTime);

		// aero drag in case canopy is gone
		updateCanopyDrag(frameTime);

		// TODO: update cockpit pressure from env system
		// -> set to ambient pressure if canopy is gone


		// TODO: if in flight and canopy open -> canopy gone
		// also, if sealing is not working and internal pressure exceeds external -> canopy gone

		// some light blinking pattern/sequence support here,
		// set parameters where light is enabled
		leftBlinker.updateFrame(frameTime);
		rightBlinker.updateFrame(frameTime);
		backBlinker.updateFrame(frameTime);
	}
};

#endif // ifndef _F16AIRFRAME_H_

