#ifndef _F16AIRFRAME_H_
#define _F16AIRFRAME_H_

#include "../stdafx.h"

namespace F16
{
	// - canopy status
	// - refueling slot status
	// - dragging chute status
	// - tail hook support?
	// - damage in sections? 
	// - accumulated over-g stress?

	class F16Airframe
	{
	public:
		double	canopyAngle;		// Canopy status/angle {0=closed;0.9=elevated;1=no draw}

		F16Airframe() 
			: canopyAngle(0)
		{}
		~F16Airframe() {}

		void setCanopyOpen()
		{
			canopyAngle = 0.9; // up
		}
		void setCanopyClosed()
		{
			canopyAngle = 0; // down
		}
		void setCanopyGone()
		{
			canopyAngle = 1.0; // gone
		}

		// canopy open/close toggle
		void canopyToggle()
		{
			if (canopyAngle > 0 && canopyAngle < 1.0)
			{
				canopyAngle = 0; // down
			}
			else
			{
				canopyAngle = 0.9; // up
			}
		}

		void canopyJettison()
		{
			canopyAngle = 1.0; // gone
		}

		// draw angle of canopy {0=closed;0.9=elevated;1=no draw}
		double getCanopyAngle() const
		{
			return canopyAngle;
		}

		// get cockpit pressure in pascals over external (get update from oxygen system also)
		double getCockpitPressure() const
		{
			return 0;
		}

		// accumulate stress?
		// update pressure?
		void updateFrame(const double frameTime)
		{
		}
	};
}

#endif // ifndef _F16AIRFRAME_H_

