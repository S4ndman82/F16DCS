#ifndef _F16OXYGEN_H_
#define _F16OXYGEN_H_

#include "../stdafx.h"

namespace F16
{
	// amount of bleed air from engine (oxygen generator)
	// oxygen tanks
	// valve open/not

	// 5-liter liquid oxygen -> diluter (0..100 percent O2)

	class F16Oxygen
	{
	protected:
		double pressure; // pressure provided (pascals)

	public:
		F16Oxygen() 
			: pressure(0)
		{}
		~F16Oxygen() {}

		double getPressure() const
		{
			return pressure;
		}

		void updateFrame(const double frameTime)
		{
			// double use = frameTime * usage;
			//tanks -= use;
			//tanks += o2_gen;
		}
	};
}

#endif // ifndef _F16OXYGEN_H_
