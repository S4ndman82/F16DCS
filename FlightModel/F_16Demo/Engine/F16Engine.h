#ifndef _F16ENGINE_H_
#define _F16ENGINE_H_

#include "../stdafx.h"

namespace F16
{
	// Engine: Pratt & Whitney F100-PW-129 or General Electric F110-GE-129
	// Thrust: Pratt & Whitney: 65 kN, AB 106 kN; General Electric: 76 kN, AB 129 kN
	// -> adapt to support either one?

	// Coded from the simulator study document
	class F16Engine
	{
	protected:
		double m_power3;

	public:
		double		thrust_N; // Engine thrust (N)
		double		throttleInput;	// Throttle input command normalized (-1 to 1)

		double percentPower;
		double afterburner;

		// fuel flow: LEAN - RICH
		// -> 100pph increase?
		// range 0 - 80,000pph (instruments)

		// oil pressure: 0-100 psi

		// temperatures, overheat

		double engineRPM; // rounds per minute: non-zero if shutdown in air?
		//double drag; // amount of drag if not running while in air?

		bool starting; // "spooling up"
		bool stopping; // "spooling down"

		bool isIgnited; // if it is really running or just rotating from airflow? (out of fuel mid-air?)


		F16Engine() 
			: m_power3(0)
			, thrust_N(0)
			, throttleInput(0)
			, percentPower(0)
			, afterburner(0)
			, engineRPM(0)
			, starting(false)
			, stopping(false)
			, isIgnited(true) // currently, have it as started always (check initial status handling etc.)
		{}
		~F16Engine() {}

		void setThrottleInput(double value)
		{
			throttleInput = value;
		}
		double getThrottleInput() const
		{
			return throttleInput;
		}

		double getEngineRpm() const
		{
			if (isIgnited == false) // ignore throttle setting if engine is not running
			{
				return 0;
			}

			// ED_FM_ENGINE_1_RPM:
			return (throttleInput/100.0) * 3000;
		}
		double getEngineRelatedRpm() const
		{
			if (isIgnited == false)
			{
				return 0;
			}

			// ED_FM_ENGINE_1_RELATED_RPM:
			return (throttleInput/100.0);
		}
		double getEngineThrust() const
		{
			if (isIgnited == false)
			{
				return 0;
			}

			// ED_FM_ENGINE_1_THRUST:
			return (throttleInput/100.0) * 5000 * 9.81;
		}
		double getEngineRelatedThrust() const
		{
			if (isIgnited == false)
			{
				return 0;
			}

			// ED_FM_ENGINE_1_RELATED_THRUST:
			return (throttleInput/100.0);
		}

		void startEngine()
		{
			starting = true;
			stopping = false;

			// temporary for testing
			isIgnited = true;
		}
		void stopEngine()
		{
			stopping = true;
			starting = false;

			// temporary for testing
			isIgnited = false;
		}

		void updateFrame(const double mach, double alt, double frameTime);
	};

	void F16Engine::updateFrame(const double mach, double alt, double frameTime)
	{
		/*
		if (starting == true)
		{
			// TODO: update according to frametime

			// just skip to final result now..
			starting = false;
			isIgnited = true;
		}
		else if (stopping == true)
		{
			// TODO: update according to frametime

			// just skip to final result now..
			stopping = false;
			isIgnited = false;
		}

		if (isIgnited == false)
		{
			engineRPM = 0;
			afterburner = 0;
			percentPower = 0;
			thrust_N = 0;
			m_power3 = 0;
			return;
		}
		*/

		afterburner = (throttleInput - 80.0) / 20.0;
		if(throttleInput < 78.0)
		{
			percentPower = throttleInput * 0.6923;
		}
		else
		{
			percentPower = throttleInput *4.5455 - 354.55;
		}
		percentPower = limit(percentPower,0.0,100.0);

		double power1 = percentPower;
		double power2 = 0.0;
		double power3rate = 0.0;

		//if(!(F16::simInitialized))
		//{
		//	m_power3 = power1;
		//}

		if(power1 < 50.0)
		{
			if(m_power3 < 50.0)
			{
				power2 = power1;
				if((power2-m_power3) < 40.0)
				{
					power3rate = 1.0 * (power2 - m_power3);
				}
				else
				{
					power3rate = 0.1 * (power2 - m_power3);
				}
			}
			else
			{
				power2 = 40.0;
				power3rate = 5.0 * (power2 - m_power3);
			}
		}
		else
		{
			if(m_power3 < 50.0)
			{
				power2 = 60.0;
				if((power2-m_power3) < 40.0)
				{
					power3rate = 1.0 * (power2 - m_power3);
				}
				else
				{
					power3rate = 0.1 * (power2 - m_power3);
				}
			}
			else
			{
				power2 = power1;
				power3rate = 5.0 * (power2 - m_power3);
			}
		}

		m_power3 += (power3rate * frameTime);
		m_power3 = limit(m_power3,0.0,100.0);

		//From Simulator Study document (use 0 altitude values for now)
		//TODO: This should really be a look-up table per the document reference but this is sufficient for now...
		double altTemp = (alt/55000.0);
		double altTemp2 = (alt/50000.0);
		double machLimited = limit(mach,0.2,1.0);
		double Tidle = (-24976.0 * machLimited + 9091.5) + (altTemp * 12000.0);
		double Tmil = (-25958.0 * pow(machLimited,3.0) + 34336.0 * pow(machLimited,2.0) - 14575.0 * machLimited + 58137.0) + (altTemp2 * -42000.0);
		double Tmax = (26702.0 * pow(machLimited,2.0) + 8661.4 * machLimited + 92756.0) + (altTemp2 * -100000.0);

		double thrustTmp = 0.0;
		if(m_power3 < 50.0)
		{
			thrustTmp = Tidle + (Tmil-Tidle)*(m_power3/50.0);
		}
		else
		{
			thrustTmp = Tmil + (Tmax-Tmil)*(m_power3 - 50.0)/50.0;
		}

		thrust_N = limit(thrustTmp,0.0,129000.0);
	}
}

#endif // ifndef _F16ENGINE_H_
