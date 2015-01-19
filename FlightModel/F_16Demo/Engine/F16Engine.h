#include "../stdafx.h"

namespace F16
{
	// Coded from the simulator study document
	class F16Engine
	{
	protected:
		double m_power3;

	public:
		double m_thrust_N; // Engine thrust (N)

		F16Engine() 
			: m_power3(0)
			, m_thrust_N(0)
		{}
		~F16Engine() {}

		void updateFrame(double throttleInput, double mach, double alt, double frameTime);
	};

	void F16Engine::updateFrame(double throttleInput, double mach, double alt, double frameTime)
	{
		double percentPower = 0.0;

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
		double machLimited = limit(mach,0.2,1.0);
		double Tidle = (-24976.0 * machLimited + 9091.5) + ((alt/55000) * 12000.0);
		double Tmil = (-25958.0 * pow(machLimited,3.0) + 34336.0 * pow(machLimited,2.0) - 14575.0 * machLimited + 58137.0) + ((alt/50000.0) * -42000.0);
		double Tmax = (26702.0 * pow(machLimited,2.0) + 8661.4 * machLimited + 92756.0) + ((alt/50000.0) * -100000.0);

		double thrust = 0.0;
		if(m_power3 < 50.0)
		{
			thrust = Tidle + (Tmil-Tidle)*(m_power3/50.0);
		}
		else
		{
			thrust = Tmil + (Tmax-Tmil)*(m_power3 - 50.0)/50.0;
		}

		m_thrust_N = limit(thrust,0.0,129000.0);
	}
}
