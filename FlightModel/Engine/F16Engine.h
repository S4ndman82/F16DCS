#ifndef _F16ENGINE_H_
#define _F16ENGINE_H_

#include "../stdafx.h"

/*
		engine = 
		{
			Nmg	=	62, -- RPM at idle
			MinRUD	=	0, -- Min state of the throttle
			MaxRUD	=	1, -- Max state of the throttle
			MaksRUD	=	0.85, -- Military power state of the throttle
			ForsRUD	=	0.91, -- Afterburner state of the throttle
			typeng	=	E_TURBOJET_AB,
			
			hMaxEng	=	19, -- Max altitude for safe engine operation in km
			dcx_eng	=	0.0124, -- Engine drag coeficient
			cemax	=	1.24, -- not used for fuel calulation , only for AI routines to check flight time ( fuel calculation algorithm is built in )
			cefor	=	2.56, -- not used for fuel calulation , only for AI routines to check flight time ( fuel calculation algorithm is built in )
			dpdh_m	=	2000, --  altitude coefficient for max thrust
			dpdh_f	=	4200,  --  altitude coefficient for AB thrust

			-- M - Mach number
			-- Pmax - Engine thrust at military power
			-- Pfor - Engine thrust at AFB
			table_data = 
			{		--   M		Pmax		 Pfor	
				[1] = 	{0,	88000,	141000},
				[2] = 	{0.2,	80000,	143000},
				[3] = 	{0.4,	79000,	150000},
				[4] = 	{0.6,	82000,	165000},
				[5] = 	{0.7,	90000,	177000},
				[6] = 	{0.8,	94000,	193000},
				[7] = 	{0.9,	96000,	200000},
				[8] = 	{1,	100000,	205000},
				[9] = 	{1.1,	100000,	214000},
				[10] = 	{1.2,	98000,	222000},
				[11] = 	{1.3,	100000,	235000},
				[12] = 	{1.5,	98000,	258000},
				[13] = 	{1.8,	94000,	276000},
				[14] = 	{2,	88000,	283000},
				[15] = 	{2.2,	82000,	285000},
				[16] = 	{2.5,	80000,	287000},
				[17] = 	{3.9,	50000,	200000},
			}, -- end of table_data
		}, -- end of engine
*/

/*
some data from: http://www.jet-engine.net/miltfspec.html

Manufacturer	Model	Application(s)	Thrust	Thrust	SFC		SFC		Airflow		OPR		FPR		BPR		TIT		Number	Fan		LPC		HPC		HPT		IPT		LPT		Fan		Length	Width/		Dry	
										(dry)	(wet)	(dry)	(wet)	[lb/s]										Spools	Stgs	Stgs	Stgs	Stgs	Stgs	Stgs	Diameter		Diameter	Weight		
										[lbf]	[lbf]	[lb/lbf hr]	[lb/lbf hr]								[K]																[in]	[in]	[in]		[lb]

PW	F100-PW-200			F-16A/B			14,670	23,830	-		2.100	-			25.0	-		0.71	-		2		3		-		10		2		-		2		-		196.3	46.5		
PW	F100-PW-220			F-16A/B/C/D		14,670	23,830	-		2.100	228			25.0	-		0.71	-		2		3		-		10		2		-		2		-		208.0	46.5	3,200	
PW	F100-PW-220E		F-16A/B/C/D		14,670	23,830	-		2.100	-			25.0	-		-		-		2		3		-		10		2		-		2		-		196.3	46.5	3,151	

GE	F110-GE-100			F-16C/D			16,600	28,000	0.745	1.971	264			29.0	2.98	0.87	-		2		3		-		9		1		-		2		-		182.3	46.5	3,920	
GE	F110-GE-129			F-16C/D			17,000	29,000	-		1.900	270			30.7	-		0.76	-		2		3		-		9		1		-		2		-		181.9	46.5	3,980	
*/

namespace F16
{
	// Engine: Pratt & Whitney F100-PW-129 or General Electric F110-GE-129
	// Thrust: Pratt & Whitney: 65 kN, AB 106 kN; General Electric: 76 kN, AB 129 kN
	// -> adapt to support either one?
	// Turbine inlet temperature: 1,350 °C (2,460 °F)

	// other engines: F100-PW-220, F110-GE-100

	enum EngineType
	{
		ET_F110GE100, // F110-GE-100 // F-16 C/D
		ET_F110GE129, // F110-GE-129 // F-16 C/D
		ET_F100PW200, // F100-PW-200 // F-16A/B
		ET_F100PW220, // F100-PW-220 // F-16A/B/C/D
		ET_F100PW220E, // F100-PW-220E // F-16A/B/C/D
		ET_F100PW229, // F100-PW-229 // F-16 C/D
		ET_F100PW229A, // F100-PW-229A // F-16 C/D
		ET_Reserved
	};

	class F16Engine
	{
	protected:
		double m_power3;

	public:
		double	thrust_N; // Engine thrust (N)
		double	throttleInput;	// Throttle input command normalized (-1 to 1)

		double percentPower;
		double afterburner;

		// amount of fuel used in this engine setting in current flight conditions (temperature, airspeed..)
		double fuelPerFrame;

		// fuel flow: LEAN - RICH
		// -> 100pph increase?
		// range 0 - 80,000pph (instruments)

		// amount of air bypassing compressor and engine core
		//double bypassRatio;

		// oil pressure: 0-100 psi
		// directly related to engine rpm?
		double oilPressure;
		bool oilPressureWarning;

		// temperatures, overheat

		// which part of engine we need? 
		// likely we need calculation of each section separately?
		// cockpit gauge: 300..900 range
		double engineTemperature;

		double engineRPM; // rounds per minute: non-zero if shutdown in air?
		//double drag; // amount of drag if not running while in air? windmilling effect?

		double compressorRotation;

		bool starting; // "spooling up"
		bool stopping; // "spooling down"

		bool isIgnited; // if it is really running or just rotating from airflow? (out of fuel mid-air?)

		F16Engine() 
			: m_power3(0)
			, thrust_N(0)
			, throttleInput(0)
			, percentPower(0)
			, afterburner(0)
			, fuelPerFrame(0)
			, oilPressure(100)
			, oilPressureWarning(false)
			, engineTemperature(900)
			, engineRPM(0)
			, compressorRotation(0)
			, starting(false)
			, stopping(false)
			, isIgnited(true) // currently, have it as started always (check initial status handling etc.)
		{}
		~F16Engine() {}

		// engine can get torque from JFS to spoolup when starting,
		// set that here to determine rotation
		void setStarterTorque(double value)
		{
		}

		// additional air pressure/flow either when in air (airstart)
		// or other source such as engine bleed air feedback
		// to improve starting (avoid compressor stall),
		// set that here
		void setIntakeAirPressure(double value)
		{
		}

		// fuel pump provided pressure of fuel
		void setFuelPressure(double value)
		{
		}

		// MaksRUD	=	0.85, -- Military power state of the throttle
		// ForsRUD	=	0.91, -- Afterburner state of the throttle
		void setThrottleInputRaw(double value)
		{
			// old code, see if we need changes..
			double limited = limit(((-value + 1.0) / 2.0) * 100.0, 0.0, 100.0);

			throttleInput = limited;
		}
		double getThrottleInput() const
		{
			return throttleInput;
		}

		double getOilPressure() const
		{
			return oilPressure;
		}
		double getEngineTemperature() const
		{
			return engineTemperature;
		}
		double getFuelFlow() const
		{
			return fuelPerFrame;
		}

		// torque from engine can be used
		// on gearbox-connected equipment:
		// some threshold must be exceeded for them to function
		double getEngineTorque() const
		{
			if (isIgnited == false) // ignore throttle setting if engine is not running
			{
				return 0;
			}

			// no idea of the value currently, just have non-zero value for now
			return 1;
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

		void initEngineOff()
		{
			throttleInput = 0;
			fuelPerFrame = 0;

			starting = false;
			stopping = false;

			// temporary for testing
			isIgnited = false;
		}
		void initEngineIdle()
		{
			throttleInput = 1;
			fuelPerFrame = 1;

			starting = false;
			stopping = false;

			// temporary for testing
			isIgnited = true;
		}
		void initEngineCruise()
		{
			throttleInput = 50;
			fuelPerFrame = 10;

			starting = false;
			stopping = false;

			// temporary for testing
			isIgnited = true;
		}

		void startEngine()
		{
			//throttleInput = 0;
			//fuelPerFrame = 0;

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
			fuelPerFrame = 0;
		}

		double getLowPressureBleedAir() const
		{
			// output pressure 
			// as a simple function of engine RPM?
			// (seventh-stage)
			// -> fan duct, inlet anti-icing
			return 0;
		}
		double getHighPressureBleedAir() const
		{
			// output pressure 
			// as a simple function of engine RPM?
			// (thirteenth-stage)
			// -> EPU
			return 0;
		}

		// fuel use per frame in current conditions
		double getFuelPerFrame() const
		{
			return fuelPerFrame;
		}
		double getThrustN() const
		{
			return thrust_N;
		}

		// spool up when starting engine normally (with JFS, upto idle)
		void spoolUp(double frameTime)
		{
		}
		// spool down when stopping engine normally (from idle)
		void spoolDown(double frameTime)
		{
		}

		// inlet velocity and compressor rotational speed (blade aoa)
		bool isCompressorStall()
		{
			// TODO: determine conditions

			return false;
		}

		// engine returning to idle
		bool isRollback()
		{
			// TODO: determine conditions

			return false;
		}

		// low pressure compressor stages
		void lpcStage(double airpressure, double airvelocity, double frameTime)
		{
		}

		// high pressure compressor stages
		void hpcStage(double airpressure, double airvelocity, double frameTime)
		{
		}

		// combustion stage
		void combustionStage(double fuel, double airpressure, double airvelocity, double frameTime)
		{
		}

		void exhaustStage(double exhaustpressure, double frameTime)
		{
		}


		void airstart(double frameTime);

		void updateFrame(const double mach, double alt, double frameTime);

	};

	// airstart essentially needs enough airflow/pressure to rotate engine
	// (UFC, BUC)
	void F16Engine::airstart(double frameTime)
	{
		// check conditions if engine can ignite

		// when rpm within 25-40 percent, airstart possible (30 degree dive)
		// (spooldown)

		// another way is use of JFS

		// throttle setting, airspeed (windmilling effect)
	}


	// Coded from the simulator study document
	void F16Engine::updateFrame(const double mach, double alt, double frameTime)
	{
		// calculate intake airflow/pressure
		// -> windmilling if engine stopped
		// -> compressor stall?


		// calculate bleed air pressure at current engine rpm:
		// must rotate AB fuel pump at sufficient speed

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

		// TODO: usage by actual engine ?
		//fuelPerFrame =  10 * throttleInput * frameTime; //10 kg persecond
		fuelPerFrame =  10 * frameTime; //10 kg persecond
	}

	/*
	A table I read said that the F-16 C expended 415 kg of fuel per minute at mach .5 at sea level, 
	310 kg/min at mach .8 at 15,000 feet, 
	and was at peak fuel efficiency of 260 kg/min at mach 1.4 at 36,000 feet

	//if (afterburner -> 23 gallons per minute?

	-> speed, altitude, throttle -> fuel usage?
	*/
}

#endif // ifndef _F16ENGINE_H_
