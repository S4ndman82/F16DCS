/*
Sources of data:
- http://www.geaviation.com/military/engines/f110/
- https://www.grc.nasa.gov/WWW/K-12/airplane/burnth.html
-

*/

#ifndef _F16ENGINE_H_
#define _F16ENGINE_H_

#include "../stdafx.h"

#include "Atmosphere/F16Atmosphere.h"

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

		average_fuel_consumption 	= 0.245, -- this is highly relative, but good estimates are 36-40l/min = 28-31kg/min = 0.47-0.52kg/s -- 45l/min = 35kg/min = 0.583kg/s
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
	// air data as it passes through engine:
	// air is compressed, fuel added and ignited, various parameters affect burning
	// and air parameters change when passing through engine
	// -> air, air&fuel, exhaust gas
	class GasData
	{
	public:
		double pressure;
		double density;
		double velocity;
		double volume;
		double temperature;
		double humidity;
		double massflow;

		GasData() 
			: pressure(0), density(0), velocity(0), 
			volume(0), temperature(0), humidity(0), massflow(0)
		{}
		GasData(double press, double dens, double temp)
			: pressure(press), density(dens), velocity(0), 
			volume(0), temperature(temp), humidity(0), massflow(0)
		{}
		GasData(const GasData &other)
			: pressure(other.pressure), density(other.density), velocity(other.velocity), 
			volume(other.volume), temperature(other.temperature), humidity(other.humidity), massflow(other.massflow)
		{}

		GasData& operator=(const GasData &other)
		{
			if (this == &other) { return *this; }
			pressure = other.pressure;
			density = other.density;
			velocity = other.velocity;
			volume = other.volume;
			temperature = other.temperature;
			humidity = other.humidity;
			massflow = other.massflow;
			return *this;
		}

		~GasData() {}
	};

	// Engine: Pratt & Whitney F100-PW-129 or General Electric F110-GE-129
	// Thrust: Pratt & Whitney: 65 kN, AB 106 kN; General Electric: 76 kN, AB 129 kN
	// -> adapt to support either one?
	// Turbine inlet temperature: 1,350 °C (2,460 °F)

	// other engines: F100-PW-220, F110-GE-100

	enum EngineType
	{
		ET_F110GE100,	// F110-GE-100 // F-16 C/D
		ET_F110GE129,	// F110-GE-129 // F-16 C/D
		ET_F100PW200,	// F100-PW-200 // F-16A/B
		ET_F100PW220,	// F100-PW-220 // F-16A/B/C/D
		ET_F100PW220E,	// F100-PW-220E // F-16A/B/C/D
		ET_F100PW229,	// F100-PW-229 // F-16 C/D
		ET_F100PW229A,	// F100-PW-229A // F-16 C/D
		ET_Reserved
	};

	// parameters for each engine (remove unnecessary ones later)
	class F16EngineParameters
	{
	public:
		EngineType engineType;

		// same
		double maxDiameter; // cm
		double inletDiameter; // cm

		// differences, likely calculated differently:
		// amount bypassed vs. amount for normal intake
		double bypassRatio;

		F16EngineParameters(EngineType engine)
			: engineType(engine)
			, maxDiameter(46.5 * inchesToCentim) // in -> cm
			, inletDiameter(34.8 * inchesToCentim) // in -> cm
			, bypassRatio(0)
		{
			if (engineType == ET_F110GE129)
			{
				bypassRatio = 0.76;
			}
			else
			{
				bypassRatio = 0.36; //
			}
		}
		~F16EngineParameters() {}
	};


	class F16Engine
	{
	protected:
		double m_power3;

	public:
		double	thrust_N; // Engine thrust (N)
		double	throttleInput;	// Throttle input command normalized (-1 to 1)

		double percentPower;
		double afterburnerDraw; // just draw argument

		// amount of fuel used in this engine setting in current flight conditions (temperature, airspeed..)
		double fuelPerFrame;

		// fuel flow: LEAN - RICH
		// -> 100pph increase?
		// range 0 - 80,000pph (instruments)

		// amount of air bypassing compressor and engine core

		// oil pressure: 0-100 psi
		// directly related to engine rpm?
		double oilPressure;
		bool oilPressureWarning;

		// temperatures, overheat

		// which part of engine we need? 
		// likely we need calculation of each section separately?
		// cockpit gauge: 300..900 range
		double engineTemperature;
		double inletTemperature;
		double combustionPressure; // combustion chamber

		//CIVV : compressor inlet variable vanes
		// -> modify inlet

		double engineRPM; // rounds per minute: non-zero if shutdown in air?
		//double drag; // amount of drag if not running while in air? windmilling effect?

		// rotational inertia of engine
		double inertia;
		double coreinertia;
		
		double lpcRotation; // low pressure compressor rotation speed
		double hpcRotation; // high pressure compressor rotation speed

		double inletArea; // area of inlet

		bool starting; // "spooling up"
		bool stopping; // "spooling down"

		bool isIgnited; // if it is really running or just rotating from airflow? (out of fuel mid-air?)

		bool inhibitAbIgnition; // condition if AB is restricted 

		F16EngineParameters engineParams;
		F16Atmosphere *pAtmos;

		F16Engine(F16Atmosphere *atmos)
			: m_power3(0)
			, thrust_N(0)
			, throttleInput(0)
			, percentPower(0)
			, afterburnerDraw(0)
			, fuelPerFrame(0)
			, oilPressure(100)
			, oilPressureWarning(false)
			, engineTemperature(900)
			, engineRPM(0)
			, lpcRotation(0)
			, hpcRotation(0)
			, inletArea(0.613643025) // -> m^3
			, starting(false)
			, stopping(false)
			, isIgnited(true) // currently, have it as started always (check initial status handling etc.)
			, inhibitAbIgnition(false)
			, engineParams(ET_F100PW200)
			, pAtmos(atmos)
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
		/*
		double getEngineTorque() const
		{
			if (isIgnited == false) // ignore throttle setting if engine is not running
			{
				return 0;
			}

			// no idea of the value currently, just have non-zero value for now
			return 1;
		}
		*/

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
		bool isCompressorStall(double airvelocity, double rotationspeed)
		{
			// if air velocity exceeds compressor rotational speed (blade angle-of-attack)
			// -> compressor stall

			// TODO: determine conditions, correct formula
			//if (airvelocity > rotationspeed*degtorad) {}

			return false;
		}

		// engine returning to idle
		bool isRollback()
		{
			// TODO: determine conditions

			return false;
		}

		// combustor blow-out
		bool isBlowout()
		{
			// TODO: determine conditions

			return false;
		}

		// inlet stage, variable control (CIVV), air temperature
		double inletStage(GasData &gas, double frameTime)
		{
			// so, inlet area * pressure gives volume of air
			// volume * density gives mass of air flow
			// right?
			gas.volume = inletArea * gas.pressure;
			gas.massflow = gas.volume * gas.density;

			// three fans at this stage?
			// -> functionality

			return gas.pressure;
		}

		// low pressure compressor stages
		double lpcStage(GasData &gas, double frameTime)
		{
			// no lpc stages?
			/*
			if (isCompressorStall(airvelocity, lpcRotation) == true)
			{
				// -> stall
			}
			*/

			// no stages -> output same as input
			return gas.pressure;
		}

		// high pressure compressor stages
		double hpcStage(GasData &gas, double frameTime)
		{
			/*
			if (isCompressorStall(airvelocity, hpcRotation) == true)
			{
				// -> stall
			}
			*/

			// note: some rotors here have variable vanes here so compression could change..
			// depends on fuel control mode, throttle position and fuel pressure from AB pump

			/*
			// 10 or 9 hpc stages depending on engine
			//double stages[] = {}; // <- compression ratios
			// -> just for one engine now..
			// TODO: replace with correct ratios after testing,
			// ratios should be relative to previous stage?
			double stages[] = { 1.01, 1.01, 1.01, 1.01, 1.01, 1.01, 1.01, 1.01, 1.01, 1.01 };

			// multiply pressure by each compression stage
			//double press = gas.pressure;
			for (double d : stages) // <- C++11 supported
			{
				gas.pressure *= d;
			}
			*/

			// we don't know the ratios of each rotor,
			// but PW brochure says 32:1 compression 
			// -> use that for now

			gas.pressure *= 32;

			// part of air directed to bleed air -> AB fuel pump etc.

			// pressurization and dump valve control

			return gas.pressure;
		}

		// combustion stage
		double combustionStage(double fuel, GasData &gas, double frameTime)
		{
			// core temperature for self-ignition?
			// -> need to have external ignition support if too cold
			// -> windmilling (airstart) cooling engine?

			// TODO:
			//if (isBlowout() == true)

			// fuel/air mixture: rich/lean mixture of fuel, temperature, volume (pressure)
			// -> combustion gas to turbines

			// throttle-input -> ECU -> fuel injection amount for mixture:
			// fuel to airmassflow ratio * efficiency * (fuel properties) -> temperature

			// we need exhaust gas volume for turbine power and thrust output?

			// burn efficiency multiplier?
			// also fuel properties

			return 0;
		}

		// both hpt and lpt stages as one
		double turbineStage(GasData &gas, double frameTime)
		{
			// exhaust gas temperature: after hpt, before lpt

			// high and low pressure turbines -> lpc and hpc compressor rotation

			// 2 or 1 hpt stages depending on engine
			//double hptstages[] = {}
			// -> high pressure compressor rotation speed

			// 2 lpt stages
			//double lptstages[] = {}
			// -> low pressure compressor rotation speed

			return 0;
		}

		double exhaustStage(GasData &gas, double frameTime)
		{
			// bypass air injection?

			// nozzle control (CENC)

			// exhaust -> thrust, AB
			return 0;
		}


		// airstart essentially needs enough airflow/pressure to rotate engine
		// (UFC, BUC)
		void airstart(double frameTime)
		{
			// check conditions if engine can ignite

			// when rpm within 25-40 percent, airstart possible (30 degree dive)
			// (spooldown)

			// another way is use of JFS

			// throttle setting, airspeed (windmilling effect)
		}

		float getAfterburnerDraw() const
		{
			return (float)afterburnerDraw;
		}

		// TODO: we'll need to control nozzle position:
		// for example, landing gear out -> open nozzle more 
		// to reduce possibility of power loss
		float getNozzlePos() const
		{
			return 0;
		}

		void updateFrame(double frameTime);
	};



	// Coded from the simulator study document
	void F16Engine::updateFrame(double frameTime)
	{
		// calculate intake airflow/pressure
		// -> windmilling if engine stopped
		// -> compressor stall?

		// at subsonic speed, inlet does not matter
		// at supersonic speeds, inlet must reduce shockwaves

		// start by setting ambient conditions
		GasData gas(pAtmos->ambientPressure, pAtmos->ambientDensity, pAtmos->ambientTemperature_DegK);
		GasData bypass(gas); // <- according to bypass ratio, injection back to engine

		inletStage(gas, frameTime);
		bypass.volume = gas.volume * engineParams.bypassRatio; // after normal inlet calculated
		bypass.massflow = gas.massflow * engineParams.bypassRatio;

		lpcStage(gas, frameTime);
		hpcStage(gas, frameTime);

		// TODO: usage by actual engine ?
		// -> mixture by throttle and air massflow -> determine mixture
		//
		fuelPerFrame = 10 * frameTime; //10 kg persecond
		
		combustionStage(fuelPerFrame, gas, frameTime);

		turbineStage(gas, frameTime);
		exhaustStage(gas, frameTime);

		/*

		pressure = lpcStage(dynpressure, inletvelocity, frameTime);
		pressure = hpcStage(pressure, frameTime);


		double fuel = (throttleInput -> fuel mixture setting)
		pressure = combustionStage(fuel, pressure, frameTime);

		pressure = turbineStage(pressure, frameTime);
		pressure = exhaustStage(pressure, frameTime);
		*/


		// calculate bleed air pressure at current engine rpm:
		// must rotate AB fuel pump at sufficient speed

		afterburnerDraw = (throttleInput - 80.0) / 20.0;
		afterburnerDraw = limit(afterburnerDraw, 0.0, 1.0); // just draw argument
		/*
		if (inhibitAbIgnition == false)
		{
			afterburnerDraw = limit(afterburnerDraw, 0.0, 1.0); // just draw argument
		}
		else
		{
			// TODO: same argument controls nozzle position.. 
			// AB ignition inhibited -> no draw

			// limit nozzle to under AB ignition?
			afterburnerDraw = limit(afterburnerDraw, 0.0, .8); // just draw argument
		}
		*/

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

		double altFeet = pAtmos->getAltitudeFeet();

		//From Simulator Study document (use 0 altitude values for now)
		//TODO: This should really be a look-up table per the document reference but this is sufficient for now...
		double mach = pAtmos->getMachSpeed();
		double altTemp = (altFeet / 55000.0);
		double altTemp2 = (altFeet/50000.0);
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

	/*
	A table I read said that the F-16 C expended 415 kg of fuel per minute at mach .5 at sea level, 
	310 kg/min at mach .8 at 15,000 feet, 
	and was at peak fuel efficiency of 260 kg/min at mach 1.4 at 36,000 feet

	//if (afterburner -> 23 gallons per minute?

	-> speed, altitude, throttle -> fuel usage?
	*/
}

#endif // ifndef _F16ENGINE_H_
