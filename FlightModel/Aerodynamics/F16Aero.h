// Source Data:
// 1) F-16 University of Minnesota Non-Linear Flight Model
//	  http://www.aem.umn.edu/people/faculty/balas/darpa_sec/SEC.Software.html
// 2) NASA TP 1538 Simulator Study of Stall/Post-Stall Characteristics of a 
//	  Fighter Airplane With Relaxed Longitudinal Static Stability

#ifndef _F16AERO_H_
#define _F16AERO_H_

#include <cmath>

#include "F16Constants.h"
#include "F16AeroData.h"
#include "F16AeroFunction.h"

#include "FlightControls/F16FcsCommon.h"

#include "Atmosphere/F16Atmosphere.h"
#include "Atmosphere/F16GroundSurface.h"

class F16Aero
{
protected:
	F16Atmosphere *pAtmos;
	F16GroundSurface *pGrounds;

	double		m_Cx_total;
	double		m_Cz_total;
	double		m_Cm_total;
	double		m_Cy_total;
	double		m_Cn_total;
	double		m_Cl_total;

	AERO_Function fn_Cx;
	AERO_Function fn_CxEle0;
	AERO_Function fn_Cz;
	AERO_Function fn_CzEle0;
	AERO_Function fn_Cm;
	AERO_Function fn_CmEle0;
	AERO_Function fn_Cy;
	AERO_Function fn_Cn;
	AERO_Function fn_CnEle0;
	AERO_Function fn_Cl;
	AERO_Function fn_ClEle0;
	AERO_Function fn_Cx_lef;
	AERO_Function fn_Cz_lef;
	AERO_Function fn_Cm_lef;
	AERO_Function fn_Cy_lef;
	AERO_Function fn_Cn_lef;
	AERO_Function fn_Cl_lef;
	AERO_Function fn_CXq;
	AERO_Function fn_CZq;
	AERO_Function fn_CMq;
	AERO_Function fn_CYp;
	AERO_Function fn_CYr;
	AERO_Function fn_CNr;
	AERO_Function fn_CNp;
	AERO_Function fn_CLp;
	AERO_Function fn_CLr;
	AERO_Function fn_delta_CXq_lef;
	AERO_Function fn_delta_CYr_lef;
	AERO_Function fn_delta_CYp_lef;
	AERO_Function fn_delta_CZq_lef;
	AERO_Function fn_delta_CLr_lef;
	AERO_Function fn_delta_CLp_lef;
	AERO_Function fn_delta_CMq_lef;
	AERO_Function fn_delta_CNr_lef;
	AERO_Function fn_delta_CNp_lef;
	AERO_Function fn_Cy_r30;
	AERO_Function fn_Cn_r30;
	AERO_Function fn_Cl_r30;
	AERO_Function fn_Cy_a20;
	AERO_Function fn_Cy_a20_lef;
	AERO_Function fn_Cn_a20;
	AERO_Function fn_Cn_a20_lef;
	AERO_Function fn_Cl_a20;
	AERO_Function fn_Cl_a20_lef;
	AERO_Function fn_delta_CNbeta;
	AERO_Function fn_delta_CLbeta;
	AERO_Function fn_delta_Cm;
	AERO_Function fn_eta_el;

	double m_CyAilerons;
	double m_CnAilerons;
	double m_ClAilerons;
	double m_CyRudder;
	double m_CnRudder;
	double m_ClRudder;
	double m_CxFlaps;
	double m_CzFlaps;
	double m_CxAirbrake;
	double m_diffCgPCT;

public:
	F16Aero(F16Atmosphere *atmos, F16GroundSurface *grounds) :
		pAtmos(atmos),
		pGrounds(grounds),
		m_Cx_total(0),
		m_Cz_total(0),
		m_Cm_total(0),
		m_Cy_total(0),
		m_Cn_total(0),
		m_Cl_total(0),
		fn_Cx(3, F16::_CxData),
		fn_CxEle0(3, F16::_CxData),
		fn_Cz(3, F16::_CzData),
		fn_CzEle0(3, F16::_CzData),
		fn_Cm(3, F16::_CmData),
		fn_CmEle0(3, F16::_CmData),
		fn_Cy(2, F16::_CyData),
		fn_Cn(3, F16::_CnData),
		fn_CnEle0(3, F16::_CnData),
		fn_Cl(3, F16::_ClData),
		fn_ClEle0(3, F16::_ClData),
		fn_Cx_lef(2, F16::_Cx_lefData),
		fn_Cz_lef(2, F16::_Cz_lefData),
		fn_Cm_lef(2, F16::_Cm_lefData),
		fn_Cy_lef(2, F16::_Cy_lefData),
		fn_Cn_lef(2, F16::_Cn_lefData),
		fn_Cl_lef(2, F16::_Cl_lefData),
		fn_CXq(1, F16::_CxqData),
		fn_CZq(1, F16::_CzqData),
		fn_CMq(1, F16::_CmqData),
		fn_CYp(1, F16::_CypData),
		fn_CYr(1, F16::_CyrData),
		fn_CNr(1, F16::_CnrData),
		fn_CNp(1, F16::_CnpData),
		fn_CLp(1, F16::_ClpData),
		fn_CLr(1, F16::_ClrData),
		fn_delta_CXq_lef(1, F16::_delta_CXq_lefData),
		fn_delta_CYr_lef(1, F16::_delta_CYr_lefData),
		fn_delta_CYp_lef(1, F16::_delta_CYp_lefData),
		fn_delta_CZq_lef(1, F16::_delta_CZq_lefData),
		fn_delta_CLr_lef(1, F16::_delta_CLr_lefData),
		fn_delta_CLp_lef(1, F16::_delta_CLp_lefData),
		fn_delta_CMq_lef(1, F16::_delta_CMq_lefData),
		fn_delta_CNr_lef(1, F16::_delta_CNr_lefData),
		fn_delta_CNp_lef(1, F16::_delta_CNp_lefData),
		fn_Cy_r30(2, F16::_Cy_r30Data),
		fn_Cn_r30(2, F16::_Cn_r30Data),
		fn_Cl_r30(2, F16::_Cl_r30Data),
		fn_Cy_a20(2, F16::_Cy_a20Data),
		fn_Cy_a20_lef(2, F16::_Cy_a20_lefData),
		fn_Cn_a20(2, F16::_Cn_a20Data),
		fn_Cn_a20_lef(2, F16::_Cn_a20_lefData),
		fn_Cl_a20(2, F16::_Cl_a20Data),
		fn_Cl_a20_lef(2, F16::_Cl_a20_lefData),
		fn_delta_CNbeta(1, F16::_delta_CNbetaData),
		fn_delta_CLbeta(1, F16::_delta_CLbetaData),
		fn_delta_Cm(1, F16::_delta_CmData),
		fn_eta_el(1, F16::_eta_elData),
		m_CyAilerons(0), m_CnAilerons(0), m_ClAilerons(0),
		m_CyRudder(0), m_CnRudder(0), m_ClRudder(0),
		m_CxFlaps(0), m_CzFlaps(0),
		m_CxAirbrake(0),
		m_diffCgPCT(0)
	{
		initfn();
	}
	~F16Aero() {}
	void initfn();

	void updateFrame(const F16BodyState &bstate, F16FlightSurface &fsurf, const double frameTime)
	{
		const double alpha = limit(bstate.alpha_DEG, -20.0, 90.0);
		const double beta = limit(bstate.beta_DEG, -30.0, 30.0);

		// TODO: use left and right rudder angles
		// for now, symmetric use
		// TODO: support differential mode
		const double el = -fsurf.pitch_Command;

		// TODO: speed brake handling..
		// TODO Speedbrakes aero (from JBSim F16.xml config)

		/* hifi_C */
		fn_Cx.interpnf3(alpha, beta, el); //CX0120_ALPHA1_BETA1_DH1_201.dat
		fn_Cz.interpnf3(alpha, beta, el); //CZ0120_ALPHA1_BETA1_DH1_301.dat
		fn_Cm.interpnf3(alpha, beta, el); //CM0120_ALPHA1_BETA1_DH1_101.dat
		fn_Cy.interpnf2(alpha, beta); // CY0320_ALPHA1_BETA1_401.dat
		fn_Cn.interpnf3(alpha, beta, el); //CN0120_ALPHA1_BETA1_DH2_501.dat
		fn_Cl.interpnf3(alpha, beta, el);

		// also with zero elevator
		fn_CxEle0.interpnf3(alpha, beta, 0); //CX0120_ALPHA1_BETA1_DH1_201.dat
		fn_CzEle0.interpnf3(alpha, beta, 0); //CZ0120_ALPHA1_BETA1_DH1_301.dat
		fn_CmEle0.interpnf3(alpha, beta, 0); //CM0120_ALPHA1_BETA1_DH1_101.dat
		fn_CnEle0.interpnf3(alpha, beta, 0); //CN0120_ALPHA1_BETA1_DH2_501.dat
		fn_ClEle0.interpnf3(alpha, beta, 0);

		/* hifi_damping */
		fn_CXq.interpnf1(alpha); //CX1120_ALPHA1_204.dat
		fn_CYr.interpnf1(alpha); //CY1320_ALPHA1_406.dat
		fn_CYp.interpnf1(alpha); //CY1220_ALPHA1_408.dat
		fn_CZq.interpnf1(alpha); //CZ1120_ALPHA1_304.dat
		fn_CLr.interpnf1(alpha); //CL1320_ALPHA1_606.dat
		fn_CLp.interpnf1(alpha); //CL1220_ALPHA1_608.dat
		fn_CMq.interpnf1(alpha); //CM1120_ALPHA1_104.dat
		fn_CNr.interpnf1(alpha); //CN1320_ALPHA1_506.dat
		fn_CNp.interpnf1(alpha); //CN1220_ALPHA1_508.dat

		/* hifi_C_lef */ // (leading-edge flap)
		fn_Cx_lef.interpnf2Lim(alpha, beta); //CX0820_ALPHA2_BETA1_202.dat
		fn_Cz_lef.interpnf2Lim(alpha, beta); //CZ0820_ALPHA2_BETA1_302.dat
		fn_Cm_lef.interpnf2Lim(alpha, beta); //CM0820_ALPHA2_BETA1_102.dat
		fn_Cy_lef.interpnf2Lim(alpha, beta); //CY0820_ALPHA2_BETA1_402.dat
		fn_Cn_lef.interpnf2Lim(alpha, beta); //CN0820_ALPHA2_BETA1_502.dat
		fn_Cl_lef.interpnf2Lim(alpha, beta);

		/* hifi_damping_lef */
		fn_delta_CXq_lef.interpnf1Lim(alpha); //CX1420_ALPHA2_205.dat
		fn_delta_CYr_lef.interpnf1Lim(alpha); //CY1620_ALPHA2_407.dat
		fn_delta_CYp_lef.interpnf1Lim(alpha); //CY1520_ALPHA2_409.dat
		fn_delta_CZq_lef.interpnf1Lim(alpha); //CZ1420_ALPHA2_305.dat
		fn_delta_CLr_lef.interpnf1Lim(alpha); //CL1620_ALPHA2_607.dat
		fn_delta_CLp_lef.interpnf1Lim(alpha); //CL1520_ALPHA2_609.dat
		fn_delta_CMq_lef.interpnf1Lim(alpha); //CM1420_ALPHA2_105.dat
		fn_delta_CNr_lef.interpnf1Lim(alpha); //CN1620_ALPHA2_507.dat
		fn_delta_CNp_lef.interpnf1Lim(alpha); //CN1520_ALPHA2_509.dat

		/* hifi_rudder */
		fn_Cy_r30.interpnf2(alpha, beta); //CY0720_ALPHA1_BETA1_405.dat
		fn_Cn_r30.interpnf2(alpha, beta); //CN0720_ALPHA1_BETA1_503.dat
		fn_Cl_r30.interpnf2(alpha, beta); //CL0720_ALPHA1_BETA1_603.dat

		/* hifi_ailerons */
		fn_Cy_a20.interpnf2(alpha, beta); //CY0620_ALPHA1_BETA1_403.dat
		fn_Cn_a20.interpnf2(alpha, beta); //CN0620_ALPHA1_BETA1_504.dat
		fn_Cl_a20.interpnf2(alpha, beta); //CL0620_ALPHA1_BETA1_604.dat

		fn_Cy_a20_lef.interpnf2Lim(alpha, beta); //CY0920_ALPHA2_BETA1_404.dat
		fn_Cn_a20_lef.interpnf2Lim(alpha, beta); //CN0920_ALPHA2_BETA1_505.dat
		fn_Cl_a20_lef.interpnf2Lim(alpha, beta); //CL0920_ALPHA2_BETA1_605.dat

		/* hifi_other_coeffs */
		fn_delta_CNbeta.interpnf1(alpha); //CN9999_ALPHA1_brett.dat
		fn_delta_CLbeta.interpnf1(alpha); //CL9999_ALPHA1_brett.dat
		fn_delta_Cm.interpnf1(alpha); //CM9999_ALPHA1_brett.dat
		fn_eta_el.interpnf1(el); //ETA_DH1_brett.dat

		//Cm_delta_ds = 0;       /* ignore deep-stall regime, delta_Cm_ds = 0 */

	}

	// In low speeds, lift is in front of reference CG,
	// in mach 1 lift is at CG position,
	// over mach 1 lift aft of CG position (towards tail)
	// -> aerodynamic CG is different from "weight" (mass CG)
	// -> this needs to be calculated as function of velocity
	//
	void getAeroCgDiff(const double dynamicPressure_NM2, const double machNumber)
	{
		// CG may vary from 0.30 - 0.39, 0.35 at mach 1 (reference point)
		//const double diffCgPCT = (F16::referenceCG_PCT - F16::actualCG_PCT);

		double diffCgPCT = 0.0;

		// TODO: make actual calculations here (linear function)
		if (machNumber < 1)
		{
			diffCgPCT = F16::referenceCG_PCT - 0.30;
		}
		else if (machNumber > 1)
		{
			diffCgPCT = F16::referenceCG_PCT - 0.39;
		}
		m_diffCgPCT = diffCgPCT;
	}

	// drag caused by aircraft skin in contact with air (friction)
	// see: http://adg.stanford.edu/aa241/drag/wettedarea.html
	//
	double getWettedAreaDrag(const double dynamicPressure_NM2, const double machNumber)
	{
		//Sw = 2.0 * (1 + 0.2 t/c) * Se;
		return 0;
	}

	void getAirbrakeDrag(const double dynamicPressure_NM2, F16FlightSurface &fsurf)
	{
		double airbrakeDrag = 0;

		// after actuator move, calculate new drag at new position

		// TODO: switch to actual angles instead of percentages
		//double angle = cos(airbrakeActuator.m_current);

		// TEST!
		// just use full now for testing
		//double force = dynamicPressure_LBFT2 * 16.0 * cos(60) * 0.7;

		/* source: http://www.f-16.net/forum/viewtopic.php?t=11398
		Because landing is such a low speed, I did not bother to calculate those forces.
		But to estimate the force on the speedbrake at landing, you can use the dynamic pressure at landing speed x speedbrake area x cos 60 deg x Cd

		dynamic pressure q ~ 125 lb/sq ft (from q/M^2 = 1480 lb/sq ft)
		area ~ 4 sq ft x 4
		cos 60 deg = .866
		Cd ~ .7

		total drag force ~ 1212 lb

		That is the total force (parallel to the fuselage centerline) on all four panels at landing speed. It is much less than 3 tons.
		*/
		// ~1.48645m^2 area

		//double force = dynamicPressure_LBFT2 * 16.0 * cos(60) * 0.7;
		//double force = dynamicPressure_NM2 * F16::airbrakeArea_m2 * cos(60) * 0.7;

		// symmetric operation, should not make difference which one is used..?
		if (fsurf.airbrake_Left_PCT > 0 || fsurf.airbrake_Right_PCT > 0)
		{
			// for both sides separately?
			//double force = dynamicPressure_NM2 * (F16::airbrakeArea_m2 / 2);
			double force = dynamicPressure_NM2 * F16::airbrakeArea_m2;
			double CDAirbrake = cos(fsurf.airbrake_Left_PCT) * 0.7;
			//airbrakeDrag = -(CDAirbrake * cos(F16::degtorad));

			airbrakeDrag = -(CDAirbrake);

			//double pressureAreaFT2 = airbrakeArea_FT2 * dynamicPressure_LBFT2;
			//double airbrake_DEG = (airbrakeActuator.m_current * 60); // <- PCT to DEG
			//airbrakeDrag = -(0.7 * cos(airbrake_DEG));
		}
		else
		{
			airbrakeDrag = 0;
		}
		m_CxAirbrake = airbrakeDrag;
	}

	void getFlapCoeff(const double flap_PCT, const double body_alpha_DEG, double &CzFlap, double &CxFlap) const
	{
		// this does not work correctly when flap and aileron are combined

		// FLAPS (From JBSim F16.xml config)
		double CLFlaps = 0.35 * flap_PCT;
		double CDFlaps = 0.08 * flap_PCT;
		CzFlap = -(CLFlaps * cos(body_alpha_DEG * F16::degtorad) + CDFlaps * sin(F16::degtorad));
		CxFlap = -(-CLFlaps * sin(body_alpha_DEG * F16::degtorad) + CDFlaps * cos(F16::degtorad));
	}
	void getFlapsCoeff(const double dynamicPressure_NM2, const F16FlightSurface &fsurf, const F16BodyState &bstate)
	{
		double CzLeft = 0.0, CzRight = 0.0, CxLeft = 0.0, CxRight = 0.0;
		getFlapCoeff(fsurf.flap_Left_PCT, bstate.alpha_DEG, CzLeft, CxLeft);
		getFlapCoeff(fsurf.flap_Right_PCT, bstate.alpha_DEG, CzRight, CxRight);

		// this does not work correctly when flap and aileron are combined

		// check this
		m_CzFlaps = CzLeft + CzRight;
		m_CxFlaps = CxLeft + CxRight;
	}

	double getAileronCoeff(const double fn_C_a20, const double fn_C_a20_lef, const double fn_C, const double fn_C_lef, const F16FlightSurface &fsurf) const
	{
		const double leadingEdgeFlap_PCT = fsurf.leadingEdgeFlap_Right_PCT;
		const double C_delta_a20 = fn_C_a20 - fn_C;
		const double C_delta_a20_lef = fn_C_a20_lef - fn_C_lef - C_delta_a20;
		const double dail = C_delta_a20 + C_delta_a20_lef*leadingEdgeFlap_PCT; // <- lef symmetric

		// this does not work correctly when flap and aileron are combined

		// check
		// dail * (fsurf.aileron_Right_PCT + fsurf.aileron_Left_PCT);
		return dail*fsurf.flaperon_Right_PCT + dail*fsurf.flaperon_Left_PCT;
	}
	void getAileronsCoeff(const F16FlightSurface &fsurf)
	{
		m_CyAilerons = getAileronCoeff(fn_Cy_a20.m_result,
										fn_Cy_a20_lef.m_result,
										fn_Cy.m_result,
										fn_Cy_lef.m_result,
										fsurf);
		m_CnAilerons = getAileronCoeff(fn_Cn_a20.m_result,
										fn_Cn_a20_lef.m_result,
										fn_CnEle0.m_result,
										fn_Cn_lef.m_result,
										fsurf);
		m_ClAilerons = getAileronCoeff(fn_Cl_a20.m_result, 
										fn_Cl_a20_lef.m_result, 
										fn_ClEle0.m_result, 
										fn_Cl_lef.m_result, 
										fsurf);
	}

	void getRudderCoeff(const F16FlightSurface &fsurf)
	{
		const double Cy_delta_r30 = fn_Cy_r30.m_result - fn_Cy.m_result;
		const double Cn_delta_r30 = fn_Cn_r30.m_result - fn_CnEle0.m_result;
		const double Cl_delta_r30 = fn_Cl_r30.m_result - fn_ClEle0.m_result;

		m_CyRudder = Cy_delta_r30*fsurf.rudder_PCT;
		m_CnRudder = Cn_delta_r30*fsurf.rudder_PCT;
		m_ClRudder = Cl_delta_r30*fsurf.rudder_PCT;
	}

	/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	compute Cx_tot, Cz_tot, Cm_tot, Cy_tot, Cn_tot, and Cl_total
	(as on NASA report p37-40)
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
	void computeTotals(F16FlightSurface &fsurf, F16BodyState &bstate,
					const double LgCxGearAero, const double LgCzGearAero)
	{
		// in original nlplant there is 2*vt, but is that because lift and drag 
		// are not calculated for both sides separately? (no support for differential deflections)
		const double AerototalVelocity_FPS = pAtmos->getAeroTotalVelocityFPS() * 2; // <- why 2*vt ?
		double meanChordFPS = 0.0;
		double wingSpanFPS = 0.0;
		if (AerototalVelocity_FPS != 0.0)
		{
			meanChordFPS = (F16::meanChord_FT / AerototalVelocity_FPS);
			wingSpanFPS = (F16::wingSpan_FT / AerototalVelocity_FPS);
		}

		// TODO: left/right side when they can differ according to flight control system
		const double leadingEdgeFlap_PCT = fsurf.leadingEdgeFlap_Right_PCT;

		// TODO: dynamic CG to calculations, uses hardcoded "real" position now
		// (check: does this actually consider the RSS lift at non-CG position?)
		//
		// In low speeds, lift is in front of reference CG,
		// in mach 1 lift is at CG position,
		// over mach 1 lift aft of CG position (towards tail)
		// -> aerodynamic CG is different from "weight" (mass CG)
		// -> this needs to be calculated as function of velocity
		//
		//const double diffCgPCT = (F16::referenceCG_PCT - F16::actualCG_PCT);
		const double meanChordPerWingSpan = (F16::meanChord_FT / F16::wingSpan_FT);

		// calculate values based on interpolation results
		// TODO: duplicate this for left and right airflow (with differential stabilizer)?
		const double Cx_delta_lef = fn_Cx_lef.m_result - fn_CxEle0.m_result;
		const double Cz_delta_lef = fn_Cz_lef.m_result - fn_CzEle0.m_result;
		const double Cm_delta_lef = fn_Cm_lef.m_result - fn_CmEle0.m_result;
		const double Cy_delta_lef = fn_Cy_lef.m_result - fn_Cy.m_result;
		const double Cn_delta_lef = fn_Cn_lef.m_result - fn_CnEle0.m_result;
		const double Cl_delta_lef = fn_Cl_lef.m_result - fn_ClEle0.m_result;

		// get aerodynamic lift position as function of speed (difference from reference CG)
		getAeroCgDiff(pAtmos->totalVelocity, pAtmos->machNumber);
		getFlapsCoeff(pAtmos->dynamicPressure, fsurf, bstate);
		getAirbrakeDrag(pAtmos->dynamicPressure, fsurf);
		getAileronsCoeff(fsurf);
		getRudderCoeff(fsurf);

		/* XXXXXXXX Cx_tot XXXXXXXX */
		double dXdQ = meanChordFPS * (fn_CXq.m_result + fn_delta_CXq_lef.m_result*leadingEdgeFlap_PCT);
		m_Cx_total = fn_Cx.m_result + Cx_delta_lef*leadingEdgeFlap_PCT + dXdQ*bstate.pitchRate_RPS;
		m_Cx_total += m_CxFlaps + LgCxGearAero;

		/* airbrake - testing now*/
		m_Cx_total += m_CxAirbrake;

		/* ZZZZZZZZ Cz_tot ZZZZZZZZ */ 
		double dZdQ = meanChordFPS * (fn_CZq.m_result + Cz_delta_lef*leadingEdgeFlap_PCT);
		m_Cz_total = fn_Cz.m_result + Cz_delta_lef*leadingEdgeFlap_PCT + dZdQ*bstate.pitchRate_RPS;
		m_Cz_total += m_CzFlaps + LgCzGearAero;

		/* MMMMMMMM Cm_tot MMMMMMMM */ 
		/* ignore deep-stall regime, delta_Cm_ds = 0 */
		double dMdQ = meanChordFPS * (fn_CMq.m_result + fn_delta_CMq_lef.m_result*leadingEdgeFlap_PCT);
		double CmDelta = fn_delta_Cm.m_result + 0; // Cm_delta + Cm_delta_ds (0)
		m_Cm_total = fn_Cm.m_result*fn_eta_el.m_result + m_Cz_total*m_diffCgPCT
			+ Cm_delta_lef*leadingEdgeFlap_PCT + dMdQ*bstate.pitchRate_RPS + CmDelta;

		/* YYYYYYYY Cy_tot YYYYYYYY */
		double dYdR = wingSpanFPS * (fn_CYr.m_result + fn_delta_CYr_lef.m_result*leadingEdgeFlap_PCT);
		double dYdP = wingSpanFPS * (fn_CYp.m_result + fn_delta_CYp_lef.m_result*leadingEdgeFlap_PCT);
		m_Cy_total = fn_Cy.m_result + Cy_delta_lef*leadingEdgeFlap_PCT + m_CyAilerons + m_CyRudder
			+ dYdR*bstate.yawRate_RPS + dYdP*bstate.rollRate_RPS;
	
		/* NNNNNNNN Cn_tot NNNNNNNN */ 
		double dNdR = wingSpanFPS * (fn_CNr.m_result + fn_delta_CNr_lef.m_result*leadingEdgeFlap_PCT);
		double dNdP = wingSpanFPS * (fn_CNp.m_result + fn_delta_CNp_lef.m_result*leadingEdgeFlap_PCT);
		double CnDeltaBetaDeg = fn_delta_CNbeta.m_result*bstate.beta_DEG;
		m_Cn_total = fn_Cn.m_result + Cn_delta_lef*leadingEdgeFlap_PCT - m_Cy_total*m_diffCgPCT*meanChordPerWingSpan
			+ m_CnAilerons + m_CnRudder + dNdR*bstate.yawRate_RPS + dNdP*bstate.rollRate_RPS + CnDeltaBetaDeg;

		/* LLLLLLLL Cl_total LLLLLLLL */
		double dLdR = wingSpanFPS * (fn_CLr.m_result + fn_delta_CLr_lef.m_result*leadingEdgeFlap_PCT);
		double dLdP = wingSpanFPS * (fn_CLp.m_result + fn_delta_CLp_lef.m_result*leadingEdgeFlap_PCT);
		double ClDeltaBetaDeg = fn_delta_CLbeta.m_result*bstate.beta_DEG;
		m_Cl_total = fn_Cl.m_result + Cl_delta_lef*leadingEdgeFlap_PCT + m_ClAilerons + m_ClRudder
			+ dLdR*bstate.yawRate_RPS + dLdP*bstate.rollRate_RPS + ClDeltaBetaDeg;
	}


	double getCxTotal() const { return m_Cx_total; }
	double getCzTotal() const { return m_Cz_total; }
	double getCmTotal() const { return m_Cm_total; }
	double getCyTotal() const { return m_Cy_total; }
	double getCnTotal() const { return m_Cn_total; }
	double getClTotal() const { return m_Cl_total; }

}; // class F16Aero

void F16Aero::initfn()
{
	fn_Cx.init();
	fn_Cx.ndinfo.nPoints[0] = F16::alpha1_size;
	fn_Cx.ndinfo.nPoints[1] = F16::beta1_size;
	fn_Cx.ndinfo.nPoints[2] = F16::dh1_size;
	fn_Cx.m_Xmat[0] = F16::alpha1;
	fn_Cx.m_Xmat[1] = F16::beta1;
	fn_Cx.m_Xmat[2] = F16::dh1;

	fn_CxEle0.init();
	fn_CxEle0.ndinfo.nPoints[0] = F16::alpha1_size;
	fn_CxEle0.ndinfo.nPoints[1] = F16::beta1_size;
	fn_CxEle0.ndinfo.nPoints[2] = F16::dh1_size;
	fn_CxEle0.m_Xmat[0] = F16::alpha1;
	fn_CxEle0.m_Xmat[1] = F16::beta1;
	fn_CxEle0.m_Xmat[2] = F16::dh1;

	fn_Cz.init(); /* alpha,beta,dele */
	fn_Cz.ndinfo.nPoints[0] = F16::alpha1_size;	/* Alpha npoints */
	fn_Cz.ndinfo.nPoints[1] = F16::beta1_size; /* Beta npoints  */
	fn_Cz.ndinfo.nPoints[2] = F16::dh1_size;  /* dele npoints  */
	fn_Cz.m_Xmat[0] = F16::alpha1;
	fn_Cz.m_Xmat[1] = F16::beta1;
	fn_Cz.m_Xmat[2] = F16::dh1;

	fn_CzEle0.init(); /* alpha,beta,dele */
	fn_CzEle0.ndinfo.nPoints[0] = F16::alpha1_size;	/* Alpha npoints */
	fn_CzEle0.ndinfo.nPoints[1] = F16::beta1_size; /* Beta npoints  */
	fn_CzEle0.ndinfo.nPoints[2] = F16::dh1_size;  /* dele npoints  */
	fn_CzEle0.m_Xmat[0] = F16::alpha1;
	fn_CzEle0.m_Xmat[1] = F16::beta1;
	fn_CzEle0.m_Xmat[2] = F16::dh1;

	fn_Cm.init();
	fn_Cm.ndinfo.nPoints[0] = F16::alpha1_size;
	fn_Cm.ndinfo.nPoints[1] = F16::beta1_size;
	fn_Cm.ndinfo.nPoints[2] = F16::dh1_size;
	fn_Cm.m_Xmat[0] = F16::alpha1;
	fn_Cm.m_Xmat[1] = F16::beta1;
	fn_Cm.m_Xmat[2] = F16::dh1;

	fn_CmEle0.init();
	fn_CmEle0.ndinfo.nPoints[0] = F16::alpha1_size;
	fn_CmEle0.ndinfo.nPoints[1] = F16::beta1_size;
	fn_CmEle0.ndinfo.nPoints[2] = F16::dh1_size;
	fn_CmEle0.m_Xmat[0] = F16::alpha1;
	fn_CmEle0.m_Xmat[1] = F16::beta1;
	fn_CmEle0.m_Xmat[2] = F16::dh1;

	fn_Cy.init();
	fn_Cy.ndinfo.nPoints[0] = F16::alpha1_size;
	fn_Cy.ndinfo.nPoints[1] = F16::beta1_size;
	fn_Cy.m_Xmat[0] = F16::alpha1;
	fn_Cy.m_Xmat[1] = F16::beta1;

	fn_Cn.init();
	fn_Cn.ndinfo.nPoints[0] = F16::alpha1_size;
	fn_Cn.ndinfo.nPoints[1] = F16::beta1_size;
	fn_Cn.ndinfo.nPoints[2] = F16::dh2_size;
	fn_Cn.m_Xmat[0] = F16::alpha1;
	fn_Cn.m_Xmat[1] = F16::beta1;
	fn_Cn.m_Xmat[2] = F16::dh2;

	fn_CnEle0.init();
	fn_CnEle0.ndinfo.nPoints[0] = F16::alpha1_size;
	fn_CnEle0.ndinfo.nPoints[1] = F16::beta1_size;
	fn_CnEle0.ndinfo.nPoints[2] = F16::dh2_size;
	fn_CnEle0.m_Xmat[0] = F16::alpha1;
	fn_CnEle0.m_Xmat[1] = F16::beta1;
	fn_CnEle0.m_Xmat[2] = F16::dh2;

	fn_Cl.init();
	fn_Cl.ndinfo.nPoints[0] = F16::alpha1_size;
	fn_Cl.ndinfo.nPoints[1] = F16::beta1_size;
	fn_Cl.ndinfo.nPoints[2] = F16::dh2_size;
	fn_Cl.m_Xmat[0] = F16::alpha1;
	fn_Cl.m_Xmat[1] = F16::beta1;
	fn_Cl.m_Xmat[2] = F16::dh2;

	fn_ClEle0.init();
	fn_ClEle0.ndinfo.nPoints[0] = F16::alpha1_size;
	fn_ClEle0.ndinfo.nPoints[1] = F16::beta1_size;
	fn_ClEle0.ndinfo.nPoints[2] = F16::dh2_size;
	fn_ClEle0.m_Xmat[0] = F16::alpha1;
	fn_ClEle0.m_Xmat[1] = F16::beta1;
	fn_ClEle0.m_Xmat[2] = F16::dh2;

	fn_Cx_lef.init();
	fn_Cx_lef.m_xPar1Limit = 45.0;
	fn_Cx_lef.ndinfo.nPoints[0] = F16::alpha2_size;
	fn_Cx_lef.ndinfo.nPoints[1] = F16::beta1_size;
	fn_Cx_lef.m_Xmat[0] = F16::alpha2;
	fn_Cx_lef.m_Xmat[1] = F16::beta1;

	fn_Cz_lef.init();
	fn_Cz_lef.m_xPar1Limit = 45.0;
	fn_Cz_lef.ndinfo.nPoints[0] = F16::alpha2_size;
	fn_Cz_lef.ndinfo.nPoints[1] = F16::beta1_size;
	fn_Cz_lef.m_Xmat[0] = F16::alpha2;
	fn_Cz_lef.m_Xmat[1] = F16::beta1;

	fn_Cm_lef.init();
	fn_Cm_lef.m_xPar1Limit = 45.0;
	fn_Cm_lef.ndinfo.nPoints[0] = F16::alpha2_size;
	fn_Cm_lef.ndinfo.nPoints[1] = F16::beta1_size;
	fn_Cm_lef.m_Xmat[0] = F16::alpha2;
	fn_Cm_lef.m_Xmat[1] = F16::beta1;

	fn_Cy_lef.init();
	fn_Cy_lef.m_xPar1Limit = 45.0;
	fn_Cy_lef.ndinfo.nPoints[0] = F16::alpha2_size;
	fn_Cy_lef.ndinfo.nPoints[1] = F16::beta1_size;
	fn_Cy_lef.m_Xmat[0] = F16::alpha2;
	fn_Cy_lef.m_Xmat[1] = F16::beta1;

	fn_Cn_lef.init();
	fn_Cn_lef.m_xPar1Limit = 45.0;
	fn_Cn_lef.ndinfo.nPoints[0] = F16::alpha2_size;
	fn_Cn_lef.ndinfo.nPoints[1] = F16::beta1_size;
	fn_Cn_lef.m_Xmat[0] = F16::alpha2;
	fn_Cn_lef.m_Xmat[1] = F16::beta1;

	fn_Cl_lef.init(); /* alpha,beta*/
	fn_Cl_lef.m_xPar1Limit = 45.0;
	fn_Cl_lef.ndinfo.nPoints[0] = F16::alpha2_size;	/* Alpha npoints */
	fn_Cl_lef.ndinfo.nPoints[1] = F16::beta1_size; /* Beta npoints  */
	fn_Cl_lef.m_Xmat[0] = F16::alpha2;
	fn_Cl_lef.m_Xmat[1] = F16::beta1;

	fn_CXq.init();
	fn_CXq.ndinfo.nPoints[0] = F16::alpha1_size;
	fn_CXq.m_Xmat[0] = F16::alpha1;

	fn_CZq.init();
	fn_CZq.ndinfo.nPoints[0] = F16::alpha1_size;
	fn_CZq.m_Xmat[0] = F16::alpha1;

	fn_CMq.init();
	fn_CMq.ndinfo.nPoints[0] = F16::alpha1_size;
	fn_CMq.m_Xmat[0] = F16::alpha1;

	fn_CYp.init();
	fn_CYp.ndinfo.nPoints[0] = F16::alpha1_size;
	fn_CYp.m_Xmat[0] = F16::alpha1;

	fn_CYr.init();
	fn_CYr.ndinfo.nPoints[0] = F16::alpha1_size;
	fn_CYr.m_Xmat[0] = F16::alpha1;

	fn_CNr.init();
	fn_CNr.ndinfo.nPoints[0] = F16::alpha1_size;
	fn_CNr.m_Xmat[0] = F16::alpha1;

	fn_CNp.init();
	fn_CNp.ndinfo.nPoints[0] = F16::alpha1_size;
	fn_CNp.m_Xmat[0] = F16::alpha1;

	fn_CLp.init();
	fn_CLp.ndinfo.nPoints[0] = F16::alpha1_size;
	fn_CLp.m_Xmat[0] = F16::alpha1;

	fn_CLr.init();
	fn_CLr.ndinfo.nPoints[0] = F16::alpha1_size;
	fn_CLr.m_Xmat[0] = F16::alpha1;

	fn_delta_CXq_lef.init();
	fn_delta_CXq_lef.m_xPar1Limit = 45.0;
	fn_delta_CXq_lef.ndinfo.nPoints[0] = F16::alpha2_size;
	fn_delta_CXq_lef.m_Xmat[0] = F16::alpha2;

	fn_delta_CYr_lef.init();
	fn_delta_CYr_lef.m_xPar1Limit = 45.0;
	fn_delta_CYr_lef.ndinfo.nPoints[0] = F16::alpha2_size;
	fn_delta_CYr_lef.m_Xmat[0] = F16::alpha2;

	fn_delta_CYp_lef.init();
	fn_delta_CYp_lef.m_xPar1Limit = 45.0;
	fn_delta_CYp_lef.ndinfo.nPoints[0] = F16::alpha2_size;
	fn_delta_CYp_lef.m_Xmat[0] = F16::alpha2;

	fn_delta_CZq_lef.init();
	fn_delta_CZq_lef.m_xPar1Limit = 45.0;
	fn_delta_CZq_lef.ndinfo.nPoints[0] = F16::alpha2_size;
	fn_delta_CZq_lef.m_Xmat[0] = F16::alpha2;

	fn_delta_CLr_lef.init();
	fn_delta_CLr_lef.m_xPar1Limit = 45.0;
	fn_delta_CLr_lef.ndinfo.nPoints[0] = F16::alpha2_size;
	fn_delta_CLr_lef.m_Xmat[0] = F16::alpha2;

	fn_delta_CLp_lef.init();
	fn_delta_CLp_lef.m_xPar1Limit = 45.0;
	fn_delta_CLp_lef.ndinfo.nPoints[0] = F16::alpha2_size;
	fn_delta_CLp_lef.m_Xmat[0] = F16::alpha2;

	fn_delta_CMq_lef.init();
	fn_delta_CMq_lef.m_xPar1Limit = 45.0;
	fn_delta_CMq_lef.ndinfo.nPoints[0] = F16::alpha2_size;
	fn_delta_CMq_lef.m_Xmat[0] = F16::alpha2;

	fn_delta_CNr_lef.init();
	fn_delta_CNr_lef.m_xPar1Limit = 45.0;
	fn_delta_CNr_lef.ndinfo.nPoints[0] = F16::alpha2_size;
	fn_delta_CNr_lef.m_Xmat[0] = F16::alpha2;

	fn_delta_CNp_lef.init();
	fn_delta_CNp_lef.m_xPar1Limit = 45.0;
	fn_delta_CNp_lef.ndinfo.nPoints[0] = F16::alpha2_size;
	fn_delta_CNp_lef.m_Xmat[0] = F16::alpha2;

	fn_Cy_r30.init();
	fn_Cy_r30.ndinfo.nPoints[0] = F16::alpha1_size;
	fn_Cy_r30.ndinfo.nPoints[1] = F16::beta1_size;
	fn_Cy_r30.m_Xmat[0] = F16::alpha1;
	fn_Cy_r30.m_Xmat[1] = F16::beta1;

	fn_Cn_r30.init();
	fn_Cn_r30.ndinfo.nPoints[0] = F16::alpha1_size;
	fn_Cn_r30.ndinfo.nPoints[1] = F16::beta1_size;
	fn_Cn_r30.m_Xmat[0] = F16::alpha1;
	fn_Cn_r30.m_Xmat[1] = F16::beta1;

	fn_Cl_r30.init();
	fn_Cl_r30.ndinfo.nPoints[0] = F16::alpha1_size;
	fn_Cl_r30.ndinfo.nPoints[1] = F16::beta1_size;
	fn_Cl_r30.m_Xmat[0] = F16::alpha1;
	fn_Cl_r30.m_Xmat[1] = F16::beta1;

	fn_Cy_a20.init();
	fn_Cy_a20.ndinfo.nPoints[0] = F16::alpha1_size;
	fn_Cy_a20.ndinfo.nPoints[1] = F16::beta1_size;
	fn_Cy_a20.m_Xmat[0] = F16::alpha1;
	fn_Cy_a20.m_Xmat[1] = F16::beta1;

	fn_Cy_a20_lef.init();
	fn_Cy_a20_lef.m_xPar1Limit = 45.0;
	fn_Cy_a20_lef.ndinfo.nPoints[0] = F16::alpha2_size;
	fn_Cy_a20_lef.ndinfo.nPoints[1] = F16::beta1_size;
	fn_Cy_a20_lef.m_Xmat[0] = F16::alpha2;
	fn_Cy_a20_lef.m_Xmat[1] = F16::beta1;

	fn_Cn_a20.init();
	fn_Cn_a20.ndinfo.nPoints[0] = F16::alpha1_size;
	fn_Cn_a20.ndinfo.nPoints[1] = F16::beta1_size;
	fn_Cn_a20.m_Xmat[0] = F16::alpha1;
	fn_Cn_a20.m_Xmat[1] = F16::beta1;

	fn_Cn_a20_lef.init();
	fn_Cn_a20_lef.m_xPar1Limit = 45.0;
	fn_Cn_a20_lef.ndinfo.nPoints[0] = F16::alpha2_size;
	fn_Cn_a20_lef.ndinfo.nPoints[1] = F16::beta1_size;
	fn_Cn_a20_lef.m_Xmat[0] = F16::alpha2;
	fn_Cn_a20_lef.m_Xmat[1] = F16::beta1;

	fn_Cl_a20.init();
	fn_Cl_a20.ndinfo.nPoints[0] = F16::alpha1_size;
	fn_Cl_a20.ndinfo.nPoints[1] = F16::beta1_size;
	fn_Cl_a20.m_Xmat[0] = F16::alpha1;
	fn_Cl_a20.m_Xmat[1] = F16::beta1;

	fn_Cl_a20_lef.init();
	fn_Cl_a20_lef.m_xPar1Limit = 45.0;
	fn_Cl_a20_lef.ndinfo.nPoints[0] = F16::alpha2_size;
	fn_Cl_a20_lef.ndinfo.nPoints[1] = F16::beta1_size;
	fn_Cl_a20_lef.m_Xmat[0] = F16::alpha2;
	fn_Cl_a20_lef.m_Xmat[1] = F16::beta1;

	fn_delta_CNbeta.init();
	fn_delta_CNbeta.ndinfo.nPoints[0] = F16::alpha1_size;
	fn_delta_CNbeta.m_Xmat[0] = F16::alpha1;

	fn_delta_CLbeta.init();
	fn_delta_CLbeta.ndinfo.nPoints[0] = F16::alpha1_size;
	fn_delta_CLbeta.m_Xmat[0] = F16::alpha1;

	fn_delta_Cm.init();
	fn_delta_Cm.ndinfo.nPoints[0] = F16::alpha1_size;
	fn_delta_Cm.m_Xmat[0] = F16::alpha1;

	fn_eta_el.init();
	fn_eta_el.ndinfo.nPoints[0] = F16::dh1_size;
	fn_eta_el.m_Xmat[0] = F16::dh1;
} // F16Aero::initfn()

#endif // ifndef _F16AERO_H_
