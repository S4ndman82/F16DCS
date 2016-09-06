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


class F16Aero
{
protected:
	double		Cx_total;
	double		Cz_total;
	double		Cm_total;
	double		Cy_total;
	double		Cn_total;
	double		Cl_total;

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

public:
	F16Aero() :
		Cx_total(0),
		Cz_total(0),
		Cm_total(0),
		Cy_total(0),
		Cn_total(0),
		Cl_total(0),
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
		fn_eta_el(1, F16::_eta_elData)
	{
		initfn();
	}
	~F16Aero() {}
	void initfn();

	void updateFrame(const double alpha_DEG, const double beta_DEG, const double elevator_DEG, const double frameTime)
	{
		const double alpha = limit(alpha_DEG, -20.0, 90.0);
		const double beta = limit(beta_DEG, -30.0, 30.0);
		const double el = elevator_DEG;

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

	/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	compute Cx_tot, Cz_tot, Cm_tot, Cy_tot, Cn_tot, and Cl_total
	(as on NASA report p37-40)
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
	void computeTotals(const double AtmosTotalVelocity_FPS, 
					const double flap_PCT, const double leadingEdgeFlap_PCT, const double aileron_PCT, const double rudder_PCT,
					const double pitchRate_RPS, const double rollRate_RPS, const double yawRate_RPS, 
					const double alpha_DEG, const double beta_DEG, const double LgCxGearAero, const double LgCzGearAero,
					const double CxAirbrake)
	{
		// precalculate some terms to simplify statements
		const double totalVelocity_FPS = 2*AtmosTotalVelocity_FPS; // <- is this a bug?
		const double meanChordFPS = (F16::meanChord_FT / totalVelocity_FPS);
		const double wingSpanFPS = (F16::wingSpan_FT / totalVelocity_FPS);

		// TODO: dynamic CG to calculations, uses hardcoded "real" position now
		// (check: does this actually consider the RSS lift at non-CG position?)
		const double diffCgPCT = (F16::referenceCG_PCT - F16::actualCG_PCT);
		const double meanChordPerWingSpan = (F16::meanChord_FT / F16::wingSpan_FT);

		// calculate values based on interpolation results
		const double Cx_delta_lef = fn_Cx_lef.m_result - fn_CxEle0.m_result;
		const double Cz_delta_lef = fn_Cz_lef.m_result - fn_CzEle0.m_result;
		const double Cm_delta_lef = fn_Cm_lef.m_result - fn_CmEle0.m_result;
		const double Cy_delta_lef = fn_Cy_lef.m_result - fn_Cy.m_result;
		const double Cn_delta_lef = fn_Cn_lef.m_result - fn_CnEle0.m_result;
		const double Cl_delta_lef = fn_Cl_lef.m_result - fn_ClEle0.m_result;

		const double Cy_delta_r30 = fn_Cy_r30.m_result - fn_Cy.m_result;
		const double Cn_delta_r30 = fn_Cn_r30.m_result - fn_CnEle0.m_result;
		const double Cl_delta_r30 = fn_Cl_r30.m_result - fn_ClEle0.m_result;

		const double Cy_delta_a20 = fn_Cy_a20.m_result - fn_Cy.m_result;
		const double Cy_delta_a20_lef = fn_Cy_a20_lef.m_result - fn_Cy_lef.m_result - Cy_delta_a20;
		const double Cn_delta_a20 = fn_Cn_a20.m_result - fn_CnEle0.m_result;
		const double Cn_delta_a20_lef = fn_Cn_a20_lef.m_result - fn_Cn_lef.m_result - Cn_delta_a20;
		const double Cl_delta_a20 = fn_Cl_a20.m_result - fn_ClEle0.m_result;
		const double Cl_delta_a20_lef = fn_Cl_a20_lef.m_result - fn_Cl_lef.m_result - Cl_delta_a20;

		// FLAPS (From JBSim F16.xml config)
		double CLFlaps = 0.35 * flap_PCT;
		double CDFlaps = 0.08 * flap_PCT;
		double CzFlaps = - (CLFlaps * cos(alpha_DEG * F16::degtorad) + CDFlaps * sin(F16::degtorad));
		double CxFlaps = - (-CLFlaps * sin(alpha_DEG * F16::degtorad) + CDFlaps * cos(F16::degtorad));

		/* XXXXXXXX Cx_tot XXXXXXXX */
		double dXdQ = meanChordFPS * (fn_CXq.m_result + fn_delta_CXq_lef.m_result*leadingEdgeFlap_PCT);
		Cx_total = fn_Cx.m_result + Cx_delta_lef*leadingEdgeFlap_PCT + dXdQ*pitchRate_RPS;
		Cx_total += CxFlaps + LgCxGearAero;

		/* airbrake - testing now*/
		Cx_total += CxAirbrake;

		/* ZZZZZZZZ Cz_tot ZZZZZZZZ */ 
		double dZdQ = meanChordFPS * (fn_CZq.m_result + Cz_delta_lef*leadingEdgeFlap_PCT);
		Cz_total = fn_Cz.m_result + Cz_delta_lef*leadingEdgeFlap_PCT + dZdQ*pitchRate_RPS;
		Cz_total += CzFlaps + LgCzGearAero;

		/* MMMMMMMM Cm_tot MMMMMMMM */ 
		/* ignore deep-stall regime, delta_Cm_ds = 0 */
		double dMdQ = meanChordFPS * (fn_CMq.m_result + fn_delta_CMq_lef.m_result*leadingEdgeFlap_PCT);
		double CmDelta = fn_delta_Cm.m_result + 0; // Cm_delta + Cm_delta_ds (0)
		Cm_total = fn_Cm.m_result*fn_eta_el.m_result + Cz_total*diffCgPCT + Cm_delta_lef*leadingEdgeFlap_PCT + dMdQ*pitchRate_RPS + CmDelta;

		/* YYYYYYYY Cy_tot YYYYYYYY */
		double dYdail = Cy_delta_a20 + Cy_delta_a20_lef*leadingEdgeFlap_PCT;
		double dYdR = wingSpanFPS * (fn_CYr.m_result + fn_delta_CYr_lef.m_result*leadingEdgeFlap_PCT);
		double dYdP = wingSpanFPS * (fn_CYp.m_result + fn_delta_CYp_lef.m_result*leadingEdgeFlap_PCT);
		Cy_total = fn_Cy.m_result + Cy_delta_lef*leadingEdgeFlap_PCT + dYdail*aileron_PCT + Cy_delta_r30*rudder_PCT + dYdR*yawRate_RPS + dYdP*rollRate_RPS;
	
		/* NNNNNNNN Cn_tot NNNNNNNN */ 
		double dNdail = Cn_delta_a20 + Cn_delta_a20_lef*leadingEdgeFlap_PCT;
		double dNdR = wingSpanFPS * (fn_CNr.m_result + fn_delta_CNr_lef.m_result*leadingEdgeFlap_PCT);
		double dNdP = wingSpanFPS * (fn_CNp.m_result + fn_delta_CNp_lef.m_result*leadingEdgeFlap_PCT);
		double CnDeltaBetaDeg = fn_delta_CNbeta.m_result*beta_DEG;
		Cn_total = fn_Cn.m_result + Cn_delta_lef*leadingEdgeFlap_PCT - Cy_total*diffCgPCT*meanChordPerWingSpan + dNdail*aileron_PCT + Cn_delta_r30*rudder_PCT + dNdR*yawRate_RPS + dNdP*rollRate_RPS + CnDeltaBetaDeg;

		/* LLLLLLLL Cl_total LLLLLLLL */
		double dLdail = Cl_delta_a20 + Cl_delta_a20_lef*leadingEdgeFlap_PCT;
		double dLdR = wingSpanFPS * (fn_CLr.m_result + fn_delta_CLr_lef.m_result*leadingEdgeFlap_PCT);
		double dLdP = wingSpanFPS * (fn_CLp.m_result + fn_delta_CLp_lef.m_result*leadingEdgeFlap_PCT);
		double ClDeltaBetaDeg = fn_delta_CLbeta.m_result*beta_DEG;
		Cl_total = fn_Cl.m_result + Cl_delta_lef*leadingEdgeFlap_PCT + dLdail*aileron_PCT + Cl_delta_r30*rudder_PCT + dLdR*yawRate_RPS + dLdP*rollRate_RPS + ClDeltaBetaDeg;
	}


	double getCxTotal() const { return Cx_total; }
	double getCzTotal() const { return Cz_total; }
	double getCmTotal() const { return Cm_total; }
	double getCyTotal() const { return Cy_total; }
	double getCnTotal() const { return Cn_total; }
	double getClTotal() const { return Cl_total; }

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
