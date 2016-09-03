// Source Data:
// 1) F-16 University of Minnesota Non-Linear Flight Model
//	  http://www.aem.umn.edu/people/faculty/balas/darpa_sec/SEC.Software.html
// 2) NASA TP 1538 Simulator Study of Stall/Post-Stall Characteristics of a 
//	  Fighter Airplane With Relaxed Longitudinal Static Stability

#ifndef _F16AERO_H_
#define _F16AERO_H_

#include "../stdafx.h"
#include "F16AeroData.h"
#include "../UtilityFunctions.h"

namespace F16
{
	class AERO_Function
	{
	protected:

		// stuff only for temporary use,
		// avoid reallocations
		UtilBuffer<int> indexVector;

	public:
		ND_INFO ndinfo; // dimensions descriptor

		double **m_Xmat; // pointers to static arrays of data (X matrix)
		double *m_Ydata; // pointer to static array of related data (Y)

		UtilBuffer<double> m_Tbuf; // reusable buffer to reduce malloc()/free()

		UtilMatrix<double> m_xPointMat; // used in interpolation, reduce reallocation
		UtilMatrix<int> m_indexMat; // used in interpolation, reduce reallocation

		//double *xPar; // parameters for interpolation (1-3 pars)
		double m_xPar1Limit; // upper limit for X-parameter 1 in functions (only upper and only for this)

		double m_result; // result value

		AERO_Function(const int nDimension, double *Ydata)
			: indexVector()
			, ndinfo()
			, m_Xmat(NULL)
			, m_Ydata(Ydata)
			, m_Tbuf()
			, m_xPointMat()
			, m_indexMat()
			, m_xPar1Limit(0)
			, m_result(0)
		{
			ndinfo.nPoints = NULL;
			ndinfo.nDimension = nDimension;
		}

		~AERO_Function()
		{
			m_indexMat.release();
			m_xPointMat.release();
			if (ndinfo.nPoints != NULL)
			{
				free(ndinfo.nPoints);
				ndinfo.nPoints = NULL;
			}
			if (m_Xmat != NULL)
			{
				free(m_Xmat);
				m_Xmat = NULL;
			}
			indexVector.release();
		}

		void init()
		{
			ndinfo.nPoints = (int*)malloc(ndinfo.nDimension*sizeof(int));

			// just array of pointers to static data
			m_Xmat = (double **)malloc(ndinfo.nDimension*sizeof(double*));

			int nVertices = (1 << ndinfo.nDimension);
			m_Tbuf.getVec(nVertices); // preallocate

			// preallocate
			m_xPointMat.allocate(ndinfo.nDimension, 2);
			m_indexMat.allocate(ndinfo.nDimension, 2);

			// preallocate another temporary buffer to reuse
			indexVector.getVec(ndinfo.nDimension);
		}


		/*
		void setDatapoint(const int index, int size, )
		{}
		*/

		double interpnf1Lim(const double xPar1)
		{
			if (xPar1 > m_xPar1Limit)
			{
				// use limit
				return interpnf1(m_xPar1Limit);
			}
			return interpnf1(xPar1);
		}
		double interpnf2Lim(const double xPar1, const double xPar2)
		{
			if (xPar1 > m_xPar1Limit)
			{
				// use limit
				return interpnf2(m_xPar1Limit, xPar2);
			}
			return interpnf2(xPar1, xPar2);
		}
		double interpnf3Lim(const double xPar1, const double xPar2, const double xPar3)
		{
			if (xPar1 > m_xPar1Limit)
			{
				// use limit
				return interpnf3(m_xPar1Limit, xPar2, xPar3);
			}
			return interpnf3(xPar1, xPar2, xPar3);
		}

		double interpnf1(const double xPar1)
		{
			double x[1];
			x[0] = xPar1;
			return interpnf(x);
		}

		double interpnf2(const double xPar1, const double xPar2)
		{
			double x[2];
			x[0] = xPar1;
			x[1] = xPar2;
			return interpnf(x);
		}

		double interpnf3(const double xPar1, const double xPar2, const double xPar3)
		{
			double x[3];
			x[0] = xPar1;
			x[1] = xPar2;
			x[2] = xPar3;
			return interpnf(x);
		}

		double interpnf(const double *xPar)
		{
			m_result = interpn(indexVector, m_Xmat, m_Ydata, xPar, m_xPointMat, m_indexMat, ndinfo, m_Tbuf);
			return m_result;
		}
	};


	// this is temporary while sorting out the stuff in namespace..
	class F16Aero
	{
	protected:
		double		Cx_total;
		double		Cx;
		double		Cx_delta_lef;
		double		Cxq_delta_lef;
		double		Cz_total;
		double		Cz;
		double		Cz_delta_lef;
		double		Czq_delta_lef;
		double		Cm_total;
		double		Cm;
		double		Cm_delta_lef;
		double		Cmq_delta_lef;
		double		Cy_total;
		double		Cy;
		double		Cy_delta_lef;
		double		Cy_delta_r30;
		double		Cy_delta_a20;
		double		Cy_delta_a20_lef;
		double		Cyr_delta_lef;
		double		Cyp_delta_lef;
		double		Cn_total;
		double		Cn;
		double		Cn_delta_lef;
		double		Cn_delta_r30;
		double		Cn_delta_a20;
		double		Cn_delta_a20_lef;
		double		Cnr_delta_lef;
		double		Cnp_delta_lef;
		double		Cl_total;
		double		Cl;
		double		Cl_delta_lef;
		double		Cl_delta_r30;
		double		Cl_delta_a20;
		double		Cl_delta_a20_lef;
		double		Clr_delta_lef;
		double		Clp_delta_lef;

		AERO_Function fn_Cx;
		AERO_Function fn_Cz;
		AERO_Function fn_Cm;
		AERO_Function fn_Cy;
		AERO_Function fn_Cn;
		AERO_Function fn_Cl;
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

		double _Cx(double alpha,double beta,double dele)
		{
			//CX0120_ALPHA1_BETA1_DH1_201.dat
			return fn_Cx.interpnf3(alpha, beta, dele);
		}

		double _Cz(double alpha,double beta, double dele)
		{
			//CZ0120_ALPHA1_BETA1_DH1_301.dat
			return fn_Cz.interpnf3(alpha, beta, dele);
		}

		double _Cm(double alpha,double beta,double dele)
		{
			//CM0120_ALPHA1_BETA1_DH1_101.dat
			return fn_Cm.interpnf3(alpha, beta, dele);
		}

		double _Cy(double alpha,double beta)
		{
			// CY0320_ALPHA1_BETA1_401.dat
			return fn_Cy.interpnf2(alpha, beta);
		}

		double _Cn(double alpha, double beta, double dele)
		{
			//CN0120_ALPHA1_BETA1_DH2_501.dat
			return fn_Cn.interpnf3(alpha, beta, dele);
		}

		double _Cl(double alpha, double beta,double dele)
		{
			return fn_Cl.interpnf3(alpha, beta, dele);
		}

		double _Cx_lef(double alpha,double beta)
		{
			//CX0820_ALPHA2_BETA1_202.dat
			return fn_Cx_lef.interpnf2Lim(alpha, beta);
		}

		double _Cz_lef(double alpha,double beta)
		{
			//CZ0820_ALPHA2_BETA1_302.dat
			return fn_Cz_lef.interpnf2Lim(alpha, beta);
		}

		double _Cm_lef(double alpha,double beta)
		{
			//CM0820_ALPHA2_BETA1_102.dat

			return fn_Cm_lef.interpnf2Lim(alpha, beta);
		}

		double _delta_CXq_lef(double alpha)
		{
			//CX1420_ALPHA2_205.dat

			return fn_delta_CXq_lef.interpnf1Lim(alpha);
		}

		double _delta_CYr_lef(double alpha)
		{
			//CY1620_ALPHA2_407.dat

			return fn_delta_CYr_lef.interpnf1Lim(alpha);
		}

		double _delta_CYp_lef(double alpha)
		{
			//CY1520_ALPHA2_409.dat
			return fn_delta_CYp_lef.interpnf1Lim(alpha);
		}

		double _delta_CZq_lef(double alpha)
		{
			//CZ1420_ALPHA2_305.dat

			return fn_delta_CZq_lef.interpnf1Lim(alpha);
		}

		double _delta_CLr_lef(double alpha)
		{
			//CL1620_ALPHA2_607.dat

			return fn_delta_CLr_lef.interpnf1Lim(alpha);
		}

		double _delta_CLp_lef(double alpha)
		{
			//CL1520_ALPHA2_609.dat

			return fn_delta_CLp_lef.interpnf1Lim(alpha);
		}

		double _delta_CMq_lef(double alpha)
		{
			//CM1420_ALPHA2_105.dat

			return fn_delta_CMq_lef.interpnf1Lim(alpha);
		}

		double _delta_CNr_lef(double alpha)
		{
			//CN1620_ALPHA2_507.dat

			return fn_delta_CNr_lef.interpnf1Lim(alpha);
		}

		double _delta_CNp_lef(double alpha)
		{
			//CN1520_ALPHA2_509.dat

			return fn_delta_CNp_lef.interpnf1Lim(alpha);
		}



		/*
		double _delta_Cm_ds(double alpha, double el){
		...............
		...............
		} End of function(...) */


	public:
		F16Aero();
		~F16Aero();
		void initfn();

		void updateFrame(const double alpha_DEG, const double beta_DEG, const double elevator_DEG, const double frameTime)
		{
			const double alpha = limit(alpha_DEG, -20.0, 90.0);
			const double beta = limit(beta_DEG, -30.0, 30.0);
			const double el = elevator_DEG;

			// TODO: speed brake handling..
			// TODO Speedbrakes aero (from JBSim F16.xml config)

			/* hifi_C */
			Cx = _Cx(alpha, beta, el);
			Cz = _Cz(alpha, beta, el);
			Cm = _Cm(alpha, beta, el);
			Cy = _Cy(alpha, beta);
			Cn = _Cn(alpha, beta, el);
			Cl = _Cl(alpha, beta, el);

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

			fn_Cy_lef.interpnf2Lim(alpha, beta); //CY0820_ALPHA2_BETA1_402.dat
			fn_Cn_lef.interpnf2Lim(alpha, beta); //CN0820_ALPHA2_BETA1_502.dat
			fn_Cl_lef.interpnf2Lim(alpha, beta);


			/* hifi_C_lef */
			Cx_delta_lef = _Cx_lef(alpha, beta) - _Cx(alpha, beta, 0);
			Cz_delta_lef = _Cz_lef(alpha, beta) - _Cz(alpha, beta, 0);
			Cm_delta_lef = _Cm_lef(alpha, beta) - _Cm(alpha, beta, 0);
			Cy_delta_lef = fn_Cy_lef.m_result - _Cy(alpha, beta);
			Cn_delta_lef = fn_Cn_lef.m_result - _Cn(alpha, beta, 0);
			Cl_delta_lef = fn_Cl_lef.m_result - _Cl(alpha, beta, 0);

			/* hifi_damping_lef */
			Cxq_delta_lef = _delta_CXq_lef(alpha);
			Cyr_delta_lef = _delta_CYr_lef(alpha);
			Cyp_delta_lef = _delta_CYp_lef(alpha);
			Czq_delta_lef = _delta_CZq_lef(alpha);
			Clr_delta_lef = _delta_CLr_lef(alpha);
			Clp_delta_lef = _delta_CLp_lef(alpha);
			Cmq_delta_lef = _delta_CMq_lef(alpha);
			Cnr_delta_lef = _delta_CNr_lef(alpha);
			Cnp_delta_lef = _delta_CNp_lef(alpha);

			/* hifi_rudder */
			//CY0720_ALPHA1_BETA1_405.dat
			fn_Cy_r30.interpnf2(alpha, beta);
			//CN0720_ALPHA1_BETA1_503.dat
			fn_Cn_r30.interpnf2(alpha, beta);
			//CL0720_ALPHA1_BETA1_603.dat
			fn_Cl_r30.interpnf2(alpha, beta);

			Cy_delta_r30 = fn_Cy_r30.m_result - _Cy(alpha, beta);
			Cn_delta_r30 = fn_Cn_r30.m_result - _Cn(alpha, beta, 0);
			Cl_delta_r30 = fn_Cl_r30.m_result - _Cl(alpha, beta, 0);

			/* hifi_ailerons */
			fn_Cy_a20.interpnf2(alpha, beta); //CY0620_ALPHA1_BETA1_403.dat
			fn_Cn_a20.interpnf2(alpha, beta); //CN0620_ALPHA1_BETA1_504.dat
			fn_Cl_a20.interpnf2(alpha, beta); //CL0620_ALPHA1_BETA1_604.dat

			fn_Cy_a20_lef.interpnf2Lim(alpha, beta); //CY0920_ALPHA2_BETA1_404.dat
			fn_Cn_a20_lef.interpnf2Lim(alpha, beta); //CN0920_ALPHA2_BETA1_505.dat
			fn_Cl_a20_lef.interpnf2Lim(alpha, beta); //CL0920_ALPHA2_BETA1_605.dat

			Cy_delta_a20 = fn_Cy_a20.m_result - _Cy(alpha, beta);
			Cy_delta_a20_lef = fn_Cy_a20_lef.m_result - fn_Cy_lef.m_result - Cy_delta_a20;
			Cn_delta_a20 = fn_Cn_a20.m_result - _Cn(alpha, beta, 0);
			Cn_delta_a20_lef = fn_Cn_a20_lef.m_result - fn_Cn_lef.m_result - Cn_delta_a20;
			Cl_delta_a20 = fn_Cl_a20.m_result - _Cl(alpha, beta, 0);
			Cl_delta_a20_lef = fn_Cl_a20_lef.m_result - fn_Cl_lef.m_result - Cl_delta_a20;

			/* hifi_other_coeffs */

			//CN9999_ALPHA1_brett.dat
			fn_delta_CNbeta.interpnf1(alpha);

			//CL9999_ALPHA1_brett.dat
			fn_delta_CLbeta.interpnf1(alpha);

			//CM9999_ALPHA1_brett.dat
			fn_delta_Cm.interpnf1(alpha);

			//ETA_DH1_brett.dat
			fn_eta_el.interpnf1(el);

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

			const double diffCgPCT = (F16::referenceCG_PCT - F16::actualCG_PCT);
			const double meanChordPerWingSpan = (F16::meanChord_FT / F16::wingSpan_FT);

			// FLAPS (From JBSim F16.xml config)
			double CLFlaps = 0.35 * flap_PCT;
			double CDFlaps = 0.08 * flap_PCT;
			double CzFlaps = - (CLFlaps * cos(alpha_DEG * F16::degtorad) + CDFlaps * sin(F16::degtorad));
			double CxFlaps = - (-CLFlaps * sin(alpha_DEG * F16::degtorad) + CDFlaps * cos(F16::degtorad));

			/* XXXXXXXX Cx_tot XXXXXXXX */
			double dXdQ = meanChordFPS * (fn_CXq.m_result + Cxq_delta_lef*leadingEdgeFlap_PCT);
			Cx_total = Cx + Cx_delta_lef*leadingEdgeFlap_PCT + dXdQ*pitchRate_RPS;
			Cx_total += CxFlaps + LgCxGearAero;

			/* airbrake - testing now*/
			Cx_total += CxAirbrake;

			/* ZZZZZZZZ Cz_tot ZZZZZZZZ */ 
			double dZdQ = meanChordFPS * (fn_CZq.m_result + Cz_delta_lef*leadingEdgeFlap_PCT);
			Cz_total = Cz + Cz_delta_lef*leadingEdgeFlap_PCT + dZdQ*pitchRate_RPS;
			Cz_total += CzFlaps + LgCzGearAero;

			/* MMMMMMMM Cm_tot MMMMMMMM */ 
			/* ignore deep-stall regime, delta_Cm_ds = 0 */
			double dMdQ = meanChordFPS * (fn_CMq.m_result + Cmq_delta_lef*leadingEdgeFlap_PCT);
			double CmDelta = fn_delta_Cm.m_result + 0; // Cm_delta + Cm_delta_ds (0)
			Cm_total = Cm*fn_eta_el.m_result + Cz_total*diffCgPCT + Cm_delta_lef*leadingEdgeFlap_PCT + dMdQ*pitchRate_RPS + CmDelta;

			/* YYYYYYYY Cy_tot YYYYYYYY */
			double dYdail = Cy_delta_a20 + Cy_delta_a20_lef*leadingEdgeFlap_PCT;
			double dYdR = wingSpanFPS * (fn_CYr.m_result + Cyr_delta_lef*leadingEdgeFlap_PCT);
			double dYdP = wingSpanFPS * (fn_CYp.m_result + Cyp_delta_lef*leadingEdgeFlap_PCT);
			Cy_total = Cy + Cy_delta_lef*leadingEdgeFlap_PCT + dYdail*aileron_PCT + Cy_delta_r30*rudder_PCT + dYdR*yawRate_RPS + dYdP*rollRate_RPS;
	
			/* NNNNNNNN Cn_tot NNNNNNNN */ 
			double dNdail = Cn_delta_a20 + Cn_delta_a20_lef*leadingEdgeFlap_PCT;
			double dNdR = wingSpanFPS * (fn_CNr.m_result + Cnr_delta_lef*leadingEdgeFlap_PCT);
			double dNdP = wingSpanFPS * (fn_CNp.m_result + Cnp_delta_lef*leadingEdgeFlap_PCT);
			double CnDeltaBetaDeg = fn_delta_CNbeta.m_result*beta_DEG;
			Cn_total = Cn + Cn_delta_lef*leadingEdgeFlap_PCT - Cy_total*diffCgPCT*meanChordPerWingSpan + dNdail*aileron_PCT + Cn_delta_r30*rudder_PCT + dNdR*yawRate_RPS + dNdP*rollRate_RPS + CnDeltaBetaDeg;

			/* LLLLLLLL Cl_total LLLLLLLL */
			double dLdail = Cl_delta_a20 + Cl_delta_a20_lef*leadingEdgeFlap_PCT;
			double dLdR = wingSpanFPS * (fn_CLr.m_result + Clr_delta_lef*leadingEdgeFlap_PCT);
			double dLdP = wingSpanFPS * (fn_CLp.m_result + Clp_delta_lef*leadingEdgeFlap_PCT);
			double ClDeltaBetaDeg = fn_delta_CLbeta.m_result*beta_DEG;
			Cl_total = Cl + Cl_delta_lef*leadingEdgeFlap_PCT + dLdail*aileron_PCT + Cl_delta_r30*rudder_PCT + dLdR*yawRate_RPS + dLdP*rollRate_RPS + ClDeltaBetaDeg;
		}


		double getCxTotal() const { return Cx_total; }
		double getCzTotal() const { return Cz_total; }
		double getCmTotal() const { return Cm_total; }
		double getCyTotal() const { return Cy_total; }
		double getCnTotal() const { return Cn_total; }
		double getClTotal() const { return Cl_total; }

	}; // class F16Aero

	// constructor
	F16Aero::F16Aero() :
		Cx_total(0),
		Cx(0),				
		Cx_delta_lef(0),	
		Cxq_delta_lef(0),	
		Cz_total(0),		
		Cz(0),				
		Cz_delta_lef(0),	
		Czq_delta_lef(0),	
		Cm_total(0),		
		Cm(0),				
		Cm_delta_lef(0),	
		Cmq_delta_lef(0),	
		Cy_total(0),		
		Cy(0),				
		Cy_delta_lef(0),	
		Cy_delta_r30(0),	
		Cy_delta_a20(0),	
		Cy_delta_a20_lef(0),
		Cyr_delta_lef(0),	
		Cyp_delta_lef(0),	
		Cn_total(0),		
		Cn(0),				
		Cn_delta_lef(0),	
		Cn_delta_r30(0),	
		Cn_delta_a20(0),	
		Cn_delta_a20_lef(0),
		Cnr_delta_lef(0),	
		Cnp_delta_lef(0),	
		Cl_total(0),		
		Cl(0),				
		Cl_delta_lef(0),	
		Cl_delta_r30(0),	
		Cl_delta_a20(0),	
		Cl_delta_a20_lef(0),
		Clr_delta_lef(0),	
		Clp_delta_lef(0),
		fn_Cx(3, _CxData),
		fn_Cz(3, _CzData),
		fn_Cm(3, _CmData),
		fn_Cy(2, _CyData),
		fn_Cn(3, _CnData),
		fn_Cl(3, _ClData),
		fn_Cx_lef(2, _Cx_lefData),
		fn_Cz_lef(2, _Cz_lefData),
		fn_Cm_lef(2, _Cm_lefData),
		fn_Cy_lef(2, _Cy_lefData),
		fn_Cn_lef(2, _Cn_lefData),
		fn_Cl_lef(2, _Cl_lefData),
		fn_CXq(1, _CxqData),
		fn_CZq(1, _CzqData),
		fn_CMq(1, _CmqData),
		fn_CYp(1, _CypData),
		fn_CYr(1, _CyrData),
		fn_CNr(1, _CnrData),
		fn_CNp(1, _CnpData),
		fn_CLp(1, _ClpData),
		fn_CLr(1, _ClrData),
		fn_delta_CXq_lef(1, _delta_CXq_lefData),
		fn_delta_CYr_lef(1, _delta_CYr_lefData),
		fn_delta_CYp_lef(1, _delta_CYp_lefData),
		fn_delta_CZq_lef(1, _delta_CZq_lefData),
		fn_delta_CLr_lef(1, _delta_CLr_lefData),
		fn_delta_CLp_lef(1, _delta_CLp_lefData),
		fn_delta_CMq_lef(1, _delta_CMq_lefData),
		fn_delta_CNr_lef(1, _delta_CNr_lefData),
		fn_delta_CNp_lef(1, _delta_CNp_lefData),
		fn_Cy_r30(2, _Cy_r30Data),
		fn_Cn_r30(2, _Cn_r30Data),
		fn_Cl_r30(2, _Cl_r30Data),
		fn_Cy_a20(2, _Cy_a20Data),
		fn_Cy_a20_lef(2, _Cy_a20_lefData),
		fn_Cn_a20(2, _Cn_a20Data),
		fn_Cn_a20_lef(2, _Cn_a20_lefData),
		fn_Cl_a20(2, _Cl_a20Data),
		fn_Cl_a20_lef(2, _Cl_a20_lefData),
		fn_delta_CNbeta(1, _delta_CNbetaData),
		fn_delta_CLbeta(1, _delta_CLbetaData),
		fn_delta_Cm(1, _delta_CmData),
		fn_eta_el(1, _eta_elData)
	{
		initfn();
	}

	// destructor
	F16Aero::~F16Aero()
	{}

	void F16Aero::initfn()
	{
		fn_Cx.init();
		fn_Cx.ndinfo.nPoints[0] = alpha1_size;	
		fn_Cx.ndinfo.nPoints[1] = beta1_size; 
		fn_Cx.ndinfo.nPoints[2] = dh1_size; 
		fn_Cx.m_Xmat[0] = alpha1;
		fn_Cx.m_Xmat[1] = beta1;
		fn_Cx.m_Xmat[2] = dh1;

		fn_Cz.init(); /* alpha,beta,dele */
		fn_Cz.ndinfo.nPoints[0] = alpha1_size;	/* Alpha npoints */
		fn_Cz.ndinfo.nPoints[1] = beta1_size; /* Beta npoints  */
		fn_Cz.ndinfo.nPoints[2] = dh1_size;  /* dele npoints  */
		fn_Cz.m_Xmat[0] = alpha1;
		fn_Cz.m_Xmat[1] = beta1;
		fn_Cz.m_Xmat[2] = dh1;

		fn_Cm.init();
		fn_Cm.ndinfo.nPoints[0] = alpha1_size;	
		fn_Cm.ndinfo.nPoints[1] = beta1_size; 
		fn_Cm.ndinfo.nPoints[2] = dh1_size; 
		fn_Cm.m_Xmat[0] = alpha1;
		fn_Cm.m_Xmat[1] = beta1;
		fn_Cm.m_Xmat[2] = dh1;

		fn_Cy.init();
		fn_Cy.ndinfo.nPoints[0] = alpha1_size;	
		fn_Cy.ndinfo.nPoints[1] = beta1_size; 
		fn_Cy.m_Xmat[0] = alpha1;
		fn_Cy.m_Xmat[1] = beta1;

		fn_Cn.init();
		fn_Cn.ndinfo.nPoints[0] = alpha1_size;	
		fn_Cn.ndinfo.nPoints[1] = beta1_size;	
		fn_Cn.ndinfo.nPoints[2] = dh2_size;
		fn_Cn.m_Xmat[0] = alpha1;
		fn_Cn.m_Xmat[1] = beta1;
		fn_Cn.m_Xmat[2] = dh2;

		fn_Cl.init();
		fn_Cl.ndinfo.nPoints[0] = alpha1_size;	
		fn_Cl.ndinfo.nPoints[1] = beta1_size;
		fn_Cl.ndinfo.nPoints[2] = dh2_size;
		fn_Cl.m_Xmat[0] = alpha1;
		fn_Cl.m_Xmat[1] = beta1;
		fn_Cl.m_Xmat[2] = dh2;

		fn_Cx_lef.init();
		fn_Cx_lef.m_xPar1Limit = 45.0;
		fn_Cx_lef.ndinfo.nPoints[0] = alpha2_size;
		fn_Cx_lef.ndinfo.nPoints[1] = beta1_size;
		fn_Cx_lef.m_Xmat[0] = alpha2;
		fn_Cx_lef.m_Xmat[1] = beta1;

		fn_Cz_lef.init();
		fn_Cz_lef.m_xPar1Limit = 45.0;
		fn_Cz_lef.ndinfo.nPoints[0] = alpha2_size;
		fn_Cz_lef.ndinfo.nPoints[1] = beta1_size; 
		fn_Cz_lef.m_Xmat[0] = alpha2;
		fn_Cz_lef.m_Xmat[1] = beta1;

		fn_Cm_lef.init();
		fn_Cm_lef.m_xPar1Limit = 45.0;
		fn_Cm_lef.ndinfo.nPoints[0] = alpha2_size;
		fn_Cm_lef.ndinfo.nPoints[1] = beta1_size; 
		fn_Cm_lef.m_Xmat[0] = alpha2;
		fn_Cm_lef.m_Xmat[1] = beta1;

		fn_Cy_lef.init();
		fn_Cy_lef.m_xPar1Limit = 45.0;
		fn_Cy_lef.ndinfo.nPoints[0] = alpha2_size;
		fn_Cy_lef.ndinfo.nPoints[1] = beta1_size; 
		fn_Cy_lef.m_Xmat[0] = alpha2;
		fn_Cy_lef.m_Xmat[1] = beta1;

		fn_Cn_lef.init();
		fn_Cn_lef.m_xPar1Limit = 45.0;
		fn_Cn_lef.ndinfo.nPoints[0] = alpha2_size;
		fn_Cn_lef.ndinfo.nPoints[1] = beta1_size; 
		fn_Cn_lef.m_Xmat[0] = alpha2;
		fn_Cn_lef.m_Xmat[1] = beta1;

		fn_Cl_lef.init(); /* alpha,beta*/
		fn_Cl_lef.m_xPar1Limit = 45.0;
		fn_Cl_lef.ndinfo.nPoints[0] = alpha2_size;	/* Alpha npoints */
		fn_Cl_lef.ndinfo.nPoints[1] = beta1_size; /* Beta npoints  */
		fn_Cl_lef.m_Xmat[0] = alpha2;
		fn_Cl_lef.m_Xmat[1] = beta1;

		fn_CXq.init();
		fn_CXq.ndinfo.nPoints[0] = alpha1_size;	
		fn_CXq.m_Xmat[0] = alpha1;

		fn_CZq.init();
		fn_CZq.ndinfo.nPoints[0] = alpha1_size;	
		fn_CZq.m_Xmat[0] = alpha1;

		fn_CMq.init();
		fn_CMq.ndinfo.nPoints[0] = alpha1_size;	
		fn_CMq.m_Xmat[0] = alpha1;

		fn_CYp.init();
		fn_CYp.ndinfo.nPoints[0] = alpha1_size;	
		fn_CYp.m_Xmat[0] = alpha1;

		fn_CYr.init();
		fn_CYr.ndinfo.nPoints[0] = alpha1_size;	
		fn_CYr.m_Xmat[0] = alpha1;

		fn_CNr.init();
		fn_CNr.ndinfo.nPoints[0] = alpha1_size;	
		fn_CNr.m_Xmat[0] = alpha1;

		fn_CNp.init();
		fn_CNp.ndinfo.nPoints[0] = alpha1_size;	
		fn_CNp.m_Xmat[0] = alpha1;

		fn_CLp.init();
		fn_CLp.ndinfo.nPoints[0] = alpha1_size;	
		fn_CLp.m_Xmat[0] = alpha1;

		fn_CLr.init();
		fn_CLr.ndinfo.nPoints[0] = alpha1_size;	
		fn_CLr.m_Xmat[0] = alpha1;

		fn_delta_CXq_lef.init();
		fn_delta_CXq_lef.m_xPar1Limit = 45.0;
		fn_delta_CXq_lef.ndinfo.nPoints[0] = alpha2_size;
		fn_delta_CXq_lef.m_Xmat[0] = alpha2;

		fn_delta_CYr_lef.init();
		fn_delta_CYr_lef.m_xPar1Limit = 45.0;
		fn_delta_CYr_lef.ndinfo.nPoints[0] = alpha2_size;
		fn_delta_CYr_lef.m_Xmat[0] = alpha2;

		fn_delta_CYp_lef.init();
		fn_delta_CYp_lef.m_xPar1Limit = 45.0;
		fn_delta_CYp_lef.ndinfo.nPoints[0] = alpha2_size;
		fn_delta_CYp_lef.m_Xmat[0] = alpha2;

		fn_delta_CZq_lef.init();
		fn_delta_CZq_lef.m_xPar1Limit = 45.0;
		fn_delta_CZq_lef.ndinfo.nPoints[0] = alpha2_size;
		fn_delta_CZq_lef.m_Xmat[0] = alpha2;

		fn_delta_CLr_lef.init();
		fn_delta_CLr_lef.m_xPar1Limit = 45.0;
		fn_delta_CLr_lef.ndinfo.nPoints[0] = alpha2_size;
		fn_delta_CLr_lef.m_Xmat[0] = alpha2;

		fn_delta_CLp_lef.init();
		fn_delta_CLp_lef.m_xPar1Limit = 45.0;
		fn_delta_CLp_lef.ndinfo.nPoints[0] = alpha2_size;
		fn_delta_CLp_lef.m_Xmat[0] = alpha2;

		fn_delta_CMq_lef.init();
		fn_delta_CMq_lef.m_xPar1Limit = 45.0;
		fn_delta_CMq_lef.ndinfo.nPoints[0] = alpha2_size;
		fn_delta_CMq_lef.m_Xmat[0] = alpha2;

		fn_delta_CNr_lef.init();
		fn_delta_CNr_lef.m_xPar1Limit = 45.0;
		fn_delta_CNr_lef.ndinfo.nPoints[0] = alpha2_size;
		fn_delta_CNr_lef.m_Xmat[0] = alpha2;

		fn_delta_CNp_lef.init();
		fn_delta_CNp_lef.m_xPar1Limit = 45.0;
		fn_delta_CNp_lef.ndinfo.nPoints[0] = alpha2_size;
		fn_delta_CNp_lef.m_Xmat[0] = alpha2;

		fn_Cy_r30.init();
		fn_Cy_r30.ndinfo.nPoints[0] = alpha1_size;	
		fn_Cy_r30.ndinfo.nPoints[1] = beta1_size;	
		fn_Cy_r30.m_Xmat[0] = alpha1;
		fn_Cy_r30.m_Xmat[1] = beta1;

		fn_Cn_r30.init();
		fn_Cn_r30.ndinfo.nPoints[0] = alpha1_size;	
		fn_Cn_r30.ndinfo.nPoints[1] = beta1_size;	
		fn_Cn_r30.m_Xmat[0] = alpha1;
		fn_Cn_r30.m_Xmat[1] = beta1;

		fn_Cl_r30.init();
		fn_Cl_r30.ndinfo.nPoints[0] = alpha1_size;
		fn_Cl_r30.ndinfo.nPoints[1] = beta1_size;	
		fn_Cl_r30.m_Xmat[0] = alpha1;
		fn_Cl_r30.m_Xmat[1] = beta1;

		fn_Cy_a20.init();
		fn_Cy_a20.ndinfo.nPoints[0] = alpha1_size;	
		fn_Cy_a20.ndinfo.nPoints[1] = beta1_size;	
		fn_Cy_a20.m_Xmat[0] = alpha1;
		fn_Cy_a20.m_Xmat[1] = beta1;

		fn_Cy_a20_lef.init();
		fn_Cy_a20_lef.m_xPar1Limit = 45.0;
		fn_Cy_a20_lef.ndinfo.nPoints[0] = alpha2_size;
		fn_Cy_a20_lef.ndinfo.nPoints[1] = beta1_size;	
		fn_Cy_a20_lef.m_Xmat[0] = alpha2;
		fn_Cy_a20_lef.m_Xmat[1] = beta1;

		fn_Cn_a20.init();
		fn_Cn_a20.ndinfo.nPoints[0] = alpha1_size;	
		fn_Cn_a20.ndinfo.nPoints[1] = beta1_size;	
		fn_Cn_a20.m_Xmat[0] = alpha1;
		fn_Cn_a20.m_Xmat[1] = beta1;

		fn_Cn_a20_lef.init();
		fn_Cn_a20_lef.m_xPar1Limit = 45.0;
		fn_Cn_a20_lef.ndinfo.nPoints[0] = alpha2_size;
		fn_Cn_a20_lef.ndinfo.nPoints[1] = beta1_size;	
		fn_Cn_a20_lef.m_Xmat[0] = alpha2;
		fn_Cn_a20_lef.m_Xmat[1] = beta1;

		fn_Cl_a20.init();
		fn_Cl_a20.ndinfo.nPoints[0] = alpha1_size;	
		fn_Cl_a20.ndinfo.nPoints[1] = beta1_size;	
		fn_Cl_a20.m_Xmat[0] = alpha1;
		fn_Cl_a20.m_Xmat[1] = beta1;

		fn_Cl_a20_lef.init();
		fn_Cl_a20_lef.m_xPar1Limit = 45.0;
		fn_Cl_a20_lef.ndinfo.nPoints[0] = alpha2_size;
		fn_Cl_a20_lef.ndinfo.nPoints[1] = beta1_size;	
		fn_Cl_a20_lef.m_Xmat[0] = alpha2;
		fn_Cl_a20_lef.m_Xmat[1] = beta1;

		fn_delta_CNbeta.init();
		fn_delta_CNbeta.ndinfo.nPoints[0] = alpha1_size;	
		fn_delta_CNbeta.m_Xmat[0] = alpha1;

		fn_delta_CLbeta.init();
		fn_delta_CLbeta.ndinfo.nPoints[0] = alpha1_size;	
		fn_delta_CLbeta.m_Xmat[0] = alpha1;

		fn_delta_Cm.init();
		fn_delta_Cm.ndinfo.nPoints[0] = alpha1_size;	
		fn_delta_Cm.m_Xmat[0] = alpha1;

		fn_eta_el.init();
		fn_eta_el.ndinfo.nPoints[0] = dh1_size;	
		fn_eta_el.m_Xmat[0] = dh1;
	} // F16Aero::initfn()
}

#endif // ifndef _F16AERO_H_
