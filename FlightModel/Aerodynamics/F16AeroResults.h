#ifndef _F16AERORESULTS_H_
#define _F16AERORESULTS_H_

// just container for results:
// keep in one place and reduce some repeated things
class F16AeroResults
{
public:
	double r_Cx = 0.0;
	double r_CxEle0 = 0.0;
	double r_Cz = 0.0;
	double r_CzEle0 = 0.0;
	double r_Cm = 0.0;
	double r_CmEle0 = 0.0;
	double r_Cy = 0.0;
	double r_Cn = 0.0;
	double r_CnEle0 = 0.0;
	double r_Cl = 0.0;
	double r_ClEle0 = 0.0;
	double r_Cx_lef = 0.0;
	double r_Cz_lef = 0.0;
	double r_Cm_lef = 0.0;
	double r_Cy_lef = 0.0;
	double r_Cn_lef = 0.0;
	double r_Cl_lef = 0.0;
	double r_CXq = 0.0;
	double r_CZq = 0.0;
	double r_CMq = 0.0;
	double r_CYp = 0.0;
	double r_CYr = 0.0;
	double r_CNr = 0.0;
	double r_CNp = 0.0;
	double r_CLp = 0.0;
	double r_CLr = 0.0;
	double r_delta_CXq_lef = 0.0;
	double r_delta_CYr_lef = 0.0;
	double r_delta_CYp_lef = 0.0;
	double r_delta_CZq_lef = 0.0;
	double r_delta_CLr_lef = 0.0;
	double r_delta_CLp_lef = 0.0;
	double r_delta_CMq_lef = 0.0;
	double r_delta_CNr_lef = 0.0;
	double r_delta_CNp_lef = 0.0;
	double r_Cy_r30 = 0.0;
	double r_Cn_r30 = 0.0;
	double r_Cl_r30 = 0.0;
	double r_Cy_a20 = 0.0;
	double r_Cy_a20_lef = 0.0;
	double r_Cn_a20 = 0.0;
	double r_Cn_a20_lef = 0.0;
	double r_Cl_a20 = 0.0;
	double r_Cl_a20_lef = 0.0;
	double r_delta_CNbeta = 0.0;
	double r_delta_CLbeta = 0.0;
	double r_delta_Cm = 0.0;
	double r_eta_el = 0.0;
public:
	F16AeroResults() {}
	~F16AeroResults() {}
};


#endif // _F16AERORESULTS_H_

