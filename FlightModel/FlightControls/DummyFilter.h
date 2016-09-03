#ifndef _DUMMYFILTER_H_
#define _DUMMYFILTER_H_

#include <memory.h>
#include "../UtilityFunctions.h"
//#include "../include/general_filter.h"

class DummyFilter
{
public:
	DummyFilter(double *pdNumerators   = NULL,
                double *pdDenominators = NULL,
                int     nOrder         = 0)
				: m_dFrameTime_SEC(0)
	{
		InitFilter(pdNumerators, pdDenominators, nOrder);
	}
	~DummyFilter() {}

    void InitFilter( double *pdNumerators,
                        double *pdDenominators,
                        int     nOrder)
	{
		SetCoefficients(pdNumerators, pdDenominators);
		m_nOrder = nOrder;
		m_dFrameTime_SEC = 0;
	}
    void SetCoefficients(double *pdNumerator, double *pdDenominator)
	{
	}
	double Filter(double dFrameTime_SEC, double dInput)
	{
		// DCS is using constant frametime (each simulation step is same length)
		// -> we could use constant value for delta time anyway
		double deltaTime = dFrameTime_SEC - m_dFrameTime_SEC;

		m_dFrameTime_SEC = dFrameTime_SEC;
		return dInput;
	}
	void ResetFilter(double dt = 0)
	{
		m_dFrameTime_SEC = dt;
	}
	double GetOutput()
	{
		return m_dOutput;
	}

protected:
	double      m_dOutput;

    int                     m_nOrder;  
    int                     m_nOrder1;
    double                  m_dFrameTime_SEC;  
    double                  m_dZDenomNormal;
    double                 *m_pdInputs;
    double                 *m_pdOutputs;
    double                 *m_pdSNumerator;
    double                 *m_pdSDenominator;
    double                 *m_pdZNumerator;
    double                 *m_pdZDenominator;
    double                 *m_pdPDt;
};

#endif // ifndef _DUMMYFILTER_H_
