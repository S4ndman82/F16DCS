#ifndef _F16AEROFUNCTION_H_
#define _F16AEROFUNCTION_H_

#include <cmath>

#include <malloc.h>
#include <memory.h>

#include "../ndinterp.h"
#include "../UtilityFunctions.h"

// mostly just wrapper to assist with interpolation 
// and related buffers and configuration for each calculation
class AERO_Function
{
private:
	// disallow copies
	AERO_Function(const AERO_Function &other) {}
	AERO_Function& operator=(const AERO_Function &other) { return *this; }

protected:

	// stuff only for temporary use,
	// avoid reallocations
	UtilBuffer<int> indexVector;
	UtilBuffer<double> m_Tbuf; // reusable buffer to reduce malloc()/free()

	UtilMatrix<double> m_xPointMat; // used in interpolation, reduce reallocation
	UtilMatrix<int> m_indexMat; // used in interpolation, reduce reallocation

public:
	ND_INFO ndinfo; // dimensions descriptor

	double **m_Xmat; // pointers to static arrays of data (X matrix)
	double *m_Ydata; // pointer to static array of related data (Y)

	//double *m_xPar; // parameters for interpolation (1-3 pars)
	double m_xPar1Limit; // upper limit for X-parameter 1 in functions (only upper and only for this)

	double m_result; // result value

	AERO_Function(const int nDimension, double *Ydata)
		: indexVector()
		, m_Tbuf()
		, m_xPointMat()
		, m_indexMat()
		, ndinfo()
		, m_Xmat(nullptr)
		//, m_xPar(nullptr)
		, m_Ydata(Ydata)
		, m_xPar1Limit(0)
		, m_result(0)
	{
		ndinfo.nPoints = nullptr;
		ndinfo.nDimension = nDimension;
	}

	~AERO_Function()
	{
		if (ndinfo.nPoints != nullptr)
		{
			free(ndinfo.nPoints);
			ndinfo.nPoints = nullptr;
		}
		/*
		if (m_xPar != nullptr)
		{
			free(m_xPar);
			m_xPar = nullptr;
		}
		*/
		if (m_Xmat != nullptr)
		{
			free(m_Xmat);
			m_Xmat = nullptr;
		}
		m_indexMat.release();
		m_xPointMat.release();
		m_Tbuf.release();
		indexVector.release();
	}

	void init()
	{
		ndinfo.nPoints = (int*)malloc(ndinfo.nDimension*sizeof(int));

		// just array of pointers to static data
		m_Xmat = (double **)malloc(ndinfo.nDimension*sizeof(double*));

		// 1-3 parameters
		//m_xPar = (double*)malloc(ndinfo.nDimension*sizeof(double*));

		int nVertices = (1 << ndinfo.nDimension);
		m_Tbuf.getVec(nVertices); // preallocate

		// preallocate
		m_xPointMat.allocate(ndinfo.nDimension, 2);
		m_indexMat.allocate(ndinfo.nDimension, 2);

		// preallocate another temporary buffer to reuse
		indexVector.getVec(ndinfo.nDimension);
	}

	double interpnf1Lim(const double xPar1)
	{
		if (xPar1 > m_xPar1Limit) // par 1 is normally alpha
		{
			// use limit
			return interpnf1(m_xPar1Limit);
		}
		return interpnf1(xPar1);
	}
	double interpnf2Lim(const double xPar1, const double xPar2)
	{
		if (xPar1 > m_xPar1Limit) // par 1 is normally alpha
		{
			// use limit
			return interpnf2(m_xPar1Limit, xPar2);
		}
		return interpnf2(xPar1, xPar2);
	}
	double interpnf3Lim(const double xPar1, const double xPar2, const double xPar3)
	{
		if (xPar1 > m_xPar1Limit) // par 1 is normally alpha
		{
			// use limit
			return interpnf3(m_xPar1Limit, xPar2, xPar3);
		}
		return interpnf3(xPar1, xPar2, xPar3);
	}

	double interpnf1(const double xPar1)
	{
		double Xpars[1];
		Xpars[0] = xPar1;

		m_result = interpn(indexVector, m_Xmat, m_Ydata, Xpars, m_xPointMat, m_indexMat, ndinfo, m_Tbuf);
		return m_result;
	}

	double interpnf2(const double xPar1, const double xPar2)
	{
		double Xpars[2];
		Xpars[0] = xPar1;
		Xpars[1] = xPar2;

		m_result = interpn(indexVector, m_Xmat, m_Ydata, Xpars, m_xPointMat, m_indexMat, ndinfo, m_Tbuf);
		return m_result;
	}

	double interpnf3(const double xPar1, const double xPar2, const double xPar3)
	{
		double Xpars[3];
		Xpars[0] = xPar1;
		Xpars[1] = xPar2;
		Xpars[2] = xPar3;

		m_result = interpn(indexVector, m_Xmat, m_Ydata, Xpars, m_xPointMat, m_indexMat, ndinfo, m_Tbuf);
		return m_result;
	}
};

#endif // ifndef _F16AEROFUNCTION_H_
