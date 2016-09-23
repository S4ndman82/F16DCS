//
// LookupTable : simple helper to generate and manage lookup table during run-time,
// made as generic and reusable with possibility to adjust resolution of values.
//
// Ilkka Prusi 2016 <ilkka.prusi@gmail.com>
//

#ifndef _LOOKUPUTILITY_H_
#define _LOOKUPUTILITY_H_

#pragma once

#include <cmath>

#include <malloc.h>
#include <memory.h>


template<typename U, typename V> class LookupTable
{
public:
	V *yAxis;
	U *xAxis;

	size_t yValueCount;
	size_t xParamCount;

	U xResolution; // increment

	LookupTable() :
		yAxis(nullptr), xAxis(nullptr), yValueCount(0), xParamCount(0), xResolution(0)
	{}
	~LookupTable() {}

	void prepare(U parMin, U parMax, U parIncrement)
	{
		xResolution = (parMax - parMin) / parIncrement;
	}

	// get lamba operator reference to generate values by resolution
	void generate()
	{
	}

	// hopefully no need for this..
	void setValue(U xPar, V yVal)
	{
	}

	V getValue(U xPar)
	{
		// note: lookup with "halving" method
		// since parameter might not be exact match
	}

};


#endif // ifndef _LOOKUPUTILITY_H_
