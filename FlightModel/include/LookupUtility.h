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

	//size_t yValueCount;
	//size_t xParamCount;
	
	// both must be same size
	size_t axisSize;

	//U xResolution; // increment
	//U parMin, parMax;

	LookupTable(size_t count) :
		yAxis(nullptr), xAxis(nullptr), axisSize(count) /*, xResolution(0)*/
	{}
	~LookupTable() 
	{

	}

	// get lamba operator reference to generate values by resolution
	void generate([](U xPar) &cb, U parMin, U parMax, U parIncrement)
	{
		U xPar = parMin;
		for (size_t index = 0; index < axisSize && xPar <= parMax; index++, xPar += parIncrement)
		{
			// TODO: callback to function giving y for x
			xAxis[index] = xPar;
			yAxis[index] = cb(xPar);
		}
	}

	// hopefully no need for this..
	void setValue(U xPar, V yVal)
	{
		size_t index = getXIndex(xPar);
		yAxis[index] = yVal;
	}

	V getValue(U xPar)
	{
		size_t index = getXIndex(xPar);
		// x-par found -> get y-val at same index
		return yAxis[index];
	}

	size_t getXIndex(U xPar)
	{
		// note: lookup with "halving" method
		// since parameter might not be exact match

		size_t index = xParamCount / 2;
		size_t xlo = 0, xhi = xParamCount;
		while (xAxis[index] != xPar)
		{
			if (xAxis[index] < xPar)
			{
				xlo = index;
			}
			else if (xAxis[index] > xPar)
			{
				xhi = index;
			}

			// special case: if there is no exact match
			if (xAxis[index - 1] < xPar && xAxis[index + 1] > xPar)
			{
				// close enough -> use index even if value is not exact match
				// (floating point rounding perhaps)
				break;
			}

			index = (xhi - xlo) / 2;
		}
		return index;
	}
};


#endif // ifndef _LOOKUPUTILITY_H_
