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

#include <functional>

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
	{
		yAxis = new V[count];
		xAxis = new U[count];
	}
	~LookupTable() 
	{
		if (yAxis != nullptr)
		{
			delete yAxis;
			yAxis = nullptr;
		}
		if (xAxis != nullptr)
		{
			delete xAxis;
			xAxis = nullptr;
		}
	}

	// get lamba operator reference to generate values by resolution,
	// there's still possibility to fill values in other methods in case x-axis values are non-linear
	void generate(std::function<V(U)> &fn, U parMin, U parMax, U parIncrement)
	{
		U xPar = parMin;
		for (size_t index = 0; index < axisSize && xPar <= parMax; index++, xPar += parIncrement)
		{
			xAxis[index] = xPar;
			yAxis[index] = fn(xPar);
		}
	}

	// hopefully no need for this..
	// you might need to modify values in some cases perhaps
	void setValue(const U xPar, const V yVal)
	{
		size_t index = getXIndex(xPar);
		yAxis[index] = yVal;
	}

	void setAtIndex(const size_t index, const U xPar, const V yVal)
	{
		xAxis[index] = xPar;
		yAxis[index] = yVal;
	}

	V getAtIndex(const size_t index) const
	{
		return yAxis[index];
	}

	V getValue(const U xPar) const
	{
		size_t index = getXIndex(xPar);
		// x-par found -> get y-val at same index
		return yAxis[index];
	}

	size_t getXIndex(const U xPar) const
	{
		// note: lookup with "halving" method
		// since parameter might not be exact match

		size_t index = xParamCount / 2;
		size_t xlo = 0, xhi = axisSize;
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

		// currently no way to indicate "not in range"
		return index;
	}

	// traditional lookup
	size_t getIndex(const U xPar) const
	{
		for (size_t index = 0; index < axisSize; index++)
		{
			if (xAxis[index] == xPar)
			{
				return index;
			}

			if ((index + 1) < axisSize && xAxis[index + 1] > xPar)
			{
				// not exact match but next one would be larger
				// (might be floating point rounding difference)
				return index;
			}
		}

		// currently no way to indicate "not in range"
		return axisSize;
	}

	// range check
	bool isInRange(const U xPar) const
	{
		if (xPar >= xAxis[0] && xPar <= xAxis[axisSize -1])
		{
			return true;
		}
		return false;
	}

};


#endif // ifndef _LOOKUPUTILITY_H_
