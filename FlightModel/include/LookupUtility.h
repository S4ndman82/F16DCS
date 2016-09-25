//
// LookupTable : simple helper to generate and manage lookup table during run-time,
// made as generic and reusable with possibility to adjust resolution of values.
//
// Lookup support imprecise and rough values for lookup:
// using something like atmospheric pressure might need infinite "depth" of values
// that is not practical to have exact values for.
//
// Also the thing about floating point numbers is that they are easily rounded
// and result of one calculation is not exact match to result of another calculation.
//
// For these reasons, lookup should handle cases where values are not exact match.
// If they were always exact, you could use something like std::map<> instead.
//
// You can use combinations of complex datatypes such as struct and class,
// provided that usual comparison operators have been defined.
// For example: LookupTable<class X, struct Y> is possible.
//
// There's multiple ways that values could be added to the table:
// - linear X with Y values from another function in generate() (using std::function<>)
// - modifying Y values later with setValue() (caching case)
// - or entirely "manual" adding of values, such as:
//  for (int i..) setAtIndex(i, X, Y)..
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

// Create with something like:
// LookupTable<double,double>
// .. where 1st is X (par) and 2nd is Y axis (val) type.
//
// You can use complex datatypes provided that usual comparison operators
// have been defined: for example, LookupTable<class X, struct Y>
//
template<typename U, typename V> class LookupTable
{
public:
	// TODO: compare with using:
	// pair<U,V> instead of two-axis approach,
	// does it slow down (by using cpu-cache more than necessary)?

	V *yAxis;
	U *xAxis;

	// both must be same size
	size_t axisSize;

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

	// helper to generate linear values for X axis,
	// caller should use some method to set values for Y
	void generateX(U parMin, U parMax)
	{
		U resolution = (parMax - parMin) / axisSize;
		U xPar = parMin;
		for (size_t index = 0; index < axisSize && xPar <= parMax; index++, xPar += resolution)
		{
			xAxis[index] = xPar;
		}
	}

	// get lamba operator reference to generate values by resolution,
	// there's still possibility to fill values in other methods in case x-axis values are non-linear:
	// this is for case where X is linear and Y result of function depending on X
	void generate(std::function<V(U)> &fnY, U parMin, U parMax, U parIncrement)
	{
		U xPar = parMin;
		for (size_t index = 0; index < axisSize && xPar <= parMax; index++, xPar += parIncrement)
		{
			xAxis[index] = xPar;
			yAxis[index] = fnY(xPar);
		}
	}

	// hopefully no need for this..
	// you might need to modify values in some cases perhaps
	void setValue(const U xPar, const V yVal)
	{
		size_t index = getXIndex(xPar);
		yAxis[index] = yVal;
	}

	// for setting values entirely in caller (non-linear X and Y)
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
		// if parameter is not in range, limit to min/max known value
		if (xPar > xAxis[axisSize - 1])
		{
			return yAxis[axisSize - 1];
		}
		else if (xPar < xAxis[0])
		{
			return yAxis[0];
		}

		size_t index = getXIndex(xPar);
		// x-par found -> get y-val at same index
		return yAxis[index];
	}

	// additional averaging
	V getValueAvg(const U xPar) const
	{
		size_t index = getXIndex(xPar);
		if (index > 0 && index < (axisSize-1))
		{
			V a = yAxis[index -1];
			V b = yAxis[index +1];
			return (a + b)/2;
		}
		return yAxis[index];
	}

	// lookup with "halving" method
	size_t getXIndex(const U xPar) const
	{
		// just check possible boundaries first
		if (xAxis[0] == xPar)
		{
			return 0;
		}
		if (xAxis[axisSize -1] == xPar)
		{
			return axisSize - 1;
		}

		size_t index = xParamCount / 2;
		size_t xlo = 0, xhi = axisSize-1;
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

				// TODO: averaging of result value in this case

				break;
			}
			index = (xhi - xlo) / 2;
		}
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

				// TODO: averaging of result value in this case

				return index;
			}
		}
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

	// obvious validity check
	bool isIndex(const size_t index) const
	{
		if (index >= 0 && index < axisSize)
		{
			return true;
		}
		return false;
	}
};

#endif // ifndef _LOOKUPUTILITY_H_
