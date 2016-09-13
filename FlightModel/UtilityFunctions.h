//-----------------------------------------------------------------------------------------------
// These are utility functions defined by the University of Minnesota Flight Model Code
// They are mostly there to help look up tables.  I added in some limiter functions
//
// You could use this header to code up common math utilities your code may use multiple times
//-----------------------------------------------------------------------------------------------
//
// File split, helper classes added.
//
// Ilkka Prusi 2016 <ilkka.prusi@gmail.com>
//

#ifndef _UTILFUNCTIONS_H_
#define _UTILFUNCTIONS_H_

#pragma once

#include <cmath>

#include <malloc.h>
#include <memory.h>


// Simple upper and lower limiter
double limit(double input, double lower_limit, double upper_limit)
{
	if(input > upper_limit)
	{
		return upper_limit;
	}
	else if(input < lower_limit)
	{
		return lower_limit;
	}
	else
	{
		return input;
	}
}

// check if there's need to keep configurable limits somewhere..
// changed to template for supporting different types
template<typename T> class Limiter
{
public:
	const T lower_limit;
	const T upper_limit;

	Limiter(const T lower, const T upper)
		: lower_limit(lower), upper_limit(upper) 
	{}
	~Limiter() {}

	T limit(const T input) const
	{
		if (input > upper_limit)
		{
			return upper_limit;
		}
		else if (input < lower_limit)
		{
			return lower_limit;
		}
		return input;
	}
	bool isOutsideRange(const T input) const
	{
		if (input > upper_limit)
		{
			return true;
		}
		else if (input < lower_limit)
		{
			return true;
		}
		return false;
	}
};

// simple configurable rate- and range-limiting class,
// keep value within hysteresis-scale
template<typename T> class DeltaLimiter
{
public:
	Limiter<T> limiter;
	const T delta_max;

	T current;

	DeltaLimiter(const T lower, const T upper, const T delta)
		: limiter(lower, upper), delta_max(delta),
		current(0)
	{}
	~DeltaLimiter() {}

	T deltaLimit(const T input) const
	{
		T diff = input - current;
		if (abs(diff) <= delta_max)
		{
			// not over delta limit (rate-change)
			// -> check range
			current = limiter.limit(input);
		}
		else
		{
			// this is crude way but should work for now..
			// limit to max rate-change,
			// then check limit to range
			if (diff > 0)
			{
				current = limiter.limit(current + delta_max);
			}
			else
			{
				current = limiter.limit(current - delta_max);
			}
		}
		return current;
	}
};

// generic linear function:
// enter values according to graphs in FLCS documents (for example)
template<typename T> class LinearFunction
{
public:
	const T angleFactor; // factor to multiply with (coefficient, ratio of X/Y)
	const T zeroBias; // zero-point offset

	// range where functional (result varies according to input)
	const T minRange;
	const T maxRange;

	// TODO: use limiter here as well
	//DeltaLimiter<double> limit;
	// for now, just use older one
	Limiter<T> limiter;

	/*
	LinearFunction(const T min, const T max, const T lower, const T upper)
		: angleFactor(0), zeroBias(0), minRange(min), maxRange(max), limiter(lower, upper)
	{
		// TODO: calculation for angle here according to values
		//angleFactor =
	}
	*/
	LinearFunction(const T angle, const T offset, const T min, const T max, const T lower, const T upper)
		: angleFactor(angle), zeroBias(offset), minRange(min), maxRange(max), limiter(lower, upper)
	{}
	~LinearFunction() {}

	bool isInRange(const T input) const
	{
		if (input >= minRange && input <= maxRange)
		{
			return true;
		}
		return false;
	}

	// TODO: use delta-limiter
	T result(const T input) const
	{
		// no change outside range
		if (input < minRange)
		{
			return limiter.lower_limit;
		}
		if (input > maxRange)
		{
			return limiter.upper_limit;
		}

		// angle * input -> value in line of the graph
		// TODO: use delta limiter for rate-change hysteresis "scale" instead of single line
		return limiter.limit((angleFactor * input) + zeroBias);
	}

};

#endif // ifndef _UTILFUNCTIONS_H_
