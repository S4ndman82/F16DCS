#ifndef _F16ANALOGINPUT_H_
#define _F16ANALOGINPUT_H_

#include <cmath>

#include "UtilityFunctions.h"

// handle normalized values with less repeating
class AnalogInput
{
protected:
	Limiter<double> limiter;

	double prev_value;
	double current_value;

public:
	AnalogInput(const double lower, const double upper) 
		: limiter(lower, upper)
		, prev_value(0)
		, current_value(0)
	{}
	~AnalogInput() {}

	AnalogInput& operator=(const double value)
	{
		prev_value = current_value;
		current_value = limiter.limit(value);
		return *this;
	}

	void setValue(const double value)
	{
		prev_value = current_value;
		current_value = limiter.limit(value);
	}
	double getValue() const
	{
		return current_value;
	}
	double getDelta() const
	{
		return (current_value - prev_value);
	}

	double getLower() const
	{
		return limiter.lower_limit;
	}
	double getUpper() const
	{
		return limiter.upper_limit;
	}
};

#endif // ifndef _F16ANALOGINPUT_H_

