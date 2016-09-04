#ifndef _F16ANALOGINPUT_H_
#define _F16ANALOGINPUT_H_

#include <cmath>

#include "../UtilityFunctions.h"

// handle normalized values with less repeating
class AnalogInput
{
protected:
	const double lower_limit;
	const double upper_limit;

	double prev_value;
	double current_value;

public:
	AnalogInput(const double lower, const double upper) 
		: lower_limit(lower)
		, upper_limit(upper)
		, prev_value(0)
		, current_value(0)
	{}
	~AnalogInput() {}

	AnalogInput& operator=(const double value)
	{
		prev_value = current_value;
		current_value = limit(value, lower_limit, upper_limit);
		return *this;
	}

	void setValue(const double value)
	{
		prev_value = current_value;
		current_value = limit(value, lower_limit, upper_limit);
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
		return lower_limit;
	}
	double getUpper() const
	{
		return upper_limit;
	}
};

#endif // ifndef _F16ANALOGINPUT_H_

