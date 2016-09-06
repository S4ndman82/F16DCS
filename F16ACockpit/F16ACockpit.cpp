// F16ACockpit.cpp : Defines the exported functions for the DLL application.
//

#include "stdafx.h"
#include <windows.h>

#include "F16ACockpit.h"

// for debug use
#include <wchar.h>
#include <stdio.h>

// integrate with EFM DLL
// -> cockpit DLL may need to call EFM to get/set information
#include "../FlightModel/F_16Demo.h"


wchar_t dbgmsg[255] = { 0 };
//dbgmsg[0] = 0;

// prototype for later..
bool locateEfmDll();

// create handlers for controls (stick, throttle, pedals)

// create handlers for "passive" instruments (clock, ADI, engine RPM)

// create handlers for displays (active instruments)

// bind handlers to inputs, animation items

/*
void * ed_cockpit_get_parameter_handle			  (const char * name)
{
}

void   ed_cockpit_update_parameter_with_string    (void		  * handle	,const char * string_value)
{
}

void   ed_cockpit_update_parameter_with_number    (void		  * handle	,double   number_value)
{
}

bool   ed_cockpit_parameter_value_to_number       (const void * handle	,double & res	,bool interpolated)
{
}

bool   ed_cockpit_parameter_value_to_string       (const void * handle	,char * buffer	,unsigned buffer_size)
{
}

//return 0 if equal , -1 if first less than second 1 otherwise
int    ed_cockpit_compare_parameters			  (void		  * handle_1,void * handle_2)
{
	return 0;
}
*/

/*
load CockpitBase.dll, 
GetProcAddress "ed_cockpit_get_base_sensor_output", 
you can call that with the argument of the base sensor index and it will return the float value to you.

-> access lua-defined sensors from C++ code this way?
*/


double test(double in)
{
	return in*10.0;
}

bool locateEfmDll()
{
	// function prototype for function exported from cockpit dll
	typedef double test(double in);

	HMODULE	efm_dll = GetModuleHandle(L"F16DemoFM.dll"); //assume that we work inside same process
	if (efm_dll == NULL)
	{
		return false;
	}

	test *pfnTest = (test*)GetProcAddress(efm_dll, "test");
	if (pfnTest == NULL)
	{
		return false;
	}

	double res = (double)(*pfnTest)(10.0);

	// all successful
	return true;
}
