//--------------------------------------------------------------------------
// F-16 Demo External Flight Model for DCS World
// 
// Author: CptSmiley (forums.eagle.ru username)
//
// Use Only for Non-Commercial Purposes
//
//--------------------------------------------------------------------------
// Source Data:
// 1) F-16 University of Minnesota Non-Linear Flight Model
//	  http://www.aem.umn.edu/people/faculty/balas/darpa_sec/SEC.Software.html
// 2) NASA TP 1538 Simulator Study of Stall/Post-Stall Characteristics of a 
//	  Fighter Airplane With Relaxed Longitudinal Static Stability
// 3) NASA TN D-8176 Simulator Study of the Effectiveness of an Automatic Control
//    System Designed to Improve the High Angle-of-Attack Characteristics of a
//    Fighter Airplane
// 4) AFIT/GE/ENG/88D-8 A Real-time Simulator for Man-In-The-Loop Testing of
//    Aircraft Control Systems
// 5) JBSim 1.0 F-16 Configuration 
//
//--------------------------------------------------------------------------
// F-16Demo.cpp : Defines the exported functions for the DLL application.
// Control the main portion of the discrete simulation event
//
// This project will compile a DLL.  This DLL needs to be compiled with the
// same machine type of your machine (x86 or x64).  This DLL then needs to
// be placed within the bin directory in your mod/aircraft/XXXairplane/ 
// directory within DCS World.  
//
// See associated entry.lua for how to tell the mod to use the DLL flight
// model
//--------------------------------------------------------------------------
// IMPORTANT!  COORDINATE CONVENTION:
//
// DCS WORLD Convention:
// Xbody: Out the front of the nose
// Ybody: Out the top of the aircraft
// Zbody: Out the right wing
//
// Normal Aerodynamics/Control Convention:
// Xbody: Out the front of the nose
// Ybody: Out the right wing
// Zbody: Out the bottom of the aircraft
//
// This means that if you are referincing from any aerodynamic, stabilty, or control document
// they are probably using the second set of directions.  Which means you always need to switch
// the Y and the Z and reverse the Y prior to output to DCS World
//---------------------------------------------------------------------------
// TODO List:
// -Make code more "object-oriented"...
// -Differential command into the pitch controller
// -Weight on wheels determination
// -Ground reaction modeling
// -Fix actuator dynamics
// -Improve look-up tables
// -Speed brake effects and control
//---------------------------------------------------------------------------
// KNOWN Issues:
// -On ground, the FCS controls flutter due to no filtering of alpha and Nz.
//  Need logic to determine when on ground (hackish right now) to zero those
//  signals out.
// -Aircraft naturally trims to 1.3g for some reason, need to apply -0.3 pitch
//  trim to get aircraft to trim at 1.0g for flight controller
// -Actuators cause flutter at high speed due to filtering of sensor signals
//  Removed servo-dynamics until I can figure this out
// -Gear reaction happening but ground handling not modeled due to lack of available
//  API calls
// -Gear automatically drops at 200ft to allow simple touch downs
//---------------------------------------------------------------------------
#include "stdafx.h"
#include "F_16Demo.h"

#include "include/ED_FM_Utility.h"		// Provided utility functions that were in the initial EFM example
#include "UtilityFunctions.h"			// Utility help functions 

#include "include/F16Constants.h"		// Common constants used throughout this DLL

#include "Inputs/F16Inputs.h"			// just list of inputs: can get potentially long list

// Model headers
#include "Actuators/F16Actuators.h"				//Actuators model functions
#include "Atmosphere/F16Atmosphere.h"			//Atmosphere model functions
#include "Aerodynamics/F16Aero.h"				//Aerodynamic model functions
#include "FlightControls/F16FlightControls.h"	//Flight Controls model functions
#include "Engine/F16Engine.h"					//Engine model functions
#include "Engine/F16FuelSystem.h"				//Fuel usage and tank usage functions
#include "LandingGear/F16LandingGear.h"			//Landing gear actuators, aerodynamic drag, wheelbrake function

#include "EquationsOfMotion/F16EquationsOfMotion.h"

//-------------------------------------------------------
// Start of F-16 Simulation Variables
// Probably doesn't need it's own namespace or anything
// I just quickly did this to organize my F-16 specific
// variables, needs to be done better eventually
//-------------------------------------------------------
namespace F16
{
	double		alpha_DEG				= 0.0;			// Angle of attack (deg)
	double		beta_DEG				= 0.0;			// Slideslip angle (deg)
	double		rollRate_RPS			= 0.0;			// Body roll rate (rad/sec)
	double		pitchRate_RPS			= 0.0;			// Body pitch rate (rad/sec)
	double		yawRate_RPS				= 0.0;			// Body yaw rate (rad/sec)
	double		elevator_DEG_commanded	= 0.0;			// Commanded elevator deflection from control system (deg)
	double		aileron_DEG_commanded	= 0.0;			// Commanded aileron deflection from control system (deg)
	double		rudder_DEG_commanded	= 0.0;			// Commanded rudder deflection from control system (deg)
	double		elevator_DEG			= 0.0;			// Elevator deflection (deg)
	double		aileron_DEG				= 0.0;			// Aileron deflection (deg)
	double		rudder_DEG				= 0.0;			// Rudder  deflection (deg)
	double		aileron_PCT				= 0.0;			// Aileron deflection as a percent of maximum (-1 to 1)
	double		rudder_PCT				= 0.0;			// Rudder deflection as a percent of maximum (-1 to 1)
	double		elevator_PCT			= 0.0;			// Elevator deflection as a percent of maximum (-1 to 1)
	double		leadingEdgeFlap_DEG		= 0.0;			// Leading edge flap deflection (deg)
	double		leadingEdgeFlap_PCT		= 0.0;			// Leading edge flap as a percent of maximum (0 to 1)
	double		flap_DEG				= 0.0;			// Trailing edge flap deflection (deg)
	double		flap_PCT				= 0.0;			// Trailing edge flap deflection (0 to 1)

	double		ay_world				= 0.0;			// World referenced up/down acceleration (m/s^2)
	double		accz					= 0.0;			// Az (per normal direction convention) out the bottom of the a/c (m/s^2)
	double		accy					= 0.0;			// Ay (per normal direction convention) out the right wing (m/s^2)

	double		weight_N				= 0.0;			// Weight force of aircraft (N)
	double		canopyAngle				= 0.0;			// Canopy status/angle

	F16Atmosphere Atmos;
	F16Aero Aero;
	F16Engine Engine;
	F16Actuators Actuators;
	F16FlightControls FlightControls;
	F16FuelSystem Fuel;
	F16LandingGear LandingGear;
	F16Motion Motion;
}

// This is where the simulation send the accumulated forces to the DCS Simulation
// after each run frame
void ed_fm_add_local_force(double &x,double &y,double &z,double &pos_x,double &pos_y,double &pos_z)
{
	F16::Motion.getLocalForce(x, y, z, pos_x, pos_y, pos_z);
}

// Not used
void ed_fm_add_global_force(double & x,double &y,double &z,double & pos_x,double & pos_y,double & pos_z)
{

}

// Not used
void ed_fm_add_global_moment(double & x,double &y,double &z)
{

}

// This is where the simulation send the accumulated moments to the DCS Simulation
// after each run frame
void ed_fm_add_local_moment(double &x,double &y,double &z)
{
	F16::Motion.getLocalMoment(x, y, z);
}

//-----------------------------------------------------------------------
// The most important part of the entire EFM code.  This is where you code
// gets called for each run frame.  Each run frame last for a duration of
// "dt" (delta time).  This can be used to help time certain features such
// as filters and lags
//-----------------------------------------------------------------------
void ed_fm_simulate(double dt)
{
	/* CJS - Removed hack to filter out flight controller if on ground
	if(F16::weight_on_wheels)
	{
		F16::alpha_DEG = 0.0;
		F16::az = 0.0;
	}
	*/

	// Very important! clear out the forces and moments before you start calculated
	// a new set for this run frame
	F16::Motion.clear();

	// Get the total absolute velocity acting on the aircraft with wind included
	// using english units so airspeed is in feet/second here
	F16::Atmos.updateFrame(dt);

	// TODO:
	// update amount of fuel used and change in mass
	//F16::Fuel.updateFrame(F16::Engine.thrust_N, dt);
	//F16::Motion.setMassChange(F16::Fuel.getFuelMass());

	//thrustsetting = Engine.getThrustForInput(throttleInput);
	//Fuel.updateFrame(thrustsetting, dt);

	// update thrust
	F16::Engine.updateFrame(F16::Atmos.mach, F16::Atmos.altitude_FT, dt);

	//---------------------------------------------
	//-----CONTROL DYNAMICS------------------------
	//---------------------------------------------

	F16::FlightControls.updateFrame(F16::Atmos.totalVelocity_FPS, F16::Atmos.dynamicPressure_LBFT2, F16::Atmos.ps_LBFT2, dt);

	// Call the leading edge flap dynamics controller, this controller is based on dynamic pressure and angle of attack
	// and is completely automatic
	F16::leadingEdgeFlap_DEG = F16::FlightControls.leading_edge_flap_controller(F16::alpha_DEG,F16::Atmos.dynamicPressure_LBFT2, F16::Atmos.ps_LBFT2,dt);	
	F16::leadingEdgeFlap_PCT = limit(F16::leadingEdgeFlap_DEG / 25.0, 0.0, 1.0);	

	// Call the longitudinal (pitch) controller.  Takes the following inputs:
	// -Normalize long stick input
	// -Trimmed G offset
	// -Angle of attack (deg)
	// -Pitch rate (rad/sec)
	// -Differential command (from roll controller, not quite implemented yet)
	F16::elevator_DEG_commanded   = -(F16::FlightControls.fcs_pitch_controller(F16::FlightControls.longStickInput,-0.3,F16::alpha_DEG,F16::pitchRate_RPS * F16::radiansToDegrees,(F16::accz/9.81),0.0,F16::Atmos.dynamicPressure_LBFT2,dt));
	// Call the servo dynamics model (not used as it causes high flutter in high speed situations, related to filtering and dt rate)
	F16::elevator_DEG	= F16::elevator_DEG_commanded; //F16::ACTUATORS::elevator_actuator(F16::elevator_DEG_commanded,dt);
	F16::elevator_DEG = limit(F16::elevator_DEG,-25.0,25.0);
	
	F16::aileron_DEG_commanded = (F16::FlightControls.fcs_roll_controller(F16::FlightControls.latStickInput,F16::FlightControls.longStickForce,F16::accy/9.81,F16::rollRate_RPS* F16::radiansToDegrees,0.0,F16::Atmos.dynamicPressure_LBFT2,dt));
	F16::aileron_DEG	= F16::aileron_DEG_commanded; //F16::ACTUATORS::aileron_actuator(F16::aileron_DEG_commanded,dt);
	F16::aileron_DEG = limit(F16::aileron_DEG,-21.5,21.5);

	F16::rudder_DEG_commanded = F16::FlightControls.fcs_yaw_controller(	F16::FlightControls.pedInput, 0.0, F16::yawRate_RPS * (180.0/3.14159), F16::rollRate_RPS* F16::radiansToDegrees,
													F16::FlightControls.alphaFiltered,F16::aileron_DEG_commanded,F16::accy/9.81,dt);
	F16::rudder_DEG		= F16::rudder_DEG_commanded; //F16::ACTUATORS::rudder_actuator(F16::rudder_DEG_commanded,dt);
	F16::rudder_DEG = limit(F16::rudder_DEG,-30.0,30.0);

	F16::flap_DEG = F16::FlightControls.fcs_flap_controller(F16::Atmos.totalVelocity_FPS);
	
	// reuse in drawargs
	F16::aileron_PCT = F16::aileron_DEG / 21.5;
	F16::elevator_PCT = F16::elevator_DEG / 25.0;
	F16::rudder_PCT = F16::rudder_DEG / 30.0;
	F16::flap_PCT = F16::flap_DEG / 20.0;

	double alpha1_DEG_Limited	= limit(F16::alpha_DEG,-20.0,90.0);
	double beta1_DEG_Limited	= limit(F16::beta_DEG,-30.0,30.0);

	// FLAPS (From JBSim F16.xml config)
	double CLFlaps = 0.35 * F16::flap_PCT;
	double CDFlaps = 0.08 * F16::flap_PCT;

	double CzFlaps = - (CLFlaps * cos(F16::alpha_DEG * (F16::pi/180.0)) + CDFlaps * sin(F16::pi/180.0));
	double CxFlaps = - (-CLFlaps * sin(F16::alpha_DEG * (F16::pi/180.0)) + CDFlaps * cos(F16::pi/180.0));
	
	// TODO Speedbrakes aero (from JBSim F16.xml config)
	F16::Aero.updateFrame(dt);

	// TODO: give wheelbrake input also
	F16::LandingGear.updateFrame(F16::weight_N, F16::rudder_PCT, dt);

	if (F16::LandingGear.weight_on_wheels)
	{
		/*
		Vec3 cx_wheel_friction_force(F16::LandingGear.CxWheelFriction, 0.0,0.0);
		Vec3 cx_wheel_friction_pos(0.0,0.0,0.0);
		add_local_force(cx_wheel_friction_force,cx_wheel_friction_pos);

		Vec3 cy_wheel_friction_force(0.0, 0.0, F16::LandingGear.CyWheelFriction);
		Vec3 cy_wheel_friction_pos(0.0,0.0,0.0);
		add_local_force(cy_wheel_friction_force,cy_wheel_friction_pos);
		*/
	}

	F16::Aero.hifi_C(alpha1_DEG_Limited, beta1_DEG_Limited, F16::elevator_DEG);
	F16::Aero.hifi_damping(alpha1_DEG_Limited);
    F16::Aero.hifi_C_lef(alpha1_DEG_Limited, beta1_DEG_Limited);
    F16::Aero.hifi_damping_lef(alpha1_DEG_Limited);
    F16::Aero.hifi_rudder(alpha1_DEG_Limited, beta1_DEG_Limited);
    F16::Aero.hifi_ailerons(alpha1_DEG_Limited, beta1_DEG_Limited);
	F16::Aero.hifi_other_coeffs(alpha1_DEG_Limited, F16::elevator_DEG);

	/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	compute Cx_tot, Cz_tot, Cm_tot, Cy_tot, Cn_tot, and Cl_total
	(as on NASA report p37-40)
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	// TODO: move calculations from here to separate method
	//F16::Aero.computeTotals(F16::Atmos.totalVelocity_FPS);

	/* XXXXXXXX Cx_tot XXXXXXXX */
	F16::Aero.dXdQ = (F16::meanChord_FT/(2*F16::Atmos.totalVelocity_FPS))*(F16::Aero.Cxq + F16::Aero.Cxq_delta_lef*F16::leadingEdgeFlap_PCT);
	F16::Aero.Cx_total = F16::Aero.Cx + F16::Aero.Cx_delta_lef*F16::leadingEdgeFlap_PCT + F16::Aero.dXdQ*F16::pitchRate_RPS;
	F16::Aero.Cx_total += CxFlaps + F16::LandingGear.CxGearAero;

	/* ZZZZZZZZ Cz_tot ZZZZZZZZ */ 
	F16::Aero.dZdQ = (F16::meanChord_FT/(2*F16::Atmos.totalVelocity_FPS))*(F16::Aero.Czq + F16::Aero.Cz_delta_lef*F16::leadingEdgeFlap_PCT);
	F16::Aero.Cz_total = F16::Aero.Cz + F16::Aero.Cz_delta_lef*F16::leadingEdgeFlap_PCT + F16::Aero.dZdQ*F16::pitchRate_RPS;
	F16::Aero.Cz_total += CzFlaps + F16::LandingGear.CzGearAero;

	/* MMMMMMMM Cm_tot MMMMMMMM */ 
	F16::Aero.dMdQ = (F16::meanChord_FT/(2*F16::Atmos.totalVelocity_FPS))*(F16::Aero.Cmq + F16::Aero.Cmq_delta_lef*F16::leadingEdgeFlap_PCT);
	F16::Aero.Cm_total = F16::Aero.Cm*F16::Aero.eta_el + F16::Aero.Cz_total*(F16::referenceCG_PCT-F16::actualCG_PCT) + F16::Aero.Cm_delta_lef*F16::leadingEdgeFlap_PCT + F16::Aero.dMdQ*F16::pitchRate_RPS + F16::Aero.Cm_delta + F16::Aero.Cm_delta_ds;

	/* YYYYYYYY Cy_tot YYYYYYYY */
	F16::Aero.dYdail = F16::Aero.Cy_delta_a20 + F16::Aero.Cy_delta_a20_lef*F16::leadingEdgeFlap_PCT;
	F16::Aero.dYdR = (F16::wingSpan_FT/(2*F16::Atmos.totalVelocity_FPS))*(F16::Aero.Cyr + F16::Aero.Cyr_delta_lef*F16::leadingEdgeFlap_PCT);
	F16::Aero.dYdP = (F16::wingSpan_FT/(2*F16::Atmos.totalVelocity_FPS))*(F16::Aero.Cyp + F16::Aero.Cyp_delta_lef*F16::leadingEdgeFlap_PCT);
	F16::Aero.Cy_total = F16::Aero.Cy + F16::Aero.Cy_delta_lef*F16::leadingEdgeFlap_PCT + F16::Aero.dYdail*F16::aileron_PCT + F16::Aero.Cy_delta_r30*F16::rudder_PCT + F16::Aero.dYdR*F16::yawRate_RPS + F16::Aero.dYdP*F16::rollRate_RPS;
	
	/* NNNNNNNN Cn_tot NNNNNNNN */ 
	F16::Aero.dNdail = F16::Aero.Cn_delta_a20 + F16::Aero.Cn_delta_a20_lef*F16::leadingEdgeFlap_PCT;
	F16::Aero.dNdR = (F16::wingSpan_FT/(2*F16::Atmos.totalVelocity_FPS))*(F16::Aero.Cnr + F16::Aero.Cnr_delta_lef*F16::leadingEdgeFlap_PCT);
	F16::Aero.dNdP = (F16::wingSpan_FT/(2*F16::Atmos.totalVelocity_FPS))*(F16::Aero.Cnp + F16::Aero.Cnp_delta_lef*F16::leadingEdgeFlap_PCT);
	F16::Aero.Cn_total = F16::Aero.Cn + F16::Aero.Cn_delta_lef*F16::leadingEdgeFlap_PCT - F16::Aero.Cy_total*(F16::referenceCG_PCT-F16::actualCG_PCT)*(F16::meanChord_FT/F16::wingSpan_FT) + F16::Aero.dNdail*F16::aileron_PCT + F16::Aero.Cn_delta_r30*F16::rudder_PCT + F16::Aero.dNdR*F16::yawRate_RPS + F16::Aero.dNdP*F16::rollRate_RPS + F16::Aero.Cn_delta_beta*F16::beta_DEG;

	/* LLLLLLLL Cl_total LLLLLLLL */
	F16::Aero.dLdail = F16::Aero.Cl_delta_a20 + F16::Aero.Cl_delta_a20_lef*F16::leadingEdgeFlap_PCT;
	F16::Aero.dLdR = (F16::wingSpan_FT/(2*F16::Atmos.totalVelocity_FPS))*(F16::Aero.Clr + F16::Aero.Clr_delta_lef*F16::leadingEdgeFlap_PCT);
	F16::Aero.dLdP = (F16::wingSpan_FT/(2*F16::Atmos.totalVelocity_FPS))*(F16::Aero.Clp + F16::Aero.Clp_delta_lef*F16::leadingEdgeFlap_PCT);
	F16::Aero.Cl_total = F16::Aero.Cl + F16::Aero.Cl_delta_lef*F16::leadingEdgeFlap_PCT + F16::Aero.dLdail*F16::aileron_PCT + F16::Aero.Cl_delta_r30*F16::rudder_PCT + F16::Aero.dLdR*F16::yawRate_RPS + F16::Aero.dLdP*F16::rollRate_RPS + F16::Aero.Cl_delta_beta*F16::beta_DEG;

	//----------------------------------------------------------------
	// All prior forces calculated in lbs, needs to be converted
	// to units.  All prior forces calculated in lb*ft, needs
	// to be converted into N*m
	//----------------------------------------------------------------

	F16::Motion.updateAeroForces(F16::Aero.Cy_total, F16::Aero.Cx_total, F16::Aero.Cz_total, F16::Aero.Cl_total, F16::Aero.Cm_total, F16::Aero.Cn_total, F16::Atmos.dynamicPressure_LBFT2);
	F16::Motion.updateEngineForces(F16::Engine.thrust_N);
	
	// Tell the simulation that it has gone through the first frame
	//F16::simInitialized = true;
	F16::Actuators.simInitialized = true;
	F16::FlightControls.simInitialized = true;

	/*
	F16::weight_on_wheels = false;
	if((F16::weight_N > cz_force.y) && (abs(F16::ay_world) >= -0.5) && (F16::gearDown == 1.0))
	{
		F16::weight_on_wheels = true;
	}
	*/
	
}

void ed_fm_set_atmosphere(	double h,//altitude above sea level			(meters)
							double t,//current atmosphere temperature   (Kelvin)
							double a,//speed of sound					(meters/sec)
							double ro,// atmosphere density				(kg/m^3)
							double p,// atmosphere pressure				(N/m^2)
							double wind_vx,//components of velocity vector, including turbulence in world coordinate system (meters/sec)
							double wind_vy,//components of velocity vector, including turbulence in world coordinate system (meters/sec)
							double wind_vz //components of velocity vector, including turbulence in world coordinate system (meters/sec)
						)
{
	F16::Atmos.setAtmosphere(t, ro, h * F16::meterToFoot, p * 0.020885434273);
}

void ed_fm_set_current_mass_state ( double mass,
									double center_of_mass_x,
									double center_of_mass_y,
									double center_of_mass_z,
									double moment_of_inertia_x,
									double moment_of_inertia_y,
									double moment_of_inertia_z
									)
{
	F16::Motion.setMassState(center_of_mass_x, center_of_mass_y, center_of_mass_z,
							moment_of_inertia_x, moment_of_inertia_y, moment_of_inertia_z);

	F16::weight_N = mass * 9.80665002864;
}

/*
called before simulation to set up your environment for the next step
*/
void ed_fm_set_current_state (double ax,//linear acceleration component in world coordinate system
							double ay,//linear acceleration component in world coordinate system
							double az,//linear acceleration component in world coordinate system
							double vx,//linear velocity component in world coordinate system
							double vy,//linear velocity component in world coordinate system
							double vz,//linear velocity component in world coordinate system
							double px,//center of the body position in world coordinate system
							double py,//center of the body position in world coordinate system
							double pz,//center of the body position in world coordinate system
							double omegadotx,//angular accelearation components in world coordinate system
							double omegadoty,//angular accelearation components in world coordinate system
							double omegadotz,//angular accelearation components in world coordinate system
							double omegax,//angular velocity components in world coordinate system
							double omegay,//angular velocity components in world coordinate system
							double omegaz,//angular velocity components in world coordinate system
							double quaternion_x,//orientation quaternion components in world coordinate system
							double quaternion_y,//orientation quaternion components in world coordinate system
							double quaternion_z,//orientation quaternion components in world coordinate system
							double quaternion_w //orientation quaternion components in world coordinate system
							)
{
	F16::ay_world = ay;
}

void ed_fm_set_current_state_body_axis(	double ax,//linear acceleration component in body coordinate system (meters/sec^2)
										double ay,//linear acceleration component in body coordinate system (meters/sec^2)
										double az,//linear acceleration component in body coordinate system (meters/sec^2)
										double vx,//linear velocity component in body coordinate system (meters/sec)
										double vy,//linear velocity component in body coordinate system (meters/sec)
										double vz,//linear velocity component in body coordinate system (meters/sec)
										double wind_vx,//wind linear velocity component in body coordinate system (meters/sec)
										double wind_vy,//wind linear velocity component in body coordinate system (meters/sec)
										double wind_vz,//wind linear velocity component in body coordinate system (meters/sec)
										double omegadotx,//angular accelearation components in body coordinate system (rad/sec^2)
										double omegadoty,//angular accelearation components in body coordinate system (rad/sec^2)
										double omegadotz,//angular accelearation components in body coordinate system (rad/sec^2)
										double omegax,//angular velocity components in body coordinate system (rad/sec)
										double omegay,//angular velocity components in body coordinate system (rad/sec)
										double omegaz,//angular velocity components in body coordinate system (rad/sec)
										double yaw,  //radians (rad)
										double pitch,//radians (rad/sec)
										double roll, //radians (rad/sec)
										double common_angle_of_attack, //AoA  (rad)
										double common_angle_of_slide   //AoS  (rad)
	)
{
	F16::Atmos.setAirspeed(vx, vy, vz, wind_vx, wind_vy, wind_vz);

	//-------------------------------
	// Start of setting F-16 states
	//-------------------------------
	F16::alpha_DEG		= common_angle_of_attack * F16::radiansToDegrees;
	F16::beta_DEG		= common_angle_of_slide * F16::radiansToDegrees;
	F16::rollRate_RPS	= omegax;
	F16::pitchRate_RPS	= omegaz;
	F16::yawRate_RPS	= -omegay;

	F16::accz = ay;
	F16::accy = az;
}

void ed_fm_set_command(int command, float value)	// Command = Command Index (See Export.lua), Value = Signal Value (-1 to 1 for Joystick Axis)
{
	//----------------------------------
	// Set F-16 Raw Inputs
	//----------------------------------

	switch (command)
	{
	case JoystickRoll:
		F16::FlightControls.setLatStickInput(limit(value, -1.0, 1.0));
		break;

	case JoystickPitch:
		F16::FlightControls.setLongStickInput(limit(-value, -1.0, 1.0));
		break;

	case JoystickYaw:
		F16::FlightControls.setPedInput(limit(-value, -1.0, 1.0));
		break;

	case JoystickThrottle:
		F16::Engine.setThrottleInput(limit(((-value + 1.0) / 2.0) * 100.0, 0.0, 100.0));
		break;

		/*
	case AirBrake:
		if (F16::FlightControls.airbrakeDown > 0)
		{
			// down -> up
			F16::FlightControls.airbrakeDown = 0;
		}
		else
		{
			// up -> down
			F16::FlightControls.airbrakeDown = 1.0;
		}
		break;
		*/
	case AirBrakeOn:
		F16::FlightControls.setAirbrake(0); // up?
		break;
	case AirBrakeOff:
		F16::FlightControls.setAirbrake(1.0); // down?
		break;

		/*
		// analog input (axis)
	case WheelBrakeLeft:
	case WheelBrakeRight:
		//F16::LandingGear
		break;
		*/

		// switch/button input
	case WheelBrakesOn:
	case WheelBrakesOff:
		//F16::LandingGear
		break;

		/**/
	case Gear:
		if (F16::LandingGear.gearDownAngle > 0)
		{
			// down -> up
			F16::LandingGear.gearDownAngle = 0;
		}
		else
		{
			// up -> down
			F16::LandingGear.gearDownAngle = 1.0;
		}
		break;
	case LandingGearUp:
		F16::LandingGear.gearDownAngle = 0.0; // up?
		break;
	case LandingGearDown:
		F16::LandingGear.gearDownAngle = 1.0; // 1.0 = down (see drawargs)
		break;
		/**/

		/*
		// adjust drawargs with these?
		// needs some support here?
	case MouseCameraRotateLeftRight:
		break;
	case MouseCameraRotateUpDown:
		break;
	case MouseCameraZoom:
		break;
		*/

	case Canopy:
		// on/off toggle (needs some actuator support as well)
		if (F16::canopyAngle > 0)
		{
			F16::canopyAngle = 0; // down
		}
		else
		{
			F16::canopyAngle = 0.9; // up
		}
		break;

	default:
		// do nothing
		break;
	}
}

/*
	Mass handling 

	will be called  after ed_fm_simulate :
	you should collect mass changes in ed_fm_simulate 

	double delta_mass = 0;
	double x = 0;
	double y = 0; 
	double z = 0;
	double piece_of_mass_MOI_x = 0;
	double piece_of_mass_MOI_y = 0; 
	double piece_of_mass_MOI_z = 0;
 
	//
	while (ed_fm_change_mass(delta_mass,x,y,z,piece_of_mass_MOI_x,piece_of_mass_MOI_y,piece_of_mass_MOI_z))
	{
	//internal DCS calculations for changing mass, center of gravity,  and moments of inertia
	}
*/
bool ed_fm_change_mass(double & delta_mass,
						double & delta_mass_pos_x,
						double & delta_mass_pos_y,
						double & delta_mass_pos_z,
						double & delta_mass_moment_of_inertia_x,
						double & delta_mass_moment_of_inertia_y,
						double & delta_mass_moment_of_inertia_z
						)
{
	// get fuel mass according to consumption
	//see: F16::Fuel.updateFrame()

	if (F16::Motion.isMassChanged() == true)
	{
		F16::Motion.getMassMomentInertiaChange(delta_mass, 
											delta_mass_pos_x, 
											delta_mass_pos_y, 
											delta_mass_pos_z,
											delta_mass_moment_of_inertia_x, 
											delta_mass_moment_of_inertia_y, 
											delta_mass_moment_of_inertia_z);


		// Can't set to true...crashing right now :(
		return false;
	}
	else
	{
		return false;
	}

	/*
	if (fuel_consumption_since_last_time > 0)
	{
		delta_mass		 = fuel_consumption_since_last_time;
		delta_mass_pos_x = -1.0;
		delta_mass_pos_y =  1.0;
		delta_mass_pos_z =  0;

		delta_mass_moment_of_inertia_x	= 0;
		delta_mass_moment_of_inertia_y	= 0;
		delta_mass_moment_of_inertia_z	= 0;

		fuel_consumption_since_last_time = 0; // set it 0 to avoid infinite loop, because it called in cycle 
		// better to use stack like structure for mass changing 
		return true;
	}
	else 
	{
		return false;
	}
	*/
}

/*
	set internal fuel volume , init function, called on object creation and for refueling , 
	you should distribute it inside at different fuel tanks
*/
void ed_fm_set_internal_fuel(double fuel)
{
	F16::Fuel.internal_fuel = fuel;
}

/*
	get internal fuel volume 
*/
double ed_fm_get_internal_fuel()
{
	return F16::Fuel.internal_fuel;
}

/*
	set external fuel volume for each payload station , called for weapon init and on reload
*/
void ed_fm_set_external_fuel(int station,
								double fuel,
								double x,
								double y,
								double z)
{
	F16::Fuel.setExternalFuel(station, fuel, x, y, z);
}

/*
	get external fuel volume 
*/
double ed_fm_get_external_fuel ()
{
	return F16::Fuel.getExternalFuel();
}

// is there somewhere a list of what each entry in array is for??
//
void ed_fm_set_draw_args(EdDrawArgument * drawargs, size_t size)
{
	// nose gear
	drawargs[0].f = (float)F16::LandingGear.gearDownAngle; // gear angle {0=retracted;1=extended}
	drawargs[1].f = (float)F16::LandingGear.strutCompressionNose; // strut compression {0=extended;0.5=parking;1=retracted}

	//Nose Gear Steering
	drawargs[2].f = (float)F16::LandingGear.noseGearAngle; // nose gear angle {-1=CW max;1=CCW max}

	// right gear
	drawargs[3].f = (float)F16::LandingGear.gearDownAngle; // gear angle {0;1}
	drawargs[4].f = (float)F16::LandingGear.strutCompressionRight; // strut compression {0;0.5;1}

	// left gear
	drawargs[5].f = (float)F16::LandingGear.gearDownAngle; // gear angle {0;1}
	drawargs[6].f = (float)F16::LandingGear.strutCompressionLeft; // strut compression {0;0.5;1}

	drawargs[9].f = (float)F16::flap_PCT; // right flap
	drawargs[10].f = (float)F16::flap_PCT; // left flap

	drawargs[11].f = (float)-F16::aileron_PCT; // right aileron
	drawargs[12].f = (float) F16::aileron_PCT; // left aileron

	drawargs[13].f   = (float)F16::leadingEdgeFlap_PCT; // right slat
	drawargs[14].f   = (float)F16::leadingEdgeFlap_PCT; // left slat

	drawargs[15].f = (float)-F16::elevator_PCT; // right elevator
	drawargs[16].f = (float)-F16::elevator_PCT; // left elevator

	drawargs[17].f = (float) F16::rudder_PCT; // right rudder
	drawargs[18].f = (float)-F16::rudder_PCT; // left rudder

	drawargs[28].f   = (float)limit(F16::Engine.afterburner, 0.0, 1.0); // afterburner right engine
	drawargs[29].f   = (float)limit(F16::Engine.afterburner, 0.0, 1.0); // afterburner left engine

	drawargs[38].f = (float)F16::canopyAngle; // draw angle of canopy {0=closed;0.9=elevated;1=no draw}

	//drawargs[49].f // nav lights
	//drawargs[51].f // landing lights
}

void ed_fm_configure(const char * cfg_path)
{

}

double ed_fm_get_param(unsigned index)
{	
	if (index > ED_FM_END_ENGINE_BLOCK)
	{
		// unlikely case?
		return 0;
	}

	switch (index)
	{
	case ED_FM_ENGINE_0_RPM:			
	case ED_FM_ENGINE_0_RELATED_RPM:	
	case ED_FM_ENGINE_0_THRUST:			
	case ED_FM_ENGINE_0_RELATED_THRUST:	
		return 0; // APU

	case ED_FM_ENGINE_1_RPM:
		return (F16::Engine.getThrottleInput()/100.0) * 3000;
	case ED_FM_ENGINE_1_RELATED_RPM:
		return (F16::Engine.getThrottleInput()/100.0);
	case ED_FM_ENGINE_1_THRUST:
		return (F16::Engine.getThrottleInput()/100.0) * 5000 * 9.81;
	case ED_FM_ENGINE_1_RELATED_THRUST:
		return (F16::Engine.getThrottleInput()/100.0);

	default:
		// silence compiler warning(s)
		break;
	}
	return 0;	
}

void ed_fm_cold_start()
{
	//F16::LandingGear.gearDownAngle = 1.0; // 1.0 as gear down?
}

void ed_fm_hot_start()
{
	//F16::LandingGear.gearDownAngle = 1.0; // 1.0 as gear down?
}

void ed_fm_hot_start_in_air()
{

}

double test()
{
	return 10.0;
}

