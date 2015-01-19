#ifndef _F16EQUATIONSOFMOTION_H_
#define _F16EQUATIONSOFMOTION_H_

#include "../stdafx.h"
#include <math.h>

#include "include/ED_FM_Utility.h"		// Provided utility functions that were in the initial EFM example
#include "include/F16Constants.h"		// Common constants used throughout this DLL

namespace F16
{
	class F16Motion
	{
	protected:
		//-----------------------------------------------------------------
		// This variable is very important.  Make sure you set this
		// to 0 at the beginning of each frame time or else the moments
		// will accumulate.  For each frame time, add up all the moments
		// acting on the air vehicle with this variable using th
		//
		// Units = Newton * meter
		//-----------------------------------------------------------------
		Vec3	common_moment;							
		//-----------------------------------------------------------------
		// This variable is also very important.  This defines all the forces
		// acting on the air vehicle.  This also needs to be reset to 0 at the
		// beginning of each frame time.  
		//
		// Units = Newton
		//-----------------------------------------------------------------
		Vec3	common_force;
		//-----------------------------------------------------------------
		// Center of gravity of the air vehicle as calculated from the 
		// DCS simulation, I don't believe this is utilized within this 
		// EFM.
		//
		// Units = meter
		//-----------------------------------------------------------------
		Vec3    center_of_gravity;
		//-----------------------------------------------------------------
		// The moments of inertia for the air vehicle as calculated from the
		// DCS Simulation.  This is not used within this EFM as there is a bug
		// when trying to manipulate weight or moment of inertia from within
		// the EFM.  The inertia is currently set from entry.lua
		//
		// Units: Newton * meter^2
		//-----------------------------------------------------------------
		Vec3	inertia;

		// current total mass (including fuel)
		//double mass;

	public:
		F16Motion() {}
		~F16Motion() {}

		// Very important! This function sum up all the forces acting on
		// the aircraft for this run frame.  It currently assume the force
		// is acting at the CG
		void add_local_force(const Vec3 & Force, const Vec3 & Force_pos)
		{
			sum_vec3(common_force, Force);
		}

		// Very important! This function sums up all the moments acting
		// on the aircraft for this run frame.  It currently assumes the
		// moment is acting at the CG
		void add_local_moment(const Vec3 & Moment)
		{
			sum_vec3(common_moment, Moment);
		}

		void clear()
		{
			// Very important! clear out the forces and moments before you start calculated
			// a new set for this run frame
			clear_vec3(common_force);
			clear_vec3(common_moment);
		}

		/*
		void setMassChange(double delta_mass)
		{
			mass += delta_mass;
		}
		*/

		void setMassState(double center_of_mass_x,
						double center_of_mass_y,
						double center_of_mass_z,
						double moment_of_inertia_x,
						double moment_of_inertia_y,
						double moment_of_inertia_z)
		{
			center_of_gravity.x  = center_of_mass_x;
			center_of_gravity.y  = center_of_mass_y;
			center_of_gravity.z  = center_of_mass_z;

			inertia.x = moment_of_inertia_x;
			inertia.y = moment_of_inertia_y;
			inertia.z = moment_of_inertia_z;
		}

		void getLocalForce(double &x,double &y,double &z,double &pos_x,double &pos_y,double &pos_z)
		{
			x = common_force.x;
			y = common_force.y;
			z = common_force.z;
			pos_x = center_of_gravity.x;
			pos_y = center_of_gravity.y;
			pos_z = center_of_gravity.z;
		}

		void getLocalMoment(double &x,double &y,double &z)
		{
			x = common_moment.x;
			y = common_moment.y;
			z = common_moment.z;
		}

		bool isMassChanged() const
		{
			if (inertia.x != F16::inertia_Ix_KGM2 
				|| inertia.y != F16::inertia_Iz_KGM2 
				|| inertia.z != F16::inertia_Iy_KGM2)
			{
				return true;
			}
			return false;
		}

		void getMassMomentInertiaChange(double & delta_mass,
										double & delta_mass_pos_x,
										double & delta_mass_pos_y,
										double & delta_mass_pos_z,
										double & delta_mass_moment_of_inertia_x,
										double & delta_mass_moment_of_inertia_y,
										double & delta_mass_moment_of_inertia_z)
		{
			// TODO: change in amount of fuel -> change in mass -> set here

			delta_mass = 0.0;
			delta_mass_pos_x = 0.0;
			delta_mass_pos_y = 0.0;
			delta_mass_pos_z = 0.0;

			delta_mass_moment_of_inertia_x = F16::inertia_Ix_KGM2 - inertia.x;
			delta_mass_moment_of_inertia_y = F16::inertia_Iy_KGM2 - inertia.y;
			delta_mass_moment_of_inertia_z = F16::inertia_Iz_KGM2 - inertia.z;

			// TODO: decrement this delta from inertia now?
		}

		//----------------------------------------------------------------
		// All prior forces calculated in lbs, needs to be converted
		// to units.  All prior forces calculated in lb*ft, needs
		// to be converted into N*m
		//----------------------------------------------------------------
		void updateAeroForces(double Cy_total, double Cx_total, double Cz_total, double Cl_total, double Cm_total, double Cn_total, double dynamicPressure_LBFT2)
		{
			// Cy	(force out the right wing)
			Vec3 cy_force(0.0, 0.0, Cy_total * F16::wingArea_FT2 * dynamicPressure_LBFT2 * 4.44822162825  );		// Output force in Newtons
			Vec3 cy_force_pos(0.0,0,0); //0.01437
			add_local_force(cy_force,cy_force_pos);	

			// Cx (force out the nose)
			Vec3 cx_force(Cx_total * F16::wingArea_FT2 * dynamicPressure_LBFT2 * 4.44822162825, 0, 0 );		// Output force in Newtons
			Vec3 cx_force_pos(0, 0.0,0.0);
			add_local_force(cx_force,cx_force_pos);

			// Cz (force down the bottom of the aircraft)
			Vec3 cz_force(0.0,  -Cz_total * F16::wingArea_FT2 * dynamicPressure_LBFT2 * 4.44822162825, 0.0 );	// Output force in Newtons
			Vec3 cz_force_pos(0,0,0);
			add_local_force(cz_force,cz_force_pos);

			// Cl	(Output force in N/m)
			Vec3 cl_moment(Cl_total * F16::wingArea_FT2 * dynamicPressure_LBFT2 * F16::wingSpan_FT * 1.35581795, 0.0,  0.0  );
			add_local_moment(cl_moment);

			// Cm	(Output force in N/m)
			Vec3 cm_moment(0.0, 0.0,  Cm_total * F16::wingArea_FT2 * dynamicPressure_LBFT2 * 1.35581795 * F16::meanChord_FT );
			add_local_moment(cm_moment);

			// Cn	(Output force in N/m)
			Vec3 cn_moment(0.0, -Cn_total * F16::wingArea_FT2 * dynamicPressure_LBFT2 * F16::wingSpan_FT * 1.35581795 ,  0.0   );
			add_local_moment(cn_moment);	
		}

		void updateEngineForces(double thrust_N)
		{
			// Thrust	
			Vec3 thrust_force(thrust_N , 0.0, 0.0);	// Output force in Newtons
			Vec3 thrust_force_pos(0,0,0);
			add_local_force(thrust_force, thrust_force_pos);	
		}
	};
}

#endif // ifndef _F16EQUATIONSOFMOTION_H_
