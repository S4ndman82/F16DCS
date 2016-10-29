#ifndef _SUSPENSIONDATA_H_
#define _SUSPENSIONDATA_H_

#include <cmath>
#include "ED_FM_Utility.h"		// Provided utility functions that were in the initial EFM example

// suspension definitions,
// this is quick&dirty to get values from lua-scripts into DLL:
// one step in getting rid of the old, more coding required

class SuspensionData
{
public:
	SuspensionData() {}
	~SuspensionData() {}

	double mass;
	Vec3 pos;
	Vec3 moment_of_inertia;
		
	double damage_element; // (deg?) Speed threshold of jamming during impact of rotation limiter
	double damage_omega; // (deg?) Designed angle of retracted gear with horizontal axis
	double state_angle_0; // (deg?) Designed angle of extended gear with verrtical axis
	double state_angle_1; // (m) attachment point to fuselage along x-axis
	double mount_pivot_x; // (m) attachment point to fuselage along y axis
	double mount_pivot_y; // (m) distance from strut-axis to attachment point of piston to gear stand
	double mount_post_radius; // (m) length of angle brace in retracted position
	double mount_length; // (deg?) length of position vector from attachment point
	double mount_angle_1; // (m) distance from rotation-axis of strut to wheel-axis
	double post_length; // (m) displacement of wheel relative to strut
	double wheel_axle_offset; // Gear is self oriented
	bool self_attitude;
	double yaw_limit;
	double damper_coeff;
		
	double amortizer_min_length;
	double amortizer_max_length;
	double amortizer_basic_length;
	double amortizer_spring_force_factor;
	double amortizer_spring_force_factor_rate;
	double amortizer_static_force;
	double amortizer_reduce_length;
	double amortizer_direct_damper_force_factor;
	double amortizer_back_damper_force_factor;

	/* already part of F16LandingWheel
	double wheel_radius;
	double wheel_static_friction_factor; // Static friction when wheel is not moving (fully braked)
	double wheel_side_friction_factor;
	double wheel_roll_friction_factor; // Rolling friction factor when wheel moving
	double wheel_glide_friction_factor; // Sliding aircraft
	double wheel_damage_force_factor; // Tire is explosing due to hard landing
	double wheel_damage_speed; // Tire burst due to excessive speed
	double wheel_moment_of_inertia; // wheel moi as rotation body
	double wheel_brake_moment_max; // maximum value of braking moment  , N*m 
	*/

	// draw arguments for wheel
	int arg_post;
	int arg_amortizer;
	int arg_wheel_rotation;
	int arg_wheel_yaw;
	char *collision_shell_name[8];
};

#endif // _SUSPENSIONDATA_H_
