/*
 * ForceModelAtH.cpp
 *
 *  Created on: 11 sept. 2020
 *      Author: mcharlou2016
 */

#include "NumericalErrorException.hpp"
#include "BodyStates.hpp"

#include "ForceModelAtH.hpp"

ForceModelAtH::ForceModelAtH(const std::string& force_name, const std::string body_name):
		ForceModel(force_name, body_name)
{}

ForceModelAtH::ForceModelAtH(const std::string& force_name, const std::string body_name, const std::vector<std::string>& commands_):
		ForceModel(force_name, body_name, commands_)
{}

ssc::kinematics::Wrench ForceModelAtH::compute_wrench_from_force(const BodyStates& states, const EnvironmentAndFrames&, Vector6d& F)
{
	const ssc::kinematics::Wrench tau_in_body_frame_at_H(states.hydrodynamic_forces_calculation_point, F);
	force_in_body_frame = tau_in_body_frame_at_H.change_point_of_application(states.G);
	force_in_ned_frame = project_into_NED_frame(force_in_body_frame, states.get_rot_from_ned_to_body());
	return force_in_body_frame;
}

