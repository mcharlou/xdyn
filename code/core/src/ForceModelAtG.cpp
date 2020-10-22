/*
 * ForceModelAtG.cpp
 *
 *  Created on: 11 sept. 2020
 *      Author: mcharlou2016
 */

#include "NumericalErrorException.hpp"
#include "BodyStates.hpp"

#include "ForceModelAtG.hpp"

ForceModelAtG::ForceModelAtG(const std::string& force_name, const std::string body_name):
		ForceModel(force_name, body_name)
{}

ForceModelAtG::ForceModelAtG(const std::string& force_name, const std::string body_name, const std::vector<std::string>& commands_):
		ForceModel(force_name, body_name, commands_)
{}

ssc::kinematics::Wrench ForceModelAtG::compute_wrench_from_force(const BodyStates& states, const EnvironmentAndFrames&, Vector6d& F)
{
	const ssc::kinematics::Wrench tau_in_body_frame_at_G(states.G, F);
	//std::cout << "Wrench (in body frame and at G):" << tau_in_body_frame_at_G << std::endl;
	force_in_body_frame = tau_in_body_frame_at_G;
	force_in_ned_frame = project_into_NED_frame(force_in_body_frame, states.get_rot_from_ned_to_body());
	return force_in_body_frame;
}
