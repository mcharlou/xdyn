/*
 * SetOfForces.cpp
 *
 *  Created on: 8 oct. 2020
 *      Author: mcharlou2016
 */

#include "SetOfForces.hpp"

SetOfForces::SetOfForces(const std::vector<ForcePtr>& forces_): forces(forces_)
{}

std::vector<ForcePtr> SetOfForces::get_forces() const
{
	return forces;
}

void SetOfForces::sum_of_forces(double t, 										// Current time
								const BodyStates& states,						// Body states
								const EnvironmentAndFrames& env, 				// Environmental models and constants, and frame definitions
								ssc::data_source::DataSource& command_listener, // DataSource containing the commands
								const ssc::kinematics::Point& point, 			// Point at which the sum of forces must be returned
								const std::string& frame) 						// Frame in which the sum of forces must be expressed
{
	ssc::kinematics::Wrench sum_of_forces(point); // Wrench is initialized with zero values
	for (auto force:forces)
	{
		const ssc::kinematics::Wrench tau = force->operator()(states, t, env, command_listener);
		sum_of_forces = sum_of_forces + tau;
	}
}
