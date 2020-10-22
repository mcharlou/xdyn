/*
 * SetOfForces.hpp
 *
 *  Created on: 8 oct. 2020
 *      Author: mcharlou2016
 */

#ifndef CORE_INC_SETOFFORCES_HPP_
#define CORE_INC_SETOFFORCES_HPP_

#include <vector>

#include "ForceModel.hpp"
#include "BodyStates.hpp"

class SetOfForces
{
public:
	SetOfForces(const std::vector<ForcePtr>& forces);

	std::vector<ForcePtr> get_forces() const;
	void sum_of_forces(double t, const BodyStates& states, const EnvironmentAndFrames& env, ssc::data_source::DataSource& command_listener,const ssc::kinematics::Point& point,const std::string& frame);

private:
	std::vector<ForcePtr> forces;
};

#endif /* CORE_INC_SETOFFORCES_HPP_ */
