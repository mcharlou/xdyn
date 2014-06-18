/*
 * update_kinematics.hpp
 *
 *  Created on: Jun 16, 2014
 *      Author: cady
 */

#ifndef UPDATE_KINEMATICS_HPP_
#define UPDATE_KINEMATICS_HPP_

#include "Point.hpp"
#include "StateMacros.hpp"

#include "tr1_macros.hpp"
#include TR1INC(memory)

class Kinematics;
typedef TR1(shared_ptr)<Kinematics> KinematicsPtr;

Point get_origin(const StateType& x, const size_t i);

#endif /* UPDATE_KINEMATICS_HPP_ */
