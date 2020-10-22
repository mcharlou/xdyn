/*
 * DampingForceModel.cpp
 *
 *  Created on: Oct 17, 2014
 *      Author: cady
 */

#include <Eigen/Dense>
#include <ssc/kinematics.hpp>

#include "Body.hpp"
#include "YamlPosition.hpp"

#include "DampingForceModel.hpp"

DampingForceModel::DampingForceModel(const std::string& name, const std::string& body_name, const EnvironmentAndFrames&, const Eigen::Matrix<double,6,6>& D_) :
	ForceModelAtH(name, body_name), D(D_)
{}

Vector6d DampingForceModel::get_force(const BodyStates& states, const double, const EnvironmentAndFrames&, const std::map<std::string,double>&) const
{
    Eigen::Matrix<double, 6, 1> W;
    W <<states.u(),
        states.v(),
        states.w(),
        states.p(),
        states.q(),
        states.r();
    return get_force_and_torque(D, W);
}

