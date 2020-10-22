/*
 * GravityForceModel.cpp
 *
 *  Created on: Jun 17, 2014
 *      Author: cady
 */

#include <ssc/kinematics.hpp>

#include "EnvironmentAndFrames.hpp"
#include "SurfaceElevationInterface.hpp"
#include "Body.hpp"
#include "YamlPosition.hpp"

#include "GravityForceModel.hpp"

std::string GravityForceModel::model_name() {return "gravity";}

GravityForceModel::GravityForceModel(const std::string& body_name, const EnvironmentAndFrames&)
    : ForceModelAtG(GravityForceModel::model_name(), body_name)
{}

Vector6d GravityForceModel::get_force(const BodyStates& states, const double t, const EnvironmentAndFrames& env, const std::map<std::string,double>& commands) const
{
    const ssc::kinematics::Transform T = env.k->get(states.name, "NED");
    const double m = states.solid_body_inertia->operator()(2,2);
    Vector6d ret;
    ret << T.get_rot()*Eigen::Vector3d(0,0,m*env.g),Eigen::Vector3d(0,0,0);
    return ret;
}

double GravityForceModel::potential_energy(const BodyStates& states, const EnvironmentAndFrames& env, const std::vector<double>& x) const
{
    const double m = states.solid_body_inertia->operator()(2,2);
    return -m*env.g*x[2];
}
