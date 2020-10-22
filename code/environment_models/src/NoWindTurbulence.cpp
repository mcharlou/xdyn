/*
 * NoWindTurbulence.cpp
 *
 *  Created on: 10 janv. 2020
 *      Author: mcharlou2016
 */

#include "NoWindTurbulence.hpp"

NoWindTurbulence::NoWindTurbulence(): WindTurbulenceModel(){}

NoWindTurbulence::~NoWindTurbulence(){}

Eigen::Vector3d NoWindTurbulence::get_wind_velocity(const Eigen::Vector3d mean_wind_velocity, const double) const
{
	return mean_wind_velocity;
}

