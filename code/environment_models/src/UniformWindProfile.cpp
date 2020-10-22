/*
 * UniformWindProfile.cpp
 *
 *  Created on: 7 janv. 2020
 *      Author: mcharlou2016
 */

#include <math.h>
#include "WindMeanVelocityProfile.hpp"

#include "UniformWindProfile.hpp"

#define PI M_PI

UniformWindProfile::UniformWindProfile(const double direction_,const double velocity_) : WindMeanVelocityProfile(direction_,velocity_),
		wind_in_NED(velocity_*cos(direction_),velocity_*sin(direction_),0)
{}

UniformWindProfile::~UniformWindProfile() {}

Eigen::Vector3d UniformWindProfile::get_mean_wind(const double) const
{
	return wind_in_NED;
}

