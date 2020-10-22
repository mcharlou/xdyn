/*
 * WindMeanVelocityProfile.cpp
 *
 *  Created on: 7 janv. 2020
 *      Author: mcharlou2016
 */

#include <Eigen/Dense>

#include "WindMeanVelocityProfile.hpp"

WindMeanVelocityProfile::WindMeanVelocityProfile(const double direction_,const double velocity_): direction(direction_), mean_velocity(velocity_){}

WindMeanVelocityProfile::~WindMeanVelocityProfile(){}


