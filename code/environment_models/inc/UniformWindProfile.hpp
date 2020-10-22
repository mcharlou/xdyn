/*
 * UniformWindProfile.hpp
 *
 *  Created on: 7 janv. 2020
 *      Author: mcharlou2016
 */

#ifndef ENVIRONMENT_MODELS_INC_UNIFORMWINDPROFILE_HPP_
#define ENVIRONMENT_MODELS_INC_UNIFORMWINDPROFILE_HPP_

#include "WindMeanVelocityProfile.hpp"

class UniformWindProfile : public WindMeanVelocityProfile
{
public:
	UniformWindProfile(const double direction_,const double velocity_);
	virtual ~UniformWindProfile();

	Eigen::Vector3d get_mean_wind(const double) const;

private:
	Eigen::Vector3d wind_in_NED;
};

#endif /* ENVIRONMENT_MODELS_INC_UNIFORMWINDPROFILE_HPP_ */
