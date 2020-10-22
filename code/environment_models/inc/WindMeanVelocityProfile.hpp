/*
 * WindMeanVelocityProfile.hpp
 *
 *  Created on: 7 janv. 2020
 *      Author: mcharlou2016
 */

#ifndef CORE_INC_WINDMEANVELOCITYPROFILE_HPP_
#define CORE_INC_WINDMEANVELOCITYPROFILE_HPP_

#include <Eigen/Dense>
#include <ssc/macros.hpp>
#include <memory>

class WindMeanVelocityProfile {
public:
	WindMeanVelocityProfile(const double direction_,const double velocity_);
	virtual ~WindMeanVelocityProfile();
	virtual Eigen::Vector3d get_mean_wind(const double) const=0;

protected:
	double direction;
	double mean_velocity;
};

typedef std::shared_ptr<WindMeanVelocityProfile> WindMeanVelocityProfilePtr;

#endif /* CORE_INC_WINDMEANVELOCITYPROFILE_HPP_ */
