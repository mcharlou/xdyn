/*
 * ActualWindModel.cpp
 *
 *  Created on: 10 janv. 2020
 *      Author: mcharlou2016
 */

#include "ActualWindModel.hpp"

ActualWindModel::ActualWindModel(const WindMeanVelocityProfilePtr &mean_velocity_profile_, const WindTurbulenceModelPtr &turbulence_model_):
WindModelInterface(), mean_velocity_profile(mean_velocity_profile_),turbulence_model(turbulence_model_)
{}

ActualWindModel::~ActualWindModel (){}

Eigen::Vector3d ActualWindModel::get_wind(const Eigen::Vector3d &pos,const double t) const
{
	Eigen::Vector3d ret;
	ret=mean_velocity_profile->get_mean_wind(pos(2)); // Getting the mean wind velocity at specified height from the wind velocity profile
	ret=turbulence_model->get_wind_velocity(ret,t); // Adding the turbulent term to the wind velocity from the turbulent wind model
	return ret;
}
