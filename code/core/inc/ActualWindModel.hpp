/*
 * ActualWindModel.hpp
 *
 *  Created on: 10 janv. 2020
 *      Author: mcharlou2016
 */

#ifndef CORE_INC_ACTUALWINDMODEL_HPP_
#define CORE_INC_ACTUALWINDMODEL_HPP_

#include <Eigen/Dense>
#include "WindMeanVelocityProfile.hpp"
#include "WindTurbulenceModel.hpp"

#include "WindModelInterface.hpp"

class ActualWindModel : public WindModelInterface
{
public:
  ActualWindModel(const WindMeanVelocityProfilePtr &mean_velocity_profile_, const WindTurbulenceModelPtr &turbulence_model_);
  virtual ~ActualWindModel ();
  Eigen::Vector3d get_wind(const Eigen::Vector3d &pos,const double t) const;

private:
  WindMeanVelocityProfilePtr mean_velocity_profile;
  WindTurbulenceModelPtr turbulence_model;
};

#endif /* CORE_INC_ACTUALWINDMODEL_HPP_ */
