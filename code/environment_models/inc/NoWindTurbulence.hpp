/*
 * NoWindTurbulence.hpp
 *
 *  Created on: 10 janv. 2020
 *      Author: mcharlou2016
 */

#ifndef ENVIRONMENT_MODELS_INC_NOWINDTURBULENCE_HPP_
#define ENVIRONMENT_MODELS_INC_NOWINDTURBULENCE_HPP_

#include "WindTurbulenceModel.hpp"

class NoWindTurbulence : public WindTurbulenceModel
{
public:
  NoWindTurbulence ();
  virtual ~NoWindTurbulence ();

  Eigen::Vector3d get_wind_velocity(const Eigen::Vector3d mean_wind_velocity, const double) const;
};

#endif /* ENVIRONMENT_MODELS_INC_NOWINDTURBULENCE_HPP_ */
