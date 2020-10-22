/*
 * WindTurbulenceModel.hpp
 *
 *  Created on: 7 janv. 2020
 *      Author: mcharlou2016
 */

#ifndef CORE_INC_WINDTURBULENCEMODEL_HPP_
#define CORE_INC_WINDTURBULENCEMODEL_HPP_

#include <Eigen/Dense>

#include <ssc/macros.hpp>
#include <memory>

class WindTurbulenceModel {
public:
	WindTurbulenceModel();
	virtual ~WindTurbulenceModel();
	virtual Eigen::Vector3d get_wind_velocity(const Eigen::Vector3d mean_wind_velocity, const double) const=0;
};

typedef std::shared_ptr<WindTurbulenceModel> WindTurbulenceModelPtr;

#endif /* CORE_INC_WINDTURBULENCEMODEL_HPP_ */
