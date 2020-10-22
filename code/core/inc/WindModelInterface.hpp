/*
 * WindModel.hpp
 *
 *  Created on: 7 janv. 2020
 *      Author: mcharlou2016
 */

#ifndef ENVIRONMENT_MODELS_INC_WINDMODELINTERFACE_HPP_
#define ENVIRONMENT_MODELS_INC_WINDMODELINTERFACE_HPP_

#include <Eigen/Dense>
#include <memory>

#include <ssc/macros.hpp>
#include <memory>

class WindModelInterface
{
public:
	WindModelInterface();
	virtual ~WindModelInterface();
	virtual Eigen::Vector3d get_wind(const Eigen::Vector3d&,const double) const=0;
};

typedef std::shared_ptr<WindModelInterface> WindModelPtr;

#endif /* ENVIRONMENT_MODELS_INC_WINDMODELINTERFACE_HPP_ */
