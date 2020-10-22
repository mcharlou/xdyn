/*
 * DefaultWindModel.hpp
 *
 *  Created on: 10 janv. 2020
 *      Author: mcharlou2016
 */

#ifndef CORE_INC_DEFAULTWINDMODEL_HPP_
#define CORE_INC_DEFAULTWINDMODEL_HPP_

#include <Eigen/Dense>

#include "WindModelInterface.hpp"

class DefaultWindModel : public WindModelInterface
{
public:
  DefaultWindModel ();
  virtual ~DefaultWindModel ();
  Eigen::Vector3d get_wind(const Eigen::Vector3d&,const double) const;
};

#endif /* CORE_INC_DEFAULTWINDMODEL_HPP_ */
