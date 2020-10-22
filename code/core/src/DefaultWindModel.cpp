/*
 * DefaultWindModel.cpp
 *
 *  Created on: 10 janv. 2020
 *      Author: mcharlou2016
 */

#include "DefaultWindModel.hpp"

DefaultWindModel::DefaultWindModel (){}

DefaultWindModel::~DefaultWindModel (){}

Eigen::Vector3d DefaultWindModel::get_wind(const Eigen::Vector3d&,const double) const
{
	Eigen::Vector3d ret=Eigen::Vector3d::Zero();
	return ret;
}
