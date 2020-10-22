/*
 * VelocityMovingAverage.hpp
 *
 *  Created on: 19 mai 2020
 *      Author: mcharlou2016
 */

#ifndef CORE_INC_VELOCITYMOVINGAVERAGE_HPP_
#define CORE_INC_VELOCITYMOVINGAVERAGE_HPP_

#include <Eigen/Dense>

#include "History.hpp"

struct BodyStates;

struct VelocityMovingAverage
{
	double Tmax;
	History U;
	History V;
	History W;
	History P;
	History Q;
	History R;

	VelocityMovingAverage(double Tmax_);
	void update(const BodyStates& states);
	void update(double t, double u_, double v_, double w_, double p_, double q_, double r_);
	void reset();
	void reset(const BodyStates& states);
	const History& get(int idx) const;
	Eigen::Matrix<double,6,1> get_vector() const;
	Eigen::Vector3d get_speed() const;

};

#endif /* CORE_INC_VELOCITYMOVINGAVERAGE_HPP_ */
