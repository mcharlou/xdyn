/*
 * FlettnerRotorForceModel.hpp
 *
 *  Created on: 14 janv. 2020
 *      Author: mcharlou2016
 */

#ifndef FORCE_MODELS_INC_FLETTNERROTORFORCEMODEL_HPP_
#define FORCE_MODELS_INC_FLETTNERROTORFORCEMODEL_HPP_

#include <string>
#include <Eigen/Dense>
#include <ssc/kinematics.hpp>
#include <ssc/interpolation.hpp>

#include "BodyStates.hpp"
#include "yaml.h"
#include "ForceModel.hpp"

class FlettnerRotorForceModel : public ForceModel
{
public:
	struct Yaml
	{
		Yaml();
		virtual ~Yaml(){}
		std::string name;
		YamlPosition position_of_rotor_frame;
		double diameter;
		double length;
		std::vector<double> skin_relative_velocity;
		std::vector<double> lift_coefficient;
		std::vector<double> drag_coefficient;
	};


	FlettnerRotorForceModel (const Yaml& input, const std::string& body_name, const EnvironmentAndFrames& env);
	static Yaml parse(const std::string& yaml);
	Vector6d get_force(const BodyStates& states, const double t, const EnvironmentAndFrames& env, const std::map<std::string,double>& commands) const override;
	static std::string model_name();

private:
	FlettnerRotorForceModel ();
	double area;
	double radius;
	double half_length;

	Eigen::Vector3d get_AW(const BodyStates& states, const double t, const EnvironmentAndFrames& env) const;

	class Impl;
	std::shared_ptr<Impl> pimpl;
};

#endif /* FORCE_MODELS_INC_FLETTNERROTORFORCEMODEL_HPP_ */
