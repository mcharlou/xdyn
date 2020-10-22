/*
 * SailForceModel.hpp
 *
 *  Created on: 16 déc. 2019
 *      Author: mcharlou2016
 */

#ifndef FORCE_MODELS_INC_SAILFORCEMODEL_HPP_
#define FORCE_MODELS_INC_SAILFORCEMODEL_HPP_

#include <string>
#include <Eigen/Dense>
#include "BodyStates.hpp"
#include "yaml.h"
#include "ForceModel.hpp"
#include <ssc/kinematics.hpp>
#include <ssc/interpolation.hpp>

class SailForceModel : public ForceModel
{
	public:
		struct Yaml
		{
			Yaml();
			virtual ~Yaml(){}
			std::string name;
			YamlPosition position_of_sail_frame;
			double area;
			double KPP;
			double heff;
			double gybe_minimum_angle;
			std::vector<double> beta;
			std::vector<double> lift_coefficient;
			std::vector<double> drag_coefficient;
		};

		static std::string model_name();
		SailForceModel(const Yaml& input, const std::string& body_name, const EnvironmentAndFrames& env);
		static Yaml parse(const std::string& yaml);
		Vector6d get_force(const BodyStates& states, const double t, const EnvironmentAndFrames& env, const std::map<std::string,double>& commands) const override;

	private:
		SailForceModel();
		double area;
		double KPP;
		double heff;
		double gybe_minimum_angle;
		double max_beta;

		Eigen::Vector3d get_AW(const BodyStates& states, const double t, const EnvironmentAndFrames& env) const;
		double get_beta(Eigen::Vector3d AW) const;

		class Impl;
		std::shared_ptr<Impl> pimpl;
};


#endif /* FORCE_MODELS_INC_SAILFORCEMODEL_HPP_ */
