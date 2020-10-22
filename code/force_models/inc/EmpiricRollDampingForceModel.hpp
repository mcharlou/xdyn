/*
 * EmpiricRollDampingForceModel.hpp
 *
 *  Created on: 11 févr. 2020
 *      Author: mcharlou2016
 */

#ifndef FORCE_MODELS_INC_EMPIRICROLLDAMPINGFORCEMODEL_HPP_
#define FORCE_MODELS_INC_EMPIRICROLLDAMPINGFORCEMODEL_HPP_

#include <ssc/kinematics.hpp>
#include "ForceModelAtH.hpp"
#include "yaml.h"

class EmpiricRollDampingForceModel : public ForceModelAtH
{
	public:
		struct Yaml
		{
			Yaml();
			double Lpp;
			double B;
			double T;
			double m;
			double OG;
			double GM;
			double Cb;
			double Cm;
			double Sw;
			double Tmax;
			double Ix;
			double A44;
		};
		EmpiricRollDampingForceModel(const Yaml& input, const std::string& body_name, const EnvironmentAndFrames& env);
		Vector6d get_force(const BodyStates& states, const double t, const EnvironmentAndFrames& env, const std::map<std::string,double>& commands) const override;
		static Yaml parse(const std::string& yaml);
		static std::string model_name();
		double get_Tmax() const override;

	private:
		EmpiricRollDampingForceModel();
		double Bl(const BodyStates& states, const EnvironmentAndFrames& env, double U) const;
		double Bf(const BodyStates& states, const EnvironmentAndFrames& env, double U, double omega_r, const std::pair<double,double>& phi_bounds) const;
		double Be(const BodyStates& states, const EnvironmentAndFrames& env, double U, double omega_r, const std::pair<double,double>& phi_bounds) const;
		double Lpp;
		double B;
		double T;
		double m;
		double OG;
		double GM;
		double Cb;
		double Cm;
		double Sw;
		double Tmax;
		double Ix;
		double A44;

};

#endif /* FORCE_MODELS_INC_EMPIRICROLLDAMPINGFORCEMODEL_HPP_ */
