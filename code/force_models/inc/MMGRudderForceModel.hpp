/*
 * MMGRudderForceModel.hpp
 *
 *  Created on: 5 mai 2020
 *      Author: mcharlou2016
 */

#ifndef FORCE_MODELS_INC_MMGRUDDERFORCEMODEL_HPP_
#define FORCE_MODELS_INC_MMGRUDDERFORCEMODEL_HPP_

#include "YamlPosition.hpp"
#include <memory>

#include "ForceModel.hpp"

class MMGRudderForceModel : public ForceModel
{
public:
	struct Yaml
	{
		Yaml();
		virtual ~Yaml(){}
		std::string name;
		YamlPosition position_of_local_frame;
		double Lpp;

		double Dp;
		double xp_;
		double k0;
		double k1;
		double k2;
		double wp0;
		double C1;
		double C2_pos;
		double C2_neg;
		double tp;
		double Jp_max;

		double xr_;
		double tr;
		double ah;
		double xh_;
		double kappa;
		double lr_;
		double epsilon;
		double fx;
		double gamma_r_pos;
		double gamma_r_neg;
		double Ar;
		double Hr;
		double f_alpha;
	};

	static std::string model_name();
	MMGRudderForceModel (const Yaml& input, const std::string& body_name, const EnvironmentAndFrames& env);
	static Yaml parse(const std::string& yaml);
	ssc::kinematics::Vector6d get_force(const BodyStates& states, const double t, const EnvironmentAndFrames& env, const std::map<std::string,double>& commands) const override;
	virtual void feed(Observer& observer, ssc::kinematics::KinematicsPtr& k, const ssc::kinematics::Point& G) const;
	virtual void extra_observations(Observer& observer) const override;

private:
	MMGRudderForceModel();
	class Impl;
	std::shared_ptr<Impl> pimpl;
};

#endif /* FORCE_MODELS_INC_MMGRUDDERFORCEMODEL_HPP_ */
