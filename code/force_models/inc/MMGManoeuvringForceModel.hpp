/*
 * MMGManoeuvringForceModel.hpp
 *
 *  Created on: 28 avr. 2020
 *      Author: mcharlou2016
 */

#ifndef FORCE_MODELS_INC_MMGMANOEUVRINGFORCEMODEL_HPP_
#define FORCE_MODELS_INC_MMGMANOEUVRINGFORCEMODEL_HPP_

#include "YamlCoordinates.hpp"
#include "external_data_structures_parsers.hpp"

#include "ForceModel.hpp"

class MMGManoeuvringForceModel : public ForceModel
{
public:
	struct Yaml
	{
		Yaml();
		YamlCoordinates application_point;
		double Lpp;
		double T;
		double Xvv;
		double Xrr;
		double Xvr;
		double Xvvvv;
		double Yv;
		double Yr;
		double Yvvv;
		double Yrvv;
		double Yvrr;
		double Yrrr;
		double Nv;
		double Nr;
		double Nvvv;
		double Nrvv;
		double Nvrr;
		double Nrrr;
	};
	MMGManoeuvringForceModel(const Yaml& input, const std::string& body_name, const EnvironmentAndFrames& env);
	Vector6d get_force(const BodyStates& states, const double t, const EnvironmentAndFrames& env, const std::map<std::string,double>& commands) const;
	static Yaml parse(const std::string& yaml);
	static std::string model_name();

private:
	MMGManoeuvringForceModel(); // Deactivating default constructor
	double Lpp;
	double T;
	double Xvv;
	double Xrr;
	double Xvr;
	double Xvvvv;
	double Yv;
	double Yr;
	double Yvvv;
	double Yrvv;
	double Yvrr;
	double Yrrr;
	double Nv;
	double Nr;
	double Nvvv;
	double Nrvv;
	double Nvrr;
	double Nrrr;

};

#endif /* FORCE_MODELS_INC_MMGMANOEUVRINGFORCEMODEL_HPP_ */
