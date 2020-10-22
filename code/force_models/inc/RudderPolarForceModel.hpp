/*
 * SimpleRudderForceModel.hpp
 *
 *  Created on: 21 févr. 2020
 *      Author: mcharlou2016
 */

#ifndef FORCE_MODELS_INC_RUDDERPOLARFORCEMODEL_HPP_
#define FORCE_MODELS_INC_RUDDERPOLARFORCEMODEL_HPP_

#include <vector>
#include <string>

#include "ForceModel.hpp"
#include "YamlCoordinates.hpp"

class RudderPolarForceModel : public ForceModel
{
public:
	struct Yaml
	{
		Yaml();
		std::string name;
		double Ar;                                                      //!< Rudder area (in m^2) (cf. "Maneuvering Technical Manual", J. Brix, Seehafen Verlag, p. 76 fig. 1.2.4)
		double b;                                                       //!< Rudder height (in m) (cf. "Maneuvering Technical Manual", J. Brix, Seehafen Verlag, p. 76 fig. 1.2.4)
		std::vector<double> beta;
		std::vector<double> lift_coeff;
		std::vector<double> drag_coeff;
		YamlCoordinates position_of_the_rudder_frame_in_the_body_frame;
	};

	RudderPolarForceModel(const Yaml& input, const std::string& body_name, const EnvironmentAndFrames& env);
	Vector6d get_force(const BodyStates& states, const double t, const EnvironmentAndFrames& env, const std::map<std::string,double>& commands) const override;
	static Yaml parse(const std::string& yaml);
	static std::string model_name();


private:
	RudderPolarForceModel(); // Deactivated
	double area;
	class Impl;
	std::shared_ptr<Impl> pimpl;
};

#endif /* FORCE_MODELS_INC_RUDDERPOLARFORCEMODEL_HPP_ */
