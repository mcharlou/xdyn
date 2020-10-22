/*
 * YamlWindModelInput.hpp
 *
 *  Created on: 8 janv. 2020
 *      Author: mcharlou2016
 */

#ifndef EXTERNAL_DATA_STRUCTURES_INC_YAMLWINDMODELINPUT_HPP_
#define EXTERNAL_DATA_STRUCTURES_INC_YAMLWINDMODELINPUT_HPP_

#include <string>

struct YamlWindProfile
{
	YamlWindProfile();
	std::string model;
	std::string model_yaml;
};

struct YamlWindTurbulence
{
	YamlWindTurbulence();
	std::string model;
	std::string model_yaml;
};

struct YamlWindModel
{
	YamlWindModel();
	YamlWindProfile vertical_profile;
	YamlWindTurbulence turbulence;
};

struct YamlUniformWind
{
	YamlUniformWind();
	double direction;
	double mean_velocity;
};





#endif /* EXTERNAL_DATA_STRUCTURES_INC_YAMLWINDMODELINPUT_HPP_ */
