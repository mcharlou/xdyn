/*
 * YamlWindModel.cpp
 *
 *  Created on: 8 janv. 2020
 *      Author: mcharlou2016
 */

#include "YamlWindModelInput.hpp"

YamlWindProfile::YamlWindProfile(): model(), model_yaml(){}

YamlWindTurbulence::YamlWindTurbulence(): model(), model_yaml(){}

YamlWindModel::YamlWindModel(): vertical_profile(), turbulence(){}

YamlUniformWind::YamlUniformWind(): direction(), mean_velocity(){}
