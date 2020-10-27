/*
 * MMGManoeuvringForceModelTest.hpp
 *
 *  Created on: 27 oct. 2020
 *      Author: mcharlou2016
 */

#ifndef FORCE_MODELS_UNIT_TESTS_INC_MMGMANOEUVRINGFORCEMODELTEST_HPP_
#define FORCE_MODELS_UNIT_TESTS_INC_MMGMANOEUVRINGFORCEMODELTEST_HPP_

#include <ssc/data_source.hpp>
#include <ssc/random_data_generator/DataGenerator.hpp>

#include "gtest/gtest.h"

#include "MMGManoeuvringForceModel.hpp"

class MMGManoeuvringForceModelTest : public ::testing::Test
{
	ssc::random_data_generator::DataGenerator a;
	MMGManoeuvringForceModel get_force_model(const MMGManoeuvringForceModel::Yaml& input, const EnvironmentAndFrames& env);
	MMGManoeuvringForceModel::Yaml get_model_input(const std::map<std::string, double>& coeffs);
	BodyStates get_states(const std::map<std::string, double>& states, const EnvironmentAndFrames& env) const;
};

#endif /* FORCE_MODELS_UNIT_TESTS_INC_MMGMANOEUVRINGFORCEMODELTEST_HPP_ */
