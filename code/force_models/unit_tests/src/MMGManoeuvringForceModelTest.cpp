/*
 * MMGManoeuvringForceModelTest.cpp
 *
 *  Created on: 27 oct. 2020
 *      Author: mcharlou2016
 */

#include "yaml_data.hpp"

#include "MMGManoeuvringForceModelTest.hpp"

#define BODY_NAME "body 1"

MMGManoeuvringForceModel MMGManoeuvringForceModelTest::get_force_model(const MMGManoeuvringForceModel::Yaml& input, const EnvironmentAndFrames& env)
{
	return MMGManoeuvringForceModel(input, BODY_NAME, env);
}

MMGManoeuvringForceModel::Yaml MMGManoeuvringForceModelTest::get_model_input(const std::map<std::string, double>& coeffs)
{
	MMGManoeuvringForceModel::Yaml input;
	input.Lpp = 1;
	input.T = 1;
	input.Xvv = 	(coeffs.find("Xvv")!=coeffs.end() ? coeffs.at("Xvv") : 0.);
	input.Xrr = 	(coeffs.find("Xrr")!=coeffs.end() ? coeffs.at("Xrr") : 0.);
	input.Xvr = 	(coeffs.find("Xvr")!=coeffs.end() ? coeffs.at("Xvr") : 0.);
	input.Xvvvv = 	(coeffs.find("Xvvvv")!=coeffs.end() ? coeffs.at("Xvvvv") : 0.);
	input.Yv = 		(coeffs.find("Yv")!=coeffs.end() ? coeffs.at("Yv") : 0.);
	input.Yr = 		(coeffs.find("Yr")!=coeffs.end() ? coeffs.at("Yr") : 0.);
	input.Yvvv = 	(coeffs.find("Yvvv")!=coeffs.end() ? coeffs.at("Yvvv") : 0.);
	input.Yrvv = 	(coeffs.find("Yrvv")!=coeffs.end() ? coeffs.at("Yrvv") : 0.);
	input.Yvrr = 	(coeffs.find("Yvrr")!=coeffs.end() ? coeffs.at("Yvrr") : 0.);
	input.Yrrr = 	(coeffs.find("Yrrr")!=coeffs.end() ? coeffs.at("Yrrr") : 0.);
	input.Nv = 		(coeffs.find("Nv")!=coeffs.end() ? coeffs.at("Nv") : 0.);
	input.Nr = 		(coeffs.find("Nr")!=coeffs.end() ? coeffs.at("Nr") : 0.);
	input.Nvvv = 	(coeffs.find("Nvvv")!=coeffs.end() ? coeffs.at("Nvvv") : 0.);
	input.Nrvv = 	(coeffs.find("Nrvv")!=coeffs.end() ? coeffs.at("Nrvv") : 0.);
	input.Nvrr = 	(coeffs.find("Nvrr")!=coeffs.end() ? coeffs.at("Nvrr") : 0.);
	input.Nrrr = 	(coeffs.find("Nrrr")!=coeffs.end() ? coeffs.at("Nrrr") : 0.);
	return input;
}

BodyStates MMGManoeuvringForceModelTest::get_states(const std::map<std::string, double>& states, const EnvironmentAndFrames& env) const
{

}

TEST_F(MMGManoeuvringForceModelTest, can_parse)
{
	const auto input = MMGManoeuvringForceModel::parse(test_data::MMG_manoeuvring_force_model());
	ASSERT_DOUBLE_EQ(input.application_point.x,-11.1);
	ASSERT_DOUBLE_EQ(input.application_point.y,0.);
	ASSERT_DOUBLE_EQ(input.application_point.z,0.);
	ASSERT_DOUBLE_EQ(input.Lpp,320);
	ASSERT_DOUBLE_EQ(input.T,20.8);
	ASSERT_DOUBLE_EQ(input.Xvv,-0.04);
	ASSERT_DOUBLE_EQ(input.Xrr,0.011);
	ASSERT_DOUBLE_EQ(input.Xvr,0.002);
	ASSERT_DOUBLE_EQ(input.Xvvvv,0.771);
	ASSERT_DOUBLE_EQ(input.Yv,-0.315);
	ASSERT_DOUBLE_EQ(input.Yr,0.083);
	ASSERT_DOUBLE_EQ(input.Yvvv,-1.607);
	ASSERT_DOUBLE_EQ(input.Yrvv,0.379);
	ASSERT_DOUBLE_EQ(input.Yvrr,-0.391);
	ASSERT_DOUBLE_EQ(input.Yrrr,0.008);
	ASSERT_DOUBLE_EQ(input.Nv,-0.137);
	ASSERT_DOUBLE_EQ(input.Nr,-0.049);
	ASSERT_DOUBLE_EQ(input.Nvvv,-0.03);
	ASSERT_DOUBLE_EQ(input.Nrvv,-0.294);
	ASSERT_DOUBLE_EQ(input.Nvrr,0.055);
	ASSERT_DOUBLE_EQ(input.Nrrr,-0.013);
}


