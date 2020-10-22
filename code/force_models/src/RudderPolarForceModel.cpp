/*
 * SimpleRudderForceModel.cpp
 *
 *  Created on: 21 févr. 2020
 *      Author: mcharlou2016
 */

#include <string>
#include <ssc/yaml_parser.hpp>
#include <ssc/interpolation.hpp>

#include "RudderPolarForceModel.hpp"
#include "yaml.h"
#include "YamlPosition.hpp"

std::string RudderPolarForceModel::model_name() {return "rudder polar";}


RudderPolarForceModel::Yaml RudderPolarForceModel::parse(const std::string& yaml)
{
    std::stringstream stream(yaml);
    YAML::Parser parser(stream);
    YAML::Node node;
    parser.GetNextDocument(node);
    Yaml ret;

    node["name"] >> ret.name;
    ssc::yaml_parser::parse_uv(node["rudder area"], ret.Ar);
    ssc::yaml_parser::parse_uv(node["rudder height"], ret.b);
    node["beta"]							 >> ret.beta;
    node["lift tuning coefficient"]          >> ret.lift_coeff;
    node["drag tuning coefficient"]          >> ret.drag_coeff;
    node["position of rudder in body frame"] >> ret.position_of_the_rudder_frame_in_the_body_frame;

    return ret;
}

class RudderPolarForceModel::Impl
{
    public:
        Impl(const std::vector<double>& beta, const std::vector<double>& Cl_, const std::vector<double>& Cd_) :
            Cl(beta, Cl_),
            Cd(beta, Cd_)
        {}
        ssc::interpolation::SplineVariableStep Cl;
        ssc::interpolation::SplineVariableStep Cd;

    private:
        Impl();
};

RudderPolarForceModel::RudderPolarForceModel(const Yaml& input, const std::string& body_name, const EnvironmentAndFrames& env):
		ForceModel(input.name, body_name, env, YamlPosition(input.position_of_the_rudder_frame_in_the_body_frame, body_name)),
		pimpl(new Impl(input.beta, input.lift_coeff, input.drag_coeff)),
		area(input.Ar)
{}

Vector6d RudderPolarForceModel::get_force(const BodyStates& states, const double t, const EnvironmentAndFrames& env, const std::map<std::string,double>& commands) const
{
	// TODO: Implement the model
	return Vector6d::Zero();
}

