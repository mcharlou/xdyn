/*
 * AbstractWageningen.cpp
 *
 *  Created on: Jun 28, 2015
 *      Author: cady
 */

#include <cmath>
#include <ssc/yaml_parser.hpp>

#include "external_data_structures_parsers.hpp"
#include "yaml.h"
#include "BodyStates.hpp"

#include "AbstractWageningen.hpp"

#define _USE_MATH_DEFINE
#define PI M_PI

AbstractWageningen::Yaml::Yaml() :
        name(),
        position_of_propeller_frame(),
        wake_coefficient(),
        relative_rotative_efficiency(),
        thrust_deduction_factor(),
        rotating_clockwise(),
        diameter()
{}

AbstractWageningen::Yaml AbstractWageningen::parse(const std::string& yaml)
{
    std::stringstream stream(yaml);
    YAML::Parser parser(stream);
    YAML::Node node;
    parser.GetNextDocument(node);
    Yaml ret;
    std::string rot;
    node["rotation"] >> rot;
    ret.rotating_clockwise = (rot == "clockwise");
    node["thrust deduction factor t"]        >> ret.thrust_deduction_factor;
    node["wake coefficient w"]               >> ret.wake_coefficient;
    node["name"]                             >> ret.name;
    node["position of propeller frame"]      >> ret.position_of_propeller_frame;
    node["relative rotative efficiency etaR"]>> ret.relative_rotative_efficiency;
    ssc::yaml_parser::parse_uv(node["diameter"], ret.diameter);
    return ret;
}

double AbstractWageningen::advance_ratio(const BodyStates& states, const std::map<std::string,double>& commands) const
{
    const double Va = fabs(states.u());
    const double n = commands.at("rpm")/(2*PI);
    return (1-w)*Va/n/D;
}

AbstractWageningen::AbstractWageningen(const Yaml& input, const std::string& body_name, const EnvironmentAndFrames& env):
            ForceModel(input.name, body_name, env, input.position_of_propeller_frame,{"rpm"}),
            w(input.wake_coefficient),
            eta_R(input.relative_rotative_efficiency),
            t(input.thrust_deduction_factor),
            kappa(input.rotating_clockwise ? -1 : 1),
            D(input.diameter),
            D4(D*D*D*D),
            D5(D4*D)
{}

std::vector<std::string> concatenate_commands(const std::vector<std::string>& set1, const std::vector<std::string>& set2)
{
	std::vector<std::string> ret(set1.begin(),set1.end());
	ret.insert(ret.end(), set2.begin(), set2.end());
	return ret;

}

AbstractWageningen::AbstractWageningen(const Yaml& input, const std::string& body_name, const EnvironmentAndFrames& env, const std::vector<std::string>& commands_):
			ForceModel(input.name, body_name, env, input.position_of_propeller_frame,concatenate_commands(commands_,{"rpm"})),
			w(input.wake_coefficient),
			eta_R(input.relative_rotative_efficiency),
			t(input.thrust_deduction_factor),
			kappa(input.rotating_clockwise ? -1 : 1),
			D(input.diameter),
			D4(D*D*D*D),
			D5(D4*D)
{}

ssc::kinematics::Vector6d AbstractWageningen::get_force(const BodyStates& states, const double, const EnvironmentAndFrames& env, const std::map<std::string,double>& commands) const
{
    ssc::kinematics::Vector6d tau = ssc::kinematics::Vector6d::Zero();
    //std::cout << "Propeller model receives command rpm: " << commands.at("rpm") << std::endl;
    const double n2 = commands.at("rpm")*commands.at("rpm")/(4*PI*PI); // In turns per second (Hz)
    const double J = advance_ratio(states, commands);
    if(n2>0)
    {
    	tau(0) = (1-t)*env.rho*n2*D4*get_Kt(commands, J);
    	tau(3) = kappa*eta_R*env.rho*n2*D5*get_Kq(commands, J);
    }
    return tau;
}
