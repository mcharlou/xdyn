/*
 * EmpiricDampingForceModel.cpp
 *
 *  Created on: 11 févr. 2020
 *      Author: mcharlou2016
 */

#include <utility>
#include <Eigen/Dense>
#include "Body.hpp"
#include <ssc/yaml_parser.hpp>
#include <ssc/kinematics.hpp>
#include <cmath>
#define PI M_PI

#include "yaml.h"
#include "EnvironmentAndFrames.hpp"
#include "YamlPosition.hpp"

#include "EmpiricRollDampingForceModel.hpp"

std::string EmpiricRollDampingForceModel::model_name() {return "empiric roll damping";}

EmpiricRollDampingForceModel::Yaml::Yaml():
		Lpp(),
		B(),
		T(),
		m(),
		OG(),
		GM(),
		Cb(),
		Cm(),
		Sw(),
		Tmax(),
		Ix(),
		A44()
{}

EmpiricRollDampingForceModel::EmpiricRollDampingForceModel(const Yaml& input, const std::string& body_name, const EnvironmentAndFrames&):
		ForceModelAtH(EmpiricRollDampingForceModel::model_name(), body_name),
		Lpp(input.Lpp),
		B(input.B),
		T(input.T),
		m(input.m),
		OG(input.OG),
		GM(input.GM),
		Cb(input.Cb),
		Cm(input.Cm),
		Sw(input.Sw),
		Tmax(input.Tmax),
		Ix(input.Ix),
		A44(input.A44)
{}

double EmpiricRollDampingForceModel::get_Tmax() const
{
	return Tmax;
}

EmpiricRollDampingForceModel::Yaml EmpiricRollDampingForceModel::parse(const std::string& yaml)
{
    std::stringstream stream(yaml);
    YAML::Parser parser(stream);
    YAML::Node node;
    parser.GetNextDocument(node);
    Yaml ret;

    ssc::yaml_parser::parse_uv(node["Lpp"], ret.Lpp);
    ssc::yaml_parser::parse_uv(node["B"], ret.B);
    ssc::yaml_parser::parse_uv(node["T"], ret.T);
    ssc::yaml_parser::parse_uv(node["m"], ret.m);
    ssc::yaml_parser::parse_uv(node["OG"], ret.OG);
    ssc::yaml_parser::parse_uv(node["GM"], ret.GM);
    node["Cm"] >> ret.Cm;
    node["Cb"] >> ret.Cb;
    ssc::yaml_parser::parse_uv(node["Sw"], ret.Sw);
    ssc::yaml_parser::parse_uv(node["Tmax"], ret.Tmax);
    ssc::yaml_parser::parse_uv(node["Ix"], ret.Ix);
    ssc::yaml_parser::parse_uv(node["A44"], ret.A44);

    return ret;
}

Vector6d EmpiricRollDampingForceModel::get_force(const BodyStates& states, const double, const EnvironmentAndFrames& env, const std::map<std::string,double>&) const
{
    Vector6d tau = Vector6d::Zero();
    const double U = states.u.average(Tmax);
    const double omega_r = sqrt(m*env.g*GM/(Ix+A44));
    const std::pair<double,double> phi_bounds = states.get_phi_bounds_over_period(Tmax);
    //std::cout << "Phi bounds: " << phi_bounds << std::endl;
    const double Beq=Bl(states, env, U)+Bf(states, env, U, omega_r, phi_bounds)+Be(states, env, U, omega_r, phi_bounds);
    tau(3)=-Beq*states.get_generalized_speed()[3];
    return tau;
}

double EmpiricRollDampingForceModel::Bl(const BodyStates& states, const EnvironmentAndFrames& env, double U) const
{
	const double kappa = (Cm <=0.92 ? 0 : (Cm>=0.97 ? 0.3 : 0.1));
	const double kN = 2*PI*T/Lpp+kappa*(4.1*B/Lpp-0.045);
	return 0.15/2*env.rho*U*Lpp*pow(T,3)*kN*(1-2.8*OG/T+4.667*pow(OG/T,2));
}

double EmpiricRollDampingForceModel::Bf(const BodyStates& states, const EnvironmentAndFrames& env, double U, double omega_r, const std::pair<double,double>& phi_bounds) const
{
	const double re = 1/PI*((0.887+0.145*Cb)*Sw/B-2*OG);
	const double Bf0 = 0.787*env.rho*Sw*pow(re,2)*sqrt(omega_r*env.nu)*(1+0.00814*pow(pow(re,2)*omega_r*pow((phi_bounds.second-phi_bounds.first)/2,2)/env.nu,0.386));
	return Bf0*(1+4.1*U/(omega_r*B));
}

double EmpiricRollDampingForceModel::Be(const BodyStates& states, const EnvironmentAndFrames& env, double U, double omega_r, const std::pair<double,double>& phi_bounds) const
{
	const double H0 = B/(2*T);
	const double Be0 = 2/PI*env.rho*B*pow(T,4)*(pow(H0,2)+1-OG/T)*(pow(H0,2)+pow((1-OG/T),2))*(phi_bounds.second-phi_bounds.first)/2*omega_r;
	if(U>0) return Be0*pow(0.04*omega_r*B/U,2)/(1+pow(0.04*omega_r*B/U,2));
	else return Be0;
}



