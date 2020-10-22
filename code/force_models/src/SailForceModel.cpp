/*
 * SailForceModel.cpp
 *
 *  Created on: 16 déc. 2019
 *      Author: mcharlou2016
 */


#include <iostream>
#include <string>
#include <algorithm>
#include <math.h>
#include <Eigen/Dense>
#include <ssc/yaml_parser.hpp>
#include <ssc/kinematics.hpp>
#include <ssc/interpolation.hpp>
#include "ForceModel.hpp"
#include "BodyStates.hpp"
#include "yaml.h"
#include "external_data_structures_parsers.hpp"
#include "InvalidInputException.hpp"

#include "SailForceModel.hpp"

#define PI M_PI

std::string SailForceModel::model_name() {return "IMS mainsail";}

class SailForceModel::Impl
{
    public:
        Impl(const std::vector<double>& beta, const std::vector<double>& Cl_, const std::vector<double>& Cd_) :
            Cl(beta, Cl_),
            Cd(beta, Cd_),
			fcdmult({0.,0.10,0.20,0.30,0.40,0.50,0.55,0.60,0.65,0.70,0.75,0.80,0.85,0.90,0.95,1.00}, {1.,1.06,1.06,1.06,1.06,1.06,1.06,1.055,1.048,1.035,1.02,1.008,1.002,1.,1.004,1.06}),
			tack(Tack::undefined)
        {
        }
        enum class Tack{port,starboard,undefined};
        ssc::interpolation::SplineVariableStep Cl;
        ssc::interpolation::SplineVariableStep Cd;
        ssc::interpolation::SplineVariableStep fcdmult; // To account for non-linearities in the relation between induced drag and Cl^2 (cf. ORC VPP Documentation 2019)
        Tack tack;

    private:
        Impl();
};

SailForceModel::Yaml::Yaml() :
			name(),
			position_of_sail_frame(),
			area(),
			KPP(),
			heff(),
			gybe_minimum_angle(0.),
			beta(),
			lift_coefficient(),
			drag_coefficient()
{
}

SailForceModel::SailForceModel(const Yaml& input, const std::string& body_name, const EnvironmentAndFrames& env) :
			ForceModel(input.name, body_name, env, input.position_of_sail_frame, {"flat","reef"}),
			pimpl(new Impl(input.beta, input.lift_coefficient, input.drag_coefficient)),
			area(input.area),KPP(input.KPP),heff(input.heff),gybe_minimum_angle(input.gybe_minimum_angle),
			max_beta(*std::max_element(input.beta.begin(),input.beta.end()))
{
//	if(max_beta<180. || *std::min_element(input.beta.begin(),input.beta.end())>0.) THROW(__PRETTY_FUNCTION__, InvalidInputException, "The lift and drag coefficients for the 'IMS mainsail' force model must be given from beta=0 to beta=180deg");
}

SailForceModel::Yaml SailForceModel::parse(const std::string& yaml)
{
	std::stringstream stream(yaml);
	YAML::Parser parser(stream);
	YAML::Node node;
	parser.GetNextDocument(node);
	Yaml ret;

	node["name"] >> ret.name;
	ssc::yaml_parser::parse_uv(node["area"], ret.area);
	ssc::yaml_parser::parse_uv(node["rig height"], ret.heff);
	if(node.FindValue("gybe minimum angle")) ssc::yaml_parser::parse_uv(node["gybe minimum angle"], ret.gybe_minimum_angle);
	node["KPP"] >> ret.KPP;
	node["beta"] >> ret.beta;
	node["lift coefficient"] >> ret.lift_coefficient;
	node["drag coefficient"] >> ret.drag_coefficient;
	node["position of sail"] >> ret.position_of_sail_frame;

	return ret;
}

ssc::kinematics::Vector6d SailForceModel::get_force(const BodyStates& states, const double t, const EnvironmentAndFrames& env, const std::map<std::string,double>& commands) const
{
	ssc::kinematics::Vector6d force = ssc::kinematics::Vector6d::Zero();
	Eigen::Vector3d AW=get_AW(states, t, env);
	double beta=get_beta(AW);
	double AWS=AW.norm();
	double flat = commands.at("flat");
	double reef = commands.at("reef");
	double Aref = area*reef;
	double CE = KPP + Aref/(PI*pow(heff,2));
	if(pimpl->tack==pimpl->Tack::undefined) pimpl->tack = (AW(1)<0 ? pimpl->Tack::starboard : pimpl->Tack::port);
	if(AW(1)<0 && pimpl->tack==pimpl->Tack::port && beta+gybe_minimum_angle>180)
	{
		double Cl = flat*pimpl->Cl.f(std::min(max_beta,beta+gybe_minimum_angle));
		double Cd = pimpl->Cd.f(std::min(max_beta,beta+gybe_minimum_angle)) + CE*pow(Cl,2);

		const double Lift=Cl*(0.5*env.air_density*pow(AWS,2)*Aref);
		const double Drag=Cd*(0.5*env.air_density*pow(AWS,2)*Aref);

		force(0)=sin(PI*beta/180)*Lift-cos(PI*beta/180)*Drag;
		force(1)=cos(PI*beta/180)*Lift-sin(PI*beta/180)*Drag;
	}
	else if(AW(1)>=0 && pimpl->tack==pimpl->Tack::starboard && beta+gybe_minimum_angle>180)
	{
		double Cl = flat*pimpl->Cl.f(std::min(max_beta,beta+gybe_minimum_angle));
		double Cd = pimpl->Cd.f(std::min(max_beta,beta+gybe_minimum_angle)) + CE*pow(Cl,2);

		const double Lift=Cl*(0.5*env.air_density*pow(AWS,2)*Aref);
		const double Drag=Cd*(0.5*env.air_density*pow(AWS,2)*Aref);

		force(0)=sin(PI*beta/180)*Lift-cos(PI*beta/180)*Drag;
		force(1)=-cos(PI*beta/180)*Lift+sin(PI*beta/180)*Drag;
	}
	else
	{
		double Cl = flat*pimpl->Cl.f(beta);
		double Cd = pimpl->Cd.f(beta) + CE*pow(Cl,2);

		const double Lift=Cl*(0.5*env.air_density*pow(AWS,2)*Aref);
		const double Drag=Cd*(0.5*env.air_density*pow(AWS,2)*Aref);

		force(0)=sin(PI*beta/180)*Lift-cos(PI*beta/180)*Drag;
		force(1)=cos(PI*beta/180)*Lift+sin(PI*beta/180)*Drag;

		force(1)=(AW(1)<0 ? -force(1) : force(1));
		pimpl->tack = (AW(1)<0 ? pimpl->Tack::starboard : pimpl->Tack::port);
	}

	return force;
}

Eigen::Vector3d SailForceModel::get_AW(const BodyStates& states,const double t, const EnvironmentAndFrames& env) const
{
    auto rot_from_NED_frame_to_body_frame = env.k->get(states.name, "NED").get_rot();
    auto rot_from_body_frame_to_internal_frame = env.k->get(get_name(), states.name).get_rot();
    Eigen::Vector3d center_of_local_frame = env.k->get("NED",get_name()).get_point().v;
    Eigen::Vector3d wind_in_NED_frame=env.wind->get_wind(center_of_local_frame,t);
    Eigen::Vector3d wind_in_body_frame=rot_from_NED_frame_to_body_frame*wind_in_NED_frame;
    Eigen::Vector3d Vs(states.u(),states.v(),states.w());
    Eigen::Vector3d AW=rot_from_body_frame_to_internal_frame*(wind_in_body_frame-Vs);
    return AW;
}

double SailForceModel::get_beta(Eigen::Vector3d AW) const
{
	double wind_angle=180*atan(AW(1)/AW(0))/PI;
	if(AW(1)<=0){
		wind_angle=(AW(0)>0 ? 180+wind_angle : wind_angle);
	}
	else{
		wind_angle=(AW(0)>0 ? 180-wind_angle : -wind_angle);
	}
	return wind_angle;
}
