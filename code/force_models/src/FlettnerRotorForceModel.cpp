/*
 * FlettnerRotorForceModel.cpp
 *
 *  Created on: 14 janv. 2020
 *      Author: mcharlou2016
 */

#include <string>
#include <math.h>
#include <Eigen/Dense>
#include <ssc/yaml_parser.hpp>
#include <ssc/kinematics.hpp>
#include <ssc/interpolation.hpp>

#include "ForceModel.hpp"
#include "BodyStates.hpp"
#include "yaml.h"
#include "external_data_structures_parsers.hpp"

#include "FlettnerRotorForceModel.hpp"

#define PI M_PI

std::string FlettnerRotorForceModel::model_name() {return "Flettner rotor";}

class FlettnerRotorForceModel::Impl
{
    public:
        Impl(const std::vector<double>& relative_velocity_, const std::vector<double>& Cl_, const std::vector<double>& Cd_) :
            Cl(relative_velocity_, Cl_),
            Cd(relative_velocity_, Cd_)
        {
        }

        ssc::interpolation::SplineVariableStep Cl;
        ssc::interpolation::SplineVariableStep Cd;

    private:
        Impl();
};

FlettnerRotorForceModel::Yaml::Yaml() :
			name(),
			position_of_rotor_frame(),
			diameter(),
			length(),
			skin_relative_velocity(),
			lift_coefficient(),
			drag_coefficient()
{
}

FlettnerRotorForceModel::FlettnerRotorForceModel(const Yaml& input, const std::string& body_name, const EnvironmentAndFrames& env) :
				ForceModel(input.name, body_name, env, input.position_of_rotor_frame, {"rotation speed"}),
				pimpl(new Impl(input.skin_relative_velocity, input.lift_coefficient, input.drag_coefficient)),
				area(input.diameter*input.length),
				radius(input.diameter/2),
				half_length(input.length/2)
{
}

FlettnerRotorForceModel::Yaml FlettnerRotorForceModel::parse(const std::string& yaml)
{
	std::stringstream stream(yaml);
	YAML::Parser parser(stream);
	YAML::Node node;
	parser.GetNextDocument(node);
	Yaml ret;

	node["name"] >> ret.name;
	ssc::yaml_parser::parse_uv(node["length"], ret.length);
	ssc::yaml_parser::parse_uv(node["diameter"], ret.diameter);
	node["skin relative velocity"] >> ret.skin_relative_velocity;
	node["lift coefficient"] >> ret.lift_coefficient;
	node["drag coefficient"] >> ret.drag_coefficient;
	node["position of rotor"] >> ret.position_of_rotor_frame;

	return ret;
}

ssc::kinematics::Vector6d FlettnerRotorForceModel::get_force(const BodyStates& states, const double t, const EnvironmentAndFrames& env, const std::map<std::string,double>& commands) const
{
	ssc::kinematics::Vector6d force = ssc::kinematics::Vector6d::Zero();

	Eigen::Vector3d application_point = Eigen::Vector3d::Zero();
	application_point(3)=half_length;
	Eigen::Vector3d AW=get_AW(states, t, env);
	const double rotation_speed=commands.at("rotation speed");
	//std::cout << "Rotation speed at t=" << t << "s: " << rotation_speed << std::endl;
	const double AWS=AW.norm();
	const double relative_velocity=abs(rotation_speed*radius/AWS);
	//std::cout << "Relative velocity (u/V): " << relative_velocity << std::endl;
	double Cl=0;
	double Cd=0;
	try
	{
		Cl=pimpl->Cl.f(relative_velocity);
		Cd=pimpl->Cd.f(relative_velocity);
	}
	catch(const ssc::exception_handling::Exception& e)
	{
		THROW(__PRETTY_FUNCTION__, ssc::exception_handling::Exception, "Error while trying to retrieve the lift and drag coefficients from the input with a skin relative velocity of " << relative_velocity << "m/s:\n" << e.get_message());
	}
	const double Lift=(rotation_speed>=0 ? Cl*(0.5*env.air_density*pow(AWS,2)*area) : -Cl*(0.5*env.air_density*pow(AWS,2)*area));
	const double Drag=Cd*(0.5*env.air_density*pow(AWS,2)*area);
	const double theta=atan2(AW(1),AW(0));
	//std::cout << "Direction of wind: " << 180*theta/PI << std::endl;
	//std::cout << "Lift: " << Lift << "	Drag: " << Drag << std::endl;
	force(0) = sin(theta)*Lift+cos(theta)*Drag;
	force(1) = -cos(theta)*Lift+sin(theta)*Drag;
	Eigen::Vector3d force3d = Eigen::Vector3d::Zero();
	force3d = force.segment(0,2);
	//std::cout << "Force in local frame: [" << force3d(0) << "," << force3d(1) << "," << force3d(2) << "]" << std::endl;

	force.segment(3,5)=application_point.cross(force3d);
	return force;
}

Eigen::Vector3d FlettnerRotorForceModel::get_AW(const BodyStates& states, const double t, const EnvironmentAndFrames& env) const
{
	ssc::kinematics::Transform T = env.k->get(states.name, "NED");
	auto rot_from_NED_frame_to_body_frame = T.get_rot();
	T = env.k->get(get_name(), states.name);
	auto rot_from_body_frame_to_internal_frame = T.get_rot();
	T = env.k->get("NED",get_name());
	auto rot_from_internal_frame_to_NED_frame = T.get_rot();
	Eigen::Vector3d center_of_local_frame = env.k->get("NED",get_name()).get_point().v;
	Eigen::Vector3d rotor_axis_in_internal_frame = Eigen::Vector3d::Zero();
	rotor_axis_in_internal_frame(2) = 1;
	Eigen::Vector3d rotor_axis_in_NED_frame = rot_from_internal_frame_to_NED_frame*rotor_axis_in_internal_frame;
	Eigen::Vector3d point_of_application_in_NED = center_of_local_frame+half_length*rotor_axis_in_NED_frame;
	Eigen::Vector3d wind_in_NED_frame=env.wind->get_wind(point_of_application_in_NED,t);
	//std::cout << "TW: [" << wind_in_NED_frame(0) << "," << wind_in_NED_frame(1) << "," << wind_in_NED_frame(2) << "] in NED frame" << std::endl;
	Eigen::Vector3d wind_in_body_frame=rot_from_NED_frame_to_body_frame*wind_in_NED_frame;
	//std::cout << "TW: [" << wind_in_body_frame(0) << "," << wind_in_body_frame(1) << "," << wind_in_body_frame(2) << "] in body frame" << std::endl;
	//Eigen::Vector3d wind_in_internal_frame=rot_from_body_frame_to_internal_frame*wind_in_body_frame;
	//std::cout << "TW: [" << wind_in_internal_frame(0) << "," << wind_in_internal_frame(1) << "," << wind_in_internal_frame(2) << "] in local frame" << std::endl;
	Eigen::Vector3d Vs(states.u(),states.v(),states.w());
	//std::cout << "Wind in NED frame : " << wind_in_NED_frame << std::endl;
	//std::cout << "Wind in body frame : " << wind_in_body_frame << std::endl;
	//std::cout << "Ship speed in body frame : " << Vs << std::endl;
	Eigen::Vector3d AW = rot_from_body_frame_to_internal_frame*(wind_in_body_frame-Vs);
	//std::cout << "AW: [" << AW(0) << "," << AW(1) << "," << AW(2) << "] in local frame located @ [" << center_of_local_frame(0) << "," << center_of_local_frame(1) << "," << center_of_local_frame(2) << "] in NED frame" << std::endl;
	return AW;
}
